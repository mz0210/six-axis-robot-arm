#include "MotorControl.h"
#include "Emm_V5.h"
#include "usart.h"
#include <math.h>

static joint_motor_t g_joints[JOINT_MOTOR_MAX_NUM];
static uint8_t g_joint_num = 0;
static volatile uint8_t g_angle_updated[JOINT_MOTOR_MAX_NUM] = {0};

/**
 * @brief 角度 -> 脉冲数
 * 16细分下：电机1圈=3200脉冲
 * 关节角度换算：steps = |deg| * ratio * 3200 / 360
 */
static uint32_t deg_to_steps(float deg, float ratio)
{
    float steps = fabsf(deg) * ratio * 3200.0f / 360.0f;
    return (uint32_t)(steps + 0.5f);
}

/**
 * @brief 关节角速度(deg/s) -> 驱动速度参数
 * rpm = deg/s * 60 * ratio / 360
 * drv = rpm * 10
 */
static uint16_t degs_to_drv_vel(float deg_s, float ratio)
{
    float rpm = fabsf(deg_s) * 60.0f * ratio / 360.0f;

    if (rpm < 1.0f) rpm = 1.0f;
    if (rpm > 5000.0f) rpm = 5000.0f;
    return (uint16_t)(rpm + 0.5f);
}

static int valid_jid(uint8_t jid)
{
    return (jid < g_joint_num);
}

void joint_motor_init_all(joint_motor_t *cfg, uint8_t num)
{
    if (num > JOINT_MOTOR_MAX_NUM) num = JOINT_MOTOR_MAX_NUM;
    g_joint_num = num;
    for (uint8_t i = 0; i < g_joint_num; i++) {
        g_joints[i] = cfg[i];
    }
}

joint_ret_t joint_motor_enable(uint8_t jid, bool en)
{
    if (!valid_jid(jid)) return JOINT_ERR_PARAM;
    Emm_V5_En_Control(g_joints[jid].can_addr, en, false);
    return JOINT_OK;
}

joint_ret_t joint_motor_stop(uint8_t jid)
{
    if (!valid_jid(jid)) return JOINT_ERR_PARAM;
    Emm_V5_Stop_Now(g_joints[jid].can_addr, false);
    return JOINT_OK;
}

joint_ret_t joint_motor_reset_zero(uint8_t jid)
{
    if (!valid_jid(jid)) return JOINT_ERR_PARAM;
    Emm_V5_Reset_CurPos_To_Zero(g_joints[jid].can_addr);
    g_joints[jid].current_angle_deg = 0.0f;
    return JOINT_OK;
}

joint_ret_t joint_motor_move_rel_deg(uint8_t jid, float delta_deg, float vel_deg_s, uint8_t acc)
{

	return joint_motor_move_rel_deg_sn(jid, delta_deg, vel_deg_s, acc, false
	);
}

joint_ret_t joint_motor_move_abs_deg(uint8_t jid, float target_deg, float vel_deg_s, uint8_t acc)
{
    return joint_motor_move_abs_deg_sn(jid, target_deg, vel_deg_s, acc, false
);
}

float joint_motor_get_angle(uint8_t jid)
{
    if (!valid_jid(jid)) return 0.0f;
    return g_joints[jid].current_angle_deg;
}

/* ===== 回读同步 ===== */
void joint_motor_on_can_frame(uint32_t extid, const uint8_t *data, uint8_t dlc)
{
    if (dlc < 7) return;

    // 只收分包0帧
    if ((extid & 0xFF) != 0) return;

    uint8_t addr = (uint8_t)(extid >> 8);

    // 只处理CPOS回包
    if (data[0] != 0x36 || data[6] != 0x6B) return;

    uint8_t jid = 0xFF;
    for (uint8_t i = 0; i < g_joint_num; i++) {
        if (g_joints[i].can_addr == addr) { jid = i; break; }
    }
    if (jid == 0xFF) return;

    uint32_t raw = ((uint32_t)data[2] << 24) |
                   ((uint32_t)data[3] << 16) |
                   ((uint32_t)data[4] << 8)  |
                    (uint32_t)data[5];

    // ✅ 正确解析：编码器计数(65536/圈)
    float motor_deg = raw * 360.0f / 65536.0f;
    if (data[1] == 0x01) motor_deg = -motor_deg;

    // ✅ 映射关节正方向
    if (g_joints[jid].positive_dir == 1) {
        motor_deg = -motor_deg;
    }

    float joint_deg = motor_deg / g_joints[jid].reduction_ratio;

    g_joints[jid].current_angle_deg = joint_deg;
    g_angle_updated[jid] = 1;
}

bool joint_motor_sync_angle(uint8_t jid, uint32_t timeout_ms)
{
    if (!valid_jid(jid)) return false;

    g_angle_updated[jid] = 0;
    Emm_V5_Read_Sys_Params(g_joints[jid].can_addr, S_CPOS);

    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (g_angle_updated[jid]) return true;
    }
    return false;
}

void joint_motor_sync_all(uint32_t timeout_ms)
{
    for (uint8_t i = 0; i < g_joint_num; i++) {
        joint_motor_sync_angle(i, timeout_ms);
    }
}
joint_ret_t joint_motor_move_rel_deg_sn(uint8_t jid, float delta_deg, float vel_deg_s, uint8_t acc, bool snF)
{
    if (!valid_jid(jid)) return JOINT_ERR_PARAM;
    if (fabsf(delta_deg) < 1e-4f) return JOINT_OK;

    joint_motor_t *j = &g_joints[jid];

    float target = j->current_angle_deg + delta_deg;
    if (target < j->min_angle_deg || target > j->max_angle_deg) {
        return JOINT_ERR_RANGE;
    }

    uint8_t joint_positive = (delta_deg >= 0.0f) ? 1 : 0;
    uint8_t motor_dir = joint_positive ? j->positive_dir : !j->positive_dir;

    uint32_t steps = deg_to_steps(delta_deg, j->reduction_ratio);
    uint16_t vel   = degs_to_drv_vel(vel_deg_s, j->reduction_ratio);

    // 关键：snF由外部控制
    Emm_V5_Pos_Control(j->can_addr, motor_dir, vel, acc, steps, false, snF);

    // 不要在这里更新 current_angle_deg，current_angle_deg 仅由回读更新


    /*j->current_angle_deg = target;*/
    return JOINT_OK;
}

joint_ret_t joint_motor_move_abs_deg_sn(uint8_t jid, float target_deg, float vel_deg_s, uint8_t acc, bool snF)
{
    if (!valid_jid(jid)) return JOINT_ERR_PARAM;

    joint_motor_t *j = &g_joints[jid];
    if (target_deg < j->min_angle_deg || target_deg > j->max_angle_deg) {
        return JOINT_ERR_RANGE;
    }

    float delta = target_deg - j->current_angle_deg;
    return joint_motor_move_rel_deg_sn(jid, delta, vel_deg_s, acc, snF);
}
joint_ret_t joint_motor_vel_deg_s(uint8_t jid, float vel_deg_s, uint8_t acc)
{
    return joint_motor_vel_deg_s_sn(jid, vel_deg_s, acc, false);
}

joint_ret_t joint_motor_vel_deg_s_sn(uint8_t jid, float vel_deg_s, uint8_t acc, bool snF)
{
    if (!valid_jid(jid)) return JOINT_ERR_PARAM;

    joint_motor_t *j = &g_joints[jid];

    // vel≈0：停
    if (fabsf(vel_deg_s) < 1e-3f) {
        Emm_V5_Stop_Now(j->can_addr, snF);
        return JOINT_OK;
    }

    // joint deg/s -> motor rpm
    float motor_rpm = fabsf(vel_deg_s) * j->reduction_ratio * 60.0f / 360.0f;

    // 限幅到驱动范围
    if (motor_rpm < 1.0f) motor_rpm = 1.0f;
    if (motor_rpm > 5000.0f) motor_rpm = 5000.0f;

    // 关节正方向：vel>0
    uint8_t joint_positive = (vel_deg_s > 0.0f) ? 1 : 0;
    uint8_t motor_dir = joint_positive ? j->positive_dir : !j->positive_dir;

    // 你确认速度模式单位先按 RPM（不是RPM*10）
    uint16_t vel_param = (uint16_t)(motor_rpm + 0.5f);

    Emm_V5_Vel_Control(j->can_addr, motor_dir, vel_param, acc, snF);
    return JOINT_OK;
}
