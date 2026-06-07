/* trajectory_planner.c
 *
 * LINEMOVE: Base-frame straight-line position + orientation hold (lock to p0_fk),
 * executed by joint velocity tracking (VEL mode).
 *
 * - 20ms tick update
 * - compute target pose along line (S-curve time law)
 * - solve IK (DLS) to get q_ref
 * - outer-loop P control in joint space -> qdot_cmd (deg/s)
 * - send speed command every 40ms, sync-measure every 100ms
 * - end: stop + low-speed final absolute alignment
 *
 * This module is independent: it only takes effect while TRAJ_RUNNING.
 */

#include "trajectory_planner.h"
#include "MotorControl.h"
#include "robot_ik_dls.h"
#include "robot_kinematics.h"
#include "usart.h"

#include <string.h>
#include <math.h>
#include <stdbool.h>

/* ===== 全局静态状态 ===== */
static volatile traj_state_t g_state = TRAJ_IDLE;

static float g_dt_s = 0.02f;                // 20ms
static uint32_t g_step = 0;
static uint32_t g_total_steps = 0;

static pose_t g_p0, g_p1;
static float g_T = 2.0f;

static uint8_t g_ik_fail_cnt = 0;
#define TRAJ_IK_FAIL_MAX 10

/* 轨迹运动参数（只用于 LINEMOVE 内部） */
static traj_joint_cmd_cfg_t g_joint_cfg = {
    .vel_deg_s = 60.0f,  // （这里不用于VEL模式，仅保留结构）
    .acc = 12,           // 你确认的acc
};

/* 上一周期参考关节角（用于 IK seed 与终点对准） */
static float g_q_prev_cmd[TRAJ_JOINT_NUM] = {0};

/* ===== VEL 跟踪专用（只影响 LINEMOVE） ===== */
static float g_q_est[TRAJ_JOINT_NUM] = {0};        // 关节角估计（deg）
static float g_qdot_last[TRAJ_JOINT_NUM] = {0};    // 上次速度命令（deg/s）

/* 你确认的参数 */
static const float POS_GATE_MM = 1.0f;             // 位置门限：位置达标就允许推进
static const float KP_QDOT = 2.5f;                 // P增益(1/s)
static const float QDOT_MAX_DEG_S = 12.0f;         // 最大关节速度
static const float QDOT_SLEW_DEG_S2 = 120.0f;      // 速度变化率上限（防撞机）

static const uint32_t RESEND_VEL_EVERY = 2;        // 40ms重发速度（2*20ms）
static const uint32_t SYNC_MEAS_EVERY  = 5;        // 100ms回读校正（5*20ms）
static const float FINAL_ALIGN_VEL_DEG_S = 5.0f;   // 终点对准低速

/* ===== 工具函数 ===== */

static float clamp01(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

/* 角度归一化到 (-180, 180] */
static float wrap_deg_180(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

/* 五次S曲线：0->1，首尾速度为0 */
static float s_curve_5th(float t)
{
    // 10t^3 - 15t^4 + 6t^5
    return t*t*t*(10.0f + t*(-15.0f + 6.0f*t));
}

/* 位姿插值：位置线性；姿态用 RPY 最短路径线性插值 */
static float shortest_delta_deg(float a_deg, float b_deg)
{
    return wrap_deg_180(b_deg - a_deg);
}

static void pose_lerp_rpy_shortest(const pose_t *a, const pose_t *b, float s, pose_t *out)
{
    out->x = a->x + (b->x - a->x) * s;
    out->y = a->y + (b->y - a->y) * s;
    out->z = a->z + (b->z - a->z) * s;

    float dr = shortest_delta_deg(a->roll,  b->roll);
    float dp = shortest_delta_deg(a->pitch, b->pitch);
    float dy = shortest_delta_deg(a->yaw,   b->yaw);

    out->roll  = a->roll  + dr * s;
    out->pitch = a->pitch + dp * s;
    out->yaw   = a->yaw   + dy * s;

    pose_set_rpy_deg(out);
}

/* ===== API ===== */

void trajectory_init(float dt_s)
{
    if (dt_s > 0.001f && dt_s < 0.2f) g_dt_s = dt_s;
    else g_dt_s = 0.02f;

    g_state = TRAJ_IDLE;
    g_step = 0;
    g_total_steps = 0;
    g_ik_fail_cnt = 0;

    memset(g_q_prev_cmd, 0, sizeof(g_q_prev_cmd));
    memset(g_q_est, 0, sizeof(g_q_est));
    memset(g_qdot_last, 0, sizeof(g_qdot_last));
}

traj_state_t trajectory_get_state(void)
{
    return g_state;
}

bool start_linear_move(const pose_t *p0, const pose_t *p1, float total_time_s)
{
    if (!p0 || !p1 || total_time_s <= 0.05f) {
        g_state = TRAJ_ERROR_PARAM;
        return false;
    }

    /* 复制输入 */
    g_p0 = *p0;
    g_p1 = *p1;

    /* 保证输入R有效（后面会覆盖姿态，但不影响） */
    pose_set_rpy_deg(&g_p0);
    pose_set_rpy_deg(&g_p1);

    g_T = total_time_s;
    g_total_steps = (uint32_t)ceilf(g_T / g_dt_s);
    if (g_total_steps < 1) g_total_steps = 1;

    /* 读取当前关节角（seed） */
    float q_seed[TRAJ_JOINT_NUM];
    get_current_joint_angles(q_seed);

    /* ✅ 隔离：启动 LINEMOVE 前先强制停止所有电机（避免其它模式残留运动） */
    for (int i = 0; i < TRAJ_JOINT_NUM; i++) {
        joint_motor_vel_deg_s((uint8_t)i, 0.0f, g_joint_cfg.acc);
    }

    /* 起点 IK：用 DLS 求一个更稳的 q0，用于锁姿态与作为初始seed */
    float q0[TRAJ_JOINT_NUM];
    int iter0 = 0;
    float pos_err0 = 0.0f, ori_err0 = 0.0f;
    bool ok0 = ikine_dls_deg_ex(&g_p0, q_seed, q0, &iter0, &pos_err0, &ori_err0);
    if (!ok0 && pos_err0 > POS_GATE_MM) {
        g_state = TRAJ_ERROR_IK;
        LOG("[TRAJ] IK(p0) failed: iter=%d pos_err=%.3f ori_err=%.3f\r\n",
            iter0, pos_err0, ori_err0);
        return false;
    }

    memcpy(g_q_prev_cmd, q0, sizeof(g_q_prev_cmd));
    memcpy(g_q_est, q0, sizeof(g_q_est));
    memset(g_qdot_last, 0, sizeof(g_qdot_last));

    /* ===== 姿态保持：锁定为 p0_fk ===== */
    pose_t p0_fk;
    fkine_deg(q0, &p0_fk);

    g_p0.roll  = p0_fk.roll;
    g_p0.pitch = p0_fk.pitch;
    g_p0.yaw   = p0_fk.yaw;

    g_p1.roll  = p0_fk.roll;
    g_p1.pitch = p0_fk.pitch;
    g_p1.yaw   = p0_fk.yaw;

    /* ✅ 避开 RPY 奇异（仅影响内部R矩阵生成） */
    if (g_p0.pitch <= -89.9f) g_p0.pitch = -89.0f;
    if (g_p0.pitch >=  89.9f) g_p0.pitch =  89.0f;
    g_p1.pitch = g_p0.pitch;

    pose_set_rpy_deg(&g_p0);
    pose_set_rpy_deg(&g_p1);

    /* 状态启动 */
    g_step = 0;
    g_ik_fail_cnt = 0;
    g_state = TRAJ_RUNNING;

    LOG("[TRAJ] start LINEMOVE(VEL): T=%.3fs dt=%.3fs steps=%lu\r\n",
        g_T, g_dt_s, (unsigned long)g_total_steps);
    LOG("[TRAJ] lock RPY to p0_fk(clamped): (%.2f %.2f %.2f)\r\n",
        g_p0.roll, g_p0.pitch, g_p0.yaw);

    return true;
}

void trajectory_timer_callback(void)
{
    if (g_state != TRAJ_RUNNING) return;

    /* step */
    g_step++;
    float t = clamp01((float)g_step / (float)g_total_steps);
    float s = s_curve_5th(t);

    /* target pose (line pos + locked orientation) */
    pose_t target_pose;
    pose_lerp_rpy_shortest(&g_p0, &g_p1, s, &target_pose);

    /* IK -> q_ref */
    float q_ref[TRAJ_JOINT_NUM];
    int iter = 0;
    float pos_err = 0.0f, ori_err = 0.0f;
    bool ok = ikine_dls_deg_ex(&target_pose, g_q_prev_cmd, q_ref, &iter, &pos_err, &ori_err);

    /* 方案A：只把“位置跟不上”当作失败 */
    if (!ok && pos_err > POS_GATE_MM) {
        g_ik_fail_cnt++;
        if ((g_ik_fail_cnt % 2) == 0) {
            LOG("[TRAJ] IK bad(pos) #%d step=%lu t=%.3f pos_err=%.3fmm ori_err=%.3fdeg\r\n",
                (int)g_ik_fail_cnt, (unsigned long)g_step, t, pos_err, ori_err);
        }

        if (g_ik_fail_cnt < TRAJ_IK_FAIL_MAX) {
            return; // 本周期不推进（不更新速度/不发命令）
        }

        /* 超限：立即停 */
        for (int i = 0; i < TRAJ_JOINT_NUM; i++) {
            joint_motor_vel_deg_s((uint8_t)i, 0.0f, g_joint_cfg.acc);
        }
        g_state = TRAJ_ERROR_IK;
        return;
    }

    g_ik_fail_cnt = 0;

    /* 每100ms回读一次，校正估计角 */
    if ((g_step % SYNC_MEAS_EVERY) == 0) {
        joint_motor_sync_all(200);
        for (int i = 0; i < TRAJ_JOINT_NUM; i++) {
            g_q_est[i] = joint_motor_get_angle((uint8_t)i);
        }
    }

    /* 关节空间速度命令：qdot = Kp * (q_ref - q_est) */
    float qdot_cmd[TRAJ_JOINT_NUM];
    for (int i = 0; i < TRAJ_JOINT_NUM; i++) {
        float err = wrap_deg_180(q_ref[i] - g_q_est[i]);
        float v = KP_QDOT * err;

        if (v >  QDOT_MAX_DEG_S) v =  QDOT_MAX_DEG_S;
        if (v < -QDOT_MAX_DEG_S) v = -QDOT_MAX_DEG_S;

        qdot_cmd[i] = v;
    }

    /* 速度变化限幅（slew-rate） */
    for (int i = 0; i < TRAJ_JOINT_NUM; i++) {
        float dv_max = QDOT_SLEW_DEG_S2 * g_dt_s;
        float dv = qdot_cmd[i] - g_qdot_last[i];

        if (dv >  dv_max) qdot_cmd[i] = g_qdot_last[i] + dv_max;
        if (dv < -dv_max) qdot_cmd[i] = g_qdot_last[i] - dv_max;

        g_qdot_last[i] = qdot_cmd[i];
    }

    /* 估计角积分预测（20ms） */
    for (int i = 0; i < TRAJ_JOINT_NUM; i++) {
        g_q_est[i] += qdot_cmd[i] * g_dt_s;
    }

    /* 40ms重发速度命令（第1步也发，避免起步等待） */
    if ((g_step % RESEND_VEL_EVERY) == 0 || g_step == 1) {
        for (int i = 0; i < TRAJ_JOINT_NUM; i++) {
            joint_motor_vel_deg_s((uint8_t)i, qdot_cmd[i], g_joint_cfg.acc);
        }
    }

    /* 更新seed */
    memcpy(g_q_prev_cmd, q_ref, sizeof(g_q_prev_cmd));

    /* done: stop + final align */
    if (g_step >= g_total_steps) {

        for (int i = 0; i < TRAJ_JOINT_NUM; i++) {
            joint_motor_vel_deg_s((uint8_t)i, 0.0f, g_joint_cfg.acc);
        }

        /* 终点低速对准一次（用最后 q_ref） */
        for (int i = 0; i < TRAJ_JOINT_NUM; i++) {
            joint_motor_move_abs_deg((uint8_t)i, g_q_prev_cmd[i], FINAL_ALIGN_VEL_DEG_S, g_joint_cfg.acc);
            // 这里不做HAL_Delay，避免你工程没包含HAL；如需要节流可在MotorControl内部做
        }

        g_state = TRAJ_DONE;
        LOG("[TRAJ] done (VEL + final align)\r\n");
    }
}
