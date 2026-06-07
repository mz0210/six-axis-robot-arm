/*
 * app_robot.c
 *
 *  Created on: 2026年5月21日
 *      Author: LEGION
 */


#include "app_robot.h"
#include "MotorControl.h"
#include "usart.h"
#include "main.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

bool app_parse_cmd_angles(const char *s, float q_out[APP_JOINT_NUM])
{
    char buf[128];
    strncpy(buf, s, sizeof(buf)-1);
    buf[sizeof(buf)-1] = 0;

    for (size_t i = 0; i < strlen(buf); i++) {
        if (buf[i] == ',') buf[i] = ' ';
    }

    int n = sscanf(buf, "J1=%f J2=%f J3=%f J4=%f J5=%f J6=%f",
                   &q_out[0], &q_out[1], &q_out[2],
                   &q_out[3], &q_out[4], &q_out[5]);
    return (n == 6);
}

bool app_parse_cmd_pose(const char *s, pose_t *pose_out)
{
    char buf[160];
    strncpy(buf, s, sizeof(buf)-1);
    buf[sizeof(buf)-1] = 0;

    for (size_t i = 0; i < strlen(buf); i++) {
        if (buf[i] == ',') buf[i] = ' ';
    }

    int n = sscanf(buf,
        "X=%f Y=%f Z=%f Roll=%f Pitch=%f Yaw=%f",
        &pose_out->x, &pose_out->y, &pose_out->z,
        &pose_out->roll, &pose_out->pitch, &pose_out->yaw);

    return (n == 6);
}

void app_print_angles(const char *tag)
{
    joint_motor_sync_all(200);
    LOG("[%s] J1=%.2f J2=%.2f J3=%.2f J4=%.2f J5=%.2f J6=%.2f\r\n",
        tag,
        joint_motor_get_angle(0), joint_motor_get_angle(1),
        joint_motor_get_angle(2), joint_motor_get_angle(3),
        joint_motor_get_angle(4), joint_motor_get_angle(5));
}

void app_print_pose(const char *tag, const pose_t *p)
{
    LOG("[%s] POS(%.2f, %.2f, %.2f) RPY(%.2f, %.2f, %.2f)\r\n",
        tag, p->x, p->y, p->z, p->roll, p->pitch, p->yaw);
}
void app_print_angles_and_pose(const char *tag)
{
    // 1) 回读关节角
    joint_motor_sync_all(200);

    float q[APP_JOINT_NUM];
    for (int i = 0; i < APP_JOINT_NUM; i++) {
        q[i] = joint_motor_get_angle((uint8_t)i);
    }

    // 2) 打印关节角
    LOG("[%s] J1=%.2f J2=%.2f J3=%.2f J4=%.2f J5=%.2f J6=%.2f\r\n",
        tag, q[0], q[1], q[2], q[3], q[4], q[5]);

    // 3) 正解得到位姿并打印
    pose_t p;
    fkine_deg(q, &p);
    app_print_pose(tag, &p);
}
// 五次S曲线 (0->1)
static float s_curve_5th(float t)
{
    // 10t^3 - 15t^4 + 6t^5
    return t*t*t*(10.0f + t*(-15.0f + 6.0f*t));
}

void app_move_joints_scurve_stable(const float q_start[APP_JOINT_NUM],
                                   const float q_target[APP_JOINT_NUM],
                                   const app_motion_cfg_t *cfg)
{
    if (!cfg) return;
    LOG("[SCURVE] steps=%d T=%.2fs vel=%.2f acc=%u sync_every=%d\r\n",
        cfg->steps, cfg->total_time_s, cfg->vel_deg_s, cfg->acc, cfg->sync_every);
    float q_prev[APP_JOINT_NUM];
    float q_next[APP_JOINT_NUM];
    memcpy(q_prev, q_start, sizeof(q_prev));

    int steps = cfg->steps;
    if (steps < 5) steps = 5;

    uint32_t dt_ms = (uint32_t)(cfg->total_time_s * 1000.0f / (float)steps);
    if (dt_ms < 10) dt_ms = 10;

    for (int s = 1; s <= steps; s++) {
        float t = (float)s / (float)steps;
        float sc = s_curve_5th(t);
        if ((s % 10) == 0) {
            joint_motor_sync_all(200);
            float j1 = joint_motor_get_angle(0);
            LOG("[SCURVE] s=%d q_next_j1=%.2f q_meas_j1=%.2f dt_ms=%lu\r\n",
                s, q_next[0], j1, (unsigned long)dt_ms);
        }
        for (int i = 0; i < APP_JOINT_NUM; i++) {
            q_next[i] = q_start[i] + (q_target[i] - q_start[i]) * sc;
        }

        for (int i = 0; i < APP_JOINT_NUM; i++) {
            float d = q_next[i] - q_prev[i];
            joint_motor_move_rel_deg((uint8_t)i, d, cfg->vel_deg_s, cfg->acc);
            HAL_Delay(2);
        }

        HAL_Delay(dt_ms);

        if (cfg->sync_every > 0 && (s % cfg->sync_every) == 0) {
            joint_motor_sync_all(200);
            for (int i = 0; i < APP_JOINT_NUM; i++) {
                q_prev[i] = joint_motor_get_angle((uint8_t)i);
            }
        } else {
            memcpy(q_prev, q_next, sizeof(q_prev));
        }
    }

    // 最终纠偏：确保到位
    joint_motor_sync_all(200);
    for (int i = 0; i < APP_JOINT_NUM; i++) {
        float cur = joint_motor_get_angle((uint8_t)i);
        float delta = q_target[i] - cur;
        joint_motor_move_rel_deg((uint8_t)i, delta, cfg->vel_deg_s, cfg->acc);
        HAL_Delay(2);
    }
}
