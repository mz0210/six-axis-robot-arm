/*
 * trajectory_planner_port.c
 *
 *  Created on: 2026年5月23日
 *      Author: LEGION
 */
#include "tim.h"
#include "trajectory_planner.h"
#include "MotorControl.h"

volatile uint8_t g_traj_tick = 0;

/* 20ms 定时器中断：只置位，不做计算/不打印/不发CAN */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        g_traj_tick = 1;
    }
}

/* 依赖：读取当前关节角（deg） */
void get_current_joint_angles(float joints_deg[TRAJ_JOINT_NUM])
{
    joint_motor_sync_all(200);
    for (int i = 0; i < TRAJ_JOINT_NUM; i++) {
        joints_deg[i] = joint_motor_get_angle((uint8_t)i);
    }
}
