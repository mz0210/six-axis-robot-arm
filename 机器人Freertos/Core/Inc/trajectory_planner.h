/*
 * trajectory_planner.h
 *
 *  Created on: 2026年5月23日
 *      Author: LEGION
 */

#ifndef INC_TRAJECTORY_PLANNER_H_
#define INC_TRAJECTORY_PLANNER_H_



#endif /* INC_TRAJECTORY_PLANNER_H_ */
#ifndef __TRAJECTORY_PLANNER_H__
#define __TRAJECTORY_PLANNER_H__

#include <stdbool.h>
#include <stdint.h>
#include "robot_kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TRAJ_JOINT_NUM 6

typedef enum {
    TRAJ_IDLE = 0,
    TRAJ_RUNNING,
    TRAJ_DONE,
    TRAJ_ERROR_IK,
    TRAJ_ERROR_PARAM,
} traj_state_t;

typedef struct {
    float vel_deg_s;     // 关节速度(deg/s)，传给 joint_motor_move_rel_deg
    uint8_t acc;         // 驱动加速度参数(0~255), 0直接启动
} traj_joint_cmd_cfg_t;

/** 初始化：设置周期dt（默认20ms）、默认关节速度/加速度也可在.c中调整 */
void trajectory_init(float dt_s);

/** 启动末端直线运动：从 p0 -> p1，持续 T 秒 */
bool start_linear_move(const pose_t *p0, const pose_t *p1, float total_time_s);

/** 查询轨迹状态 */
traj_state_t trajectory_get_state(void);

/** 定时器中断/周期回调：每 dt 调用一次 */
void trajectory_timer_callback(void);

/**
 * 依赖（你需要提供实现）：
 * 读取当前关节角（deg）。你可以基于CAN回读实现。
 *
 * 注意：本模块只在 start_linear_move() 调一次读取初值；
 * 不会在定时器中断中阻塞式回读，避免卡中断。
 */
void get_current_joint_angles(float joints_deg[TRAJ_JOINT_NUM]);

#ifdef __cplusplus
}
#endif

#endif
