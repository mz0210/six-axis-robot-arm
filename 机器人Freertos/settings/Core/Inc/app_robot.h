/*
 * app_robot.h
 *
 *  Created on: 2026年5月21日
 *      Author: LEGION
 */

#ifndef INC_APP_ROBOT_H_
#define INC_APP_ROBOT_H_



#endif /* INC_APP_ROBOT_H_ */
#ifndef __APP_ROBOT_H__
#define __APP_ROBOT_H__

#include <stdbool.h>
#include <stdint.h>
#include "robot_kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_JOINT_NUM 6

typedef struct {
    int steps;              // 插补段数
    float total_time_s;     // 总时长(s)
    float vel_deg_s;        // 关节速度(deg/s) -> MotorControl里会映射到驱动速度
    uint8_t acc;            // 驱动加速度参数(0~255), 0为直接启动
    int sync_every;         // 每N段回读校正
} app_motion_cfg_t;

/** 解析 "J1=.. J2=.. ... J6=.." */
bool app_parse_cmd_angles(const char *s, float q_out[APP_JOINT_NUM]);

/** 解析 "X=.. Y=.. Z=.. Roll=.. Pitch=.. Yaw=.." */
bool app_parse_cmd_pose(const char *s, pose_t *pose_out);

/** 打印当前6轴角度（内部会回读一次） */
void app_print_angles(const char *tag);

/** 打印位姿 */
void app_print_pose(const char *tag, const pose_t *p);
void app_print_angles_and_pose(const char *tag);
/**
 * 稳定版运动：五次S曲线 + 每N段回读校正 + 最终纠偏
 * q_start/q_target 单位：deg
 */
void app_move_joints_scurve_stable(const float q_start[APP_JOINT_NUM],
                                   const float q_target[APP_JOINT_NUM],
                                   const app_motion_cfg_t *cfg);

#ifdef __cplusplus
}
#endif

#endif
