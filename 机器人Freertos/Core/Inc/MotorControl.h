#ifndef __JOINT_MOTOR_H__
#define __JOINT_MOTOR_H__

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define JOINT_MOTOR_MAX_NUM 6

typedef enum {
    JOINT_OK = 0,
    JOINT_ERR_PARAM = -1,
    JOINT_ERR_RANGE = -2
} joint_ret_t;

typedef struct {
    uint8_t  can_addr;
    uint8_t  positive_dir;
    float    reduction_ratio;
    float    min_angle_deg;
    float    max_angle_deg;
    float    current_angle_deg;
} joint_motor_t;

void joint_motor_init_all(joint_motor_t *cfg, uint8_t num);
joint_ret_t joint_motor_enable(uint8_t jid, bool en);
joint_ret_t joint_motor_stop(uint8_t jid);
joint_ret_t joint_motor_reset_zero(uint8_t jid);
joint_ret_t joint_motor_move_rel_deg(uint8_t jid, float delta_deg, float vel_deg_s, uint8_t acc);
joint_ret_t joint_motor_move_abs_deg(uint8_t jid, float target_deg, float vel_deg_s, uint8_t acc);
float joint_motor_get_angle(uint8_t jid);
joint_ret_t joint_motor_move_rel_deg_sn(uint8_t jid, float delta_deg, float vel_deg_s, uint8_t acc, bool snF);
joint_ret_t joint_motor_move_abs_deg_sn(uint8_t jid, float target_deg, float vel_deg_s, uint8_t acc, bool snF);
/* 新增：回读同步 */
bool joint_motor_sync_angle(uint8_t jid, uint32_t timeout_ms);
void joint_motor_sync_all(uint32_t timeout_ms);
void joint_motor_on_can_frame(uint32_t extid, const uint8_t *data, uint8_t dlc);
joint_ret_t joint_motor_vel_deg_s(uint8_t jid, float vel_deg_s, uint8_t acc);
joint_ret_t joint_motor_vel_deg_s_sn(uint8_t jid, float vel_deg_s, uint8_t acc, bool snF);




#ifdef __cplusplus
}
#endif

#endif
