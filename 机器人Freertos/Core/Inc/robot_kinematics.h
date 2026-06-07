#ifndef __ROBOT_KINEMATICS_H__
#define __ROBOT_KINEMATICS_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define JOINT_NUM 6

typedef struct {
    float x, y, z;         // mm
    float roll, pitch, yaw; // deg (固定轴X/Y/Z)
    float R[3][3];         // 旋转矩阵
} pose_t;

/**
 * @brief 正解：关节角 -> 末端位姿
 * @param q_deg 关节角输入（度）
 * @param out_pose 输出位姿
 */
void fkine_deg(const float q_deg[JOINT_NUM], pose_t *out_pose);
void pose_set_rpy_deg(pose_t *p);
/**
 * @brief 逆解：末端位姿 -> 关节角（最接近当前关节角）
 * @param target 目标位姿（x,y,z,roll,pitch,yaw，单位mm/deg）
 * @param q_cur_deg 当前关节角（度）
 * @param q_out_deg 输出关节角（度）
 * @return true 成功，false 失败（此时q_out_deg应保持上一次解）
 */
bool ikine_deg(const pose_t *target,
               const float q_cur_deg[JOINT_NUM],
               float q_out_deg[JOINT_NUM]);
/* ✅ 新增：带迭代次数/误差输出的逆解 */
bool ikine_deg_ex(const pose_t *target,
const float q_cur_deg[JOINT_NUM],
float q_out_deg[JOINT_NUM],
int *out_iter,
float *out_pos_err,
float *out_ori_err);

#ifdef __cplusplus
}
#endif
#endif
