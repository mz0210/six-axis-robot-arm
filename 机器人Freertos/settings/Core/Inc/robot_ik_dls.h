#ifndef __ROBOT_IK_DLS_H__
#define __ROBOT_IK_DLS_H__

#include <stdbool.h>
#include "robot_kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DLS/LM数值逆解（更稳，抗奇异），单位：deg
 * @param target 目标位姿（x,y,z mm；R 已经有效）
 * @param q_seed_deg 初值（deg），推荐上一周期命令角
 * @param q_out_deg 输出解（deg）
 * @param out_iter 迭代次数（可NULL）
 * @param out_pos_err 位置误差(mm)（可NULL）
 * @param out_ori_err 姿态误差(deg)（可NULL）
 * @return true 达到阈值；false 未达到阈值（但仍输出 best-effort 解）
 *
 * 注意：该函数不修改你现有 ikine_deg_ex()，是LINEMOVE专用“长期正确版”IK。
 */
bool ikine_dls_deg_ex(const pose_t *target,
                      const float q_seed_deg[JOINT_NUM],
                      float q_out_deg[JOINT_NUM],
                      int *out_iter,
                      float *out_pos_err,
                      float *out_ori_err);

#ifdef __cplusplus
}
#endif
#endif
