#include "robot_kinematics.h"
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846f
#define DEG2RAD(x) ((x) * PI / 180.0f)
#define RAD2DEG(x) ((x) * 180.0f / PI)

// ======= 改进DH参数表（单位：mm / deg） =======
// i | a_{i-1} | alpha_{i-1} | d_i | theta_offset
static const float a_[6]     = {0, 0, 200, 47.63f, 0, 0};
static const float alpha_[6] = {0, 90, 180, -90, 90, 90};
static const float d_[6]     = {0, 0, 0, -184.5f, 0, 0};
static const float th0_[6]   = {90, 90, -90, 0, 90, 0};

// 关节限位（deg）
static const float q_min_[6] = {0, -90, 0, 0, -90, 0};
static const float q_max_[6] = {360, 90, 180, 360, 0, 360};

// ======= 基础矩阵工具 =======
static void mat4_mul(const float A[4][4], const float B[4][4], float C[4][4]) {
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            C[i][j]=0;
            for(int k=0;k<4;k++) C[i][j]+=A[i][k]*B[k][j];
        }
    }
}

static void rot_rpy_to_R(float roll, float pitch, float yaw, float R[3][3]) {
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    float cr = cosf(roll), sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw), sy = sinf(yaw);

    R[0][0]=cy*cp;  R[0][1]=cy*sp*sr - sy*cr;  R[0][2]=cy*sp*cr + sy*sr;
    R[1][0]=sy*cp;  R[1][1]=sy*sp*sr + cy*cr;  R[1][2]=sy*sp*cr - cy*sr;
    R[2][0]=-sp;    R[2][1]=cp*sr;             R[2][2]=cp*cr;
}

static void rot_R_to_rpy(const float R[3][3], float *roll, float *pitch, float *yaw) {
    // 逆运算：固定轴X/Y/Z
    *pitch = atan2f(-R[2][0], sqrtf(R[0][0]*R[0][0] + R[1][0]*R[1][0]));
    *roll  = atan2f(R[2][1], R[2][2]);
    *yaw   = atan2f(R[1][0], R[0][0]);
}

static void mdh_transform(float a, float alpha, float d, float theta, float T[4][4]) {
    // 改进DH：Rot(x,alpha) * Trans(x,a) * Rot(z,theta) * Trans(z,d)
    float ca = cosf(alpha), sa = sinf(alpha);
    float ct = cosf(theta), st = sinf(theta);

    T[0][0] = ct;    T[0][1] = -st;    T[0][2] = 0;     T[0][3] = a;
    T[1][0] = st*ca; T[1][1] = ct*ca;  T[1][2] = -sa;  T[1][3] = -d*sa;
    T[2][0] = st*sa; T[2][1] = ct*sa;  T[2][2] = ca;   T[2][3] = d*ca;
    T[3][0] = 0;     T[3][1] = 0;      T[3][2] = 0;    T[3][3] = 1;
}

void fkine_deg(const float q_deg[JOINT_NUM], pose_t *out_pose)
{
    float T[4][4], A[4][4], tmp[4][4];

    // 初始化为单位矩阵
    memset(T, 0, sizeof(T));
    for(int i=0;i<4;i++) T[i][i]=1;

    for(int i=0;i<6;i++) {
        float theta = DEG2RAD(q_deg[i] + th0_[i]);
        mdh_transform(a_[i], DEG2RAD(alpha_[i]), d_[i], theta, A);
        mat4_mul(T, A, tmp);
        memcpy(T, tmp, sizeof(tmp));
    }

    out_pose->x = T[0][3];
    out_pose->y = T[1][3];
    out_pose->z = T[2][3];

    out_pose->R[0][0]=T[0][0]; out_pose->R[0][1]=T[0][1]; out_pose->R[0][2]=T[0][2];
    out_pose->R[1][0]=T[1][0]; out_pose->R[1][1]=T[1][1]; out_pose->R[1][2]=T[1][2];
    out_pose->R[2][0]=T[2][0]; out_pose->R[2][1]=T[2][1]; out_pose->R[2][2]=T[2][2];

    float r,p,y;
    rot_R_to_rpy(out_pose->R, &r, &p, &y);
    out_pose->roll  = RAD2DEG(r);
    out_pose->pitch = RAD2DEG(p);
    out_pose->yaw   = RAD2DEG(y);
}

// ======= 逆解（数值法：Jacobian Transpose） =======
static void clamp_joint(float q[6]) {
    for(int i=0;i<6;i++){
        if(q[i]<q_min_[i]) q[i]=q_min_[i];
        if(q[i]>q_max_[i]) q[i]=q_max_[i];
    }
}

static void calc_error(const pose_t *cur, const pose_t *tar, float e[6])
{
    e[0] = tar->x - cur->x;
    e[1] = tar->y - cur->y;
    e[2] = tar->z - cur->z;

    // 旋转误差向量（小角度近似）
    float R_err[3][3];
    float Rt[3][3];

    // Rt = R_cur^T
    Rt[0][0]=cur->R[0][0]; Rt[0][1]=cur->R[1][0]; Rt[0][2]=cur->R[2][0];
    Rt[1][0]=cur->R[0][1]; Rt[1][1]=cur->R[1][1]; Rt[1][2]=cur->R[2][1];
    Rt[2][0]=cur->R[0][2]; Rt[2][1]=cur->R[1][2]; Rt[2][2]=cur->R[2][2];

    // R_err = R_tar * R_cur^T
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            R_err[i][j]=0;
            for(int k=0;k<3;k++) R_err[i][j]+=tar->R[i][k]*Rt[k][j];
        }
    }

    e[3] = 0.5f*(R_err[2][1]-R_err[1][2]);
    e[4] = 0.5f*(R_err[0][2]-R_err[2][0]);
    e[5] = 0.5f*(R_err[1][0]-R_err[0][1]);
}
void pose_set_rpy_deg(pose_t *p)
{
    float r = DEG2RAD(p->roll);
    float pch = DEG2RAD(p->pitch);
    float y = DEG2RAD(p->yaw);
    rot_rpy_to_R(r, pch, y, p->R);
}

bool ikine_deg(const pose_t *target,
               const float q_cur_deg[JOINT_NUM],
               float q_out_deg[JOINT_NUM])
{
    float q[6];
    memcpy(q, q_cur_deg, sizeof(q));

    pose_t cur;
    float e[6];
    float alpha = 0.2f;      // 步长
    float eps_pos = 0.5f;    // mm
    float eps_ori = DEG2RAD(0.5f); // rad

    for(int iter=0; iter<80; iter++)
    {
        fkine_deg(q, &cur);
        calc_error(&cur, target, e);

        if (fabsf(e[0])<eps_pos && fabsf(e[1])<eps_pos && fabsf(e[2])<eps_pos &&
            fabsf(e[3])<eps_ori && fabsf(e[4])<eps_ori && fabsf(e[5])<eps_ori) {
            memcpy(q_out_deg, q, sizeof(q));
            return true;
        }

        // 数值Jacobian (6x6)
        float J[6][6];
        float dq = 0.1f; // deg
        for(int i=0;i<6;i++){
            float q_tmp[6];
            memcpy(q_tmp, q, sizeof(q));
            q_tmp[i] += dq;

            pose_t p2;
            fkine_deg(q_tmp, &p2);

            float e2[6];
            calc_error(&cur, &p2, e2); // 近似列向量

            for(int r=0;r<6;r++){
                J[r][i] = e2[r] / DEG2RAD(dq);
            }
        }

        // Jacobian转置更新：dq = alpha * J^T * e
        float dq_vec[6]={0};
        for(int i=0;i<6;i++){
            for(int r=0;r<6;r++){
                dq_vec[i] += J[r][i]*e[r];
            }
        }

        for(int i=0;i<6;i++){
            q[i] += RAD2DEG(alpha * dq_vec[i]);
        }
        clamp_joint(q);
    }

    // 失败：保持上次解
    memcpy(q_out_deg, q_cur_deg, sizeof(float)*6);
    return false;
}

bool ikine_deg_ex(const pose_t *target,
const float q_cur_deg[JOINT_NUM],
float q_out_deg[JOINT_NUM],
int *out_iter,
float *out_pos_err,
float *out_ori_err)
{
float q[6];
memcpy(q, q_cur_deg, sizeof(q));
    pose_t cur;
float e[6];
float alpha = 0.05f;      // ✅ 更小步长
float eps_pos = 5.0f;
float eps_ori = DEG2RAD(5.0f);
    const float w_ori = 50.0f; // ✅ 姿态误差权重（单位mm/rad）
const float dq_limit = 2.0f; // ✅ 每步最大 2° 防止发散
    for(int iter=0; iter<200; iter++)  // ✅ 迭代次数加到200
{
fkine_deg(q, &cur);
calc_error(&cur, target, e);
        float pos_err = sqrtf(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
float ori_err = fmaxf(fabsf(e[3]), fmaxf(fabsf(e[4]), fabsf(e[5])));
        if (pos_err < eps_pos && ori_err < eps_ori) {
memcpy(q_out_deg, q, sizeof(q));
if (out_iter) *out_iter = iter + 1;
if (out_pos_err) *out_pos_err = pos_err;
if (out_ori_err) *out_ori_err = RAD2DEG(ori_err);
return true;
}
        // 数值Jacobian (6x6)
float J[6][6];
float dq = 0.1f;
for(int i=0;i<6;i++){
float q_tmp[6];
memcpy(q_tmp, q, sizeof(q));
q_tmp[i] += dq;
            pose_t p2;
fkine_deg(q_tmp, &p2);
            float e2[6];
calc_error(&cur, &p2, e2);
            for(int r=0;r<6;r++){
J[r][i] = e2[r] / DEG2RAD(dq);
}
}
        // ✅ 误差加权
e[3] *= w_ori; e[4] *= w_ori; e[5] *= w_ori;
        float dq_vec[6]={0};
for(int i=0;i<6;i++){
for(int r=0;r<6;r++){
dq_vec[i] += J[r][i]*e[r];
}
}
        for(int i=0;i<6;i++){
float dq_step = RAD2DEG(alpha * dq_vec[i]);
if (dq_step > dq_limit) dq_step = dq_limit;
if (dq_step < -dq_limit) dq_step = -dq_limit;
q[i] += dq_step;
}
clamp_joint(q);
        if (out_iter) *out_iter = iter + 1;
if (out_pos_err) *out_pos_err = pos_err;
if (out_ori_err) *out_ori_err = RAD2DEG(ori_err);
}
    memcpy(q_out_deg, q_cur_deg, sizeof(float)*6);
return false;
}
