#include "robot_ik_dls.h"
#include <math.h>
#include <string.h>
#include <float.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif
#define DEG2RAD(x) ((x) * (PI / 180.0f))
#define RAD2DEG(x) ((x) * (180.0f / PI))

/* ===== 可调参数（V1建议值） =====
 * 你要求：位置更重要、姿态次要；姿态允许 5°以内
 */
#define IK_EPS_POS_MM        3.0f
#define IK_EPS_ORI_DEG       5.0f

#define IK_MAX_ITER          200
#define IK_NUM_DQ_DEG        0.05f       // 数值Jacobian扰动
#define IK_DQ_LIMIT_DEG      2.0f        // 单步每关节最大更新，防发散

#define IK_W_ORI_MM_PER_RAD  10.0f       // ✅ 姿态软约束：权重小（位置优先）

/* LM阻尼（自适应） */
#define LAMBDA_INIT          10.0f
#define LAMBDA_MIN           0.1f
#define LAMBDA_MAX           200.0f
#define LAMBDA_UP            2.0f
#define LAMBDA_DOWN          0.7f

/* ===== 关节限位（与 robot_kinematics.c 保持一致） ===== */
static const float q_min_[6] = {0, -90, 0, 0, -90, 0};
static const float q_max_[6] = {360, 90, 180, 360, 0, 360};

/* 软夹：越界则钳回边界，并返回 violated=1 */
static int soft_clamp(float q[6])
{
    int violated = 0;
    for (int i = 0; i < 6; i++) {
        if (q[i] < q_min_[i]) { q[i] = q_min_[i]; violated = 1; }
        if (q[i] > q_max_[i]) { q[i] = q_max_[i]; violated = 1; }
    }
    return violated;
}

static float pos_err_norm(const float e[6])
{
    return sqrtf(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
}

/* 姿态误差：取旋转误差向量的 max 分量（和你原工程输出一致风格） */
static float ori_err_absmax_rad(const float e[6])
{
    float a = fabsf(e[3]);
    float b = fabsf(e[4]);
    float c = fabsf(e[5]);
    return fmaxf(a, fmaxf(b, c));
}

/* ===== 误差：tar - cur（位置 + 小角度旋转向量） ===== */
static void pose_error6(const pose_t *cur, const pose_t *tar, float e[6])
{
    e[0] = tar->x - cur->x;
    e[1] = tar->y - cur->y;
    e[2] = tar->z - cur->z;

    /* Rt = R_cur^T */
    float Rt[3][3];
    Rt[0][0]=cur->R[0][0]; Rt[0][1]=cur->R[1][0]; Rt[0][2]=cur->R[2][0];
    Rt[1][0]=cur->R[0][1]; Rt[1][1]=cur->R[1][1]; Rt[1][2]=cur->R[2][1];
    Rt[2][0]=cur->R[0][2]; Rt[2][1]=cur->R[1][2]; Rt[2][2]=cur->R[2][2];

    /* Rerr = R_tar * R_cur^T */
    float Rerr[3][3];
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            float s=0;
            for(int k=0;k<3;k++) s += tar->R[i][k]*Rt[k][j];
            Rerr[i][j]=s;
        }
    }

    /* 小角度近似旋转向量 */
    e[3] = 0.5f*(Rerr[2][1] - Rerr[1][2]);
    e[4] = 0.5f*(Rerr[0][2] - Rerr[2][0]);
    e[5] = 0.5f*(Rerr[1][0] - Rerr[0][1]);
}

/* ===== 6x6 高斯消元：解 Ax=b ===== */
static bool solve6(float A[6][6], float b[6], float x[6])
{
    float M[6][7];
    for(int i=0;i<6;i++){
        for(int j=0;j<6;j++) M[i][j]=A[i][j];
        M[i][6]=b[i];
    }

    for(int col=0; col<6; col++){
        int piv=col;
        float maxv=fabsf(M[col][col]);
        for(int r=col+1;r<6;r++){
            float v=fabsf(M[r][col]);
            if(v>maxv){ maxv=v; piv=r; }
        }
        if(maxv < 1e-9f) return false;

        if(piv != col){
            for(int k=col;k<7;k++){
                float tmp=M[col][k];
                M[col][k]=M[piv][k];
                M[piv][k]=tmp;
            }
        }

        float div=M[col][col];
        for(int k=col;k<7;k++) M[col][k]/=div;

        for(int r=0;r<6;r++){
            if(r==col) continue;
            float f=M[r][col];
            if(fabsf(f)<1e-12f) continue;
            for(int k=col;k<7;k++){
                M[r][k] -= f*M[col][k];
            }
        }
    }

    for(int i=0;i<6;i++) x[i]=M[i][6];
    return true;
}

/* 数值Jacobian：J[:,i] = (pose(q+Δ) - pose(q)) / Δqi(rad) */
static void jacobian_num(const float q_deg[6], const pose_t *cur, float J[6][6])
{
    for(int i=0;i<6;i++){
        float q2[6];
        memcpy(q2, q_deg, sizeof(q2));
        q2[i] += IK_NUM_DQ_DEG;

        pose_t p2;
        fkine_deg(q2, &p2);

        float ed[6];
        pose_error6(cur, &p2, ed);

        float dqi = DEG2RAD(IK_NUM_DQ_DEG);
        for(int r=0;r<6;r++){
            J[r][i] = ed[r] / dqi;
        }
    }
}

/* DLS：解 (J^T J + λ^2 I) dq = J^T e */
static bool dls_solve(const float J[6][6], const float e[6], float lambda, float dq_rad[6])
{
    float A[6][6] = {0};
    float b[6] = {0};

    /* b = J^T e */
    for(int i=0;i<6;i++){
        float s=0;
        for(int r=0;r<6;r++) s += J[r][i]*e[r];
        b[i]=s;
    }

    /* A = J^T J */
    for(int i=0;i<6;i++){
        for(int j=0;j<6;j++){
            float s=0;
            for(int r=0;r<6;r++) s += J[r][i]*J[r][j];
            A[i][j]=s;
        }
    }

    float l2=lambda*lambda;
    for(int i=0;i<6;i++) A[i][i] += l2;

    float x[6];
    if(!solve6(A, b, x)) return false;

    for(int i=0;i<6;i++) dq_rad[i] = x[i];
    return true;
}

bool ikine_dls_deg_ex(const pose_t *target,
                      const float q_seed_deg[JOINT_NUM],
                      float q_out_deg[JOINT_NUM],
                      int *out_iter,
                      float *out_pos_err,
                      float *out_ori_err)
{
    if(!target || !q_seed_deg || !q_out_deg) return false;

    float q[6];
    memcpy(q, q_seed_deg, sizeof(q));

    float best_q[6];
    memcpy(best_q, q, sizeof(best_q));
    float best_cost = FLT_MAX;

    float lambda = LAMBDA_INIT;

    for(int iter=0; iter<IK_MAX_ITER; iter++){
        pose_t cur;
        fkine_deg(q, &cur);

        float e[6];
        pose_error6(&cur, target, e);

        float pos_err = pos_err_norm(e);
        float ori_err_rad = ori_err_absmax_rad(e);

        if(out_iter) *out_iter = iter + 1;
        if(out_pos_err) *out_pos_err = pos_err;
        if(out_ori_err) *out_ori_err = RAD2DEG(ori_err_rad);

        if(pos_err < IK_EPS_POS_MM && RAD2DEG(ori_err_rad) < IK_EPS_ORI_DEG){
            memcpy(q_out_deg, q, sizeof(q));
            return true;
        }

        /* ✅ 位置优先：姿态误差弱化为等效mm */
        float ew[6];
        ew[0]=e[0]; ew[1]=e[1]; ew[2]=e[2];
        ew[3]=e[3]*IK_W_ORI_MM_PER_RAD;
        ew[4]=e[4]*IK_W_ORI_MM_PER_RAD;
        ew[5]=e[5]*IK_W_ORI_MM_PER_RAD;

        float J[6][6];
        jacobian_num(q, &cur, J);

        float dq_rad[6];
        if(!dls_solve(J, ew, lambda, dq_rad)){
            lambda *= LAMBDA_UP;
            if(lambda > LAMBDA_MAX) lambda = LAMBDA_MAX;
            continue;
        }

        float q_try[6];
        memcpy(q_try, q, sizeof(q_try));

        for(int i=0;i<6;i++){
            float dq_deg = RAD2DEG(dq_rad[i]);

            if(dq_deg >  IK_DQ_LIMIT_DEG) dq_deg =  IK_DQ_LIMIT_DEG;
            if(dq_deg < -IK_DQ_LIMIT_DEG) dq_deg = -IK_DQ_LIMIT_DEG;

            q_try[i] += dq_deg;
        }

        int violated = soft_clamp(q_try);
        if(violated){
            /* 越界：提高阻尼（更稳） */
            lambda *= LAMBDA_UP;
            if(lambda > LAMBDA_MAX) lambda = LAMBDA_MAX;
        }

        pose_t cur2;
        fkine_deg(q_try, &cur2);

        float e2[6];
        pose_error6(&cur2, target, e2);

        float pos2 = pos_err_norm(e2);
        float ori2 = ori_err_absmax_rad(e2);

        float cost1 = pos_err + IK_W_ORI_MM_PER_RAD*ori_err_rad;
        float cost2 = pos2    + IK_W_ORI_MM_PER_RAD*ori2;

        if(cost2 < best_cost){
            best_cost = cost2;
            memcpy(best_q, q_try, sizeof(best_q));
        }

        if(cost2 < cost1){
            /* 变好：接受并减小阻尼 */
            memcpy(q, q_try, sizeof(q));
            lambda *= LAMBDA_DOWN;
            if(lambda < LAMBDA_MIN) lambda = LAMBDA_MIN;
        }else{
            /* 变差：拒绝并增大阻尼 */
            lambda *= LAMBDA_UP;
            if(lambda > LAMBDA_MAX) lambda = LAMBDA_MAX;
        }
    }

    /* 未达阈值也输出最优解：轨迹更连续 */
    memcpy(q_out_deg, best_q, sizeof(best_q));
    return false;
}
