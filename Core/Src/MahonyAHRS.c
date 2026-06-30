#include "MahonyAHRS.h"
#include <math.h>

/* ================================================================
 * 全局四元数 — 初始水平朝北 (roll=pitch=yaw=0)
 * ================================================================ */
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

/* PI 参数 */
static float kp, ki;
static float integral_fbx, integral_fby;

/* ================================================================
 * 初始化 — 设置 PI 增益并归零积分项
 * ================================================================ */
void MahonyAHRS_init(float kp_val, float ki_val)
{
    kp = kp_val;
    ki = ki_val;
    integral_fbx = integral_fby = 0.0f;
}

/* ================================================================
 * Mahony AHRS update — 对标 Betaflight imuMahonyAHRSupdate
 *
 * 步骤：
 *   1. 归一化加速度计测量值
 *   2. 从当前四元数估计重力方向
 *   3. 误差 = cross(测量, 估计) → PI 修正 gyro
 *   4. 用修正后的 gyro 积分四元数
 *   5. 归一化四元数
 * ================================================================ */
void MahonyAHRS_update(float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float dt)
{
    float recip_norm;
    float vx, vy, vz;
    float ex, ey;

    /* 加速度计不为零时才做修正 */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        /* 归一化加速度计 */
        recip_norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        /* 从四元数估计重力方向 */
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        /* 误差 = cross(accel, 重力估计)
         * Z 轴不修正 — 加速度计无法提供 yaw 信息 */
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;

        /* 积分项（仅当 ki > 0 时生效，Z轴不积分） */
        if (ki > 0.0f) {
            integral_fbx += ki * ex * dt;
            integral_fby += ki * ey * dt;
        } else {
            integral_fbx = integral_fby = 0.0f;
        }

        /* PI 修正 gyro（Z轴纯积分） */
        gx += kp * ex + integral_fbx;
        gy += kp * ey + integral_fby;
    }

    /* 四元数积分（一阶） */
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = q0, qb = q1, qc = q2, qd = q3;
    q0 += -qb * gx - qc * gy - qd * gz;
    q1 +=  qa * gx + qd * gy - qc * gz;
    q2 +=  qa * gy - qd * gx + qb * gz;
    q3 +=  qa * gz + qc * gx - qb * gy;

    /* 归一化 */
    recip_norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recip_norm;
    q1 *= recip_norm;
    q2 *= recip_norm;
    q3 *= recip_norm;
}
