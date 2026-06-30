#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

/* Mahony AHRS — PI 修正互补滤波器
 *
 * 对照 Betaflight imu.c 的 imuMahonyAHRSupdate, 核心思路：
 *   accel → cross(实测 g, 估计 g) → PI(error) → 修正 gyro → 四元数积分
 *
 * 参数：
 *   kp = 0.5f  比例增益（加速度计修正力度）
 *   ki = 0.0f  积分增益（初始为 0，收敛后可微调消除长期漂移）
 *
 * 输入：gyro(rad/s), accel(g), dt(秒)
 * 输出：全局四元数 q0, q1, q2, q3
 */

extern volatile float q0, q1, q2, q3;

void MahonyAHRS_init(float kp, float ki);
void MahonyAHRS_update(float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float dt);

#endif /* MAHONY_AHRS_H */
