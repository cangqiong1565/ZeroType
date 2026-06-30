#ifndef ZEROTYPE_PID_H
#define ZEROTYPE_PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;//积分累加器，存误差总和
    float integral_max;//积分上限
    float prev_measurement;//上一次读数，给D项微分用
    float output_max;//输出上限
    float d_filter_tau;//D项低通滤波的时间常数 τ（越小滤波越强）
    float filtered_D;//D项经过低通滤波后的值
}PID_t;//速度控制器

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float integral_max, float output_max, float d_filter_tau);
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt);
#endif //ZEROTYPE_PID_H
