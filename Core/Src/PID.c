#include "PID.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float integral_max, float output_max, float d_filter_tau) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->integral_max = integral_max;
    pid->prev_measurement = 0.0f;
    pid->output_max = output_max;
    pid->d_filter_tau = d_filter_tau;
    pid->filtered_D = 0.0f;
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;

    float P_out = pid->Kp * error;

    pid->integral += error * dt;

    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    }

    if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }

    float I_out = pid->Ki * pid->integral;

    float D_raw = -pid->Kd * (measurement - pid->prev_measurement) / dt;

    pid->filtered_D += (D_raw - pid->filtered_D) * dt / pid->d_filter_tau;
    

    float output = P_out + I_out + pid->filtered_D;

    if (output > pid->output_max) {
        output = pid->output_max;
    }
    if (output < -pid->output_max) {
        output = -pid->output_max;
    }
    pid->prev_measurement = measurement;
    return output;
}