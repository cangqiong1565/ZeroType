#include "PID.h"
#include <stddef.h>

static float PID_Clamp(float value,float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float integral_max, float output_max, float d_filter_tau)
{
    if (pid == NULL)
    {
        return;
    }
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->integral = 0.0f;
    pid->integral_max = integral_max;

    pid->prev_measurement = 0.0f;
    pid->output_max = output_max;

    pid->d_filter_tau = d_filter_tau;
    pid->filtered_D = 0.0f;

    pid->initialized = 0U;
}

void PID_Reset(PID_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->filtered_D = 0.0f;
    pid->initialized = 0U;
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    if (dt <= 0.0f)
    {
        return 0.0f;
    }

    float error = setpoint - measurement;

    float P_out = pid->Kp * error;

    pid->integral += error * dt;

    pid->integral = PID_Clamp(pid->integral ,pid->integral_max);

    float I_out = pid->Ki * pid->integral;

    if (pid->initialized == 0U)
    {
        pid->prev_measurement = measurement;
        pid->filtered_D = 0.0f;
        pid->initialized = 1U;
    }

    float D_raw = -pid->Kd * (measurement - pid->prev_measurement) / dt;

    if (pid->d_filter_tau <= 0.0f)
    {
        pid->filtered_D = D_raw;
    }
    else
    {
        float D_alpha = dt / pid->d_filter_tau;

        if (D_alpha > 1.0f)
        {
            D_alpha = 1.0f;
        }

        pid->filtered_D += (D_raw - pid->filtered_D) * D_alpha;
    }
    

    float output = P_out + I_out + pid->filtered_D;

    output = PID_Clamp(output,pid->output_max);
    pid->prev_measurement = measurement;
    return output;
}