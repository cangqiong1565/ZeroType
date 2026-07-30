#include "Control.h"

#include <stddef.h>

#include "PID.h"
#include <stddef.h>

static PID_t roll_angle_pid;
static PID_t pitch_angle_pid;

static PID_t roll_rate_pid;
static PID_t pitch_rate_pid;
static PID_t yaw_rate_pid;

static bool control_enabled_latch = false;

//浮点限制函数
static float Control_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

//U16限制函数
static uint16_t Control_ClampU16(uint16_t value,
                                 uint16_t min_value,
                                 uint16_t max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

//姿态角安全判定函数
static bool Control_AngleUnsafe(float angle_deg)
{
    return (angle_deg > CONTROL_MAX_SAFE_ANGLE_DEG) ||
           (angle_deg < -CONTROL_MAX_SAFE_ANGLE_DEG);
}

//清空输出函数，属于安全模块
static void Control_ClearOutput(ControlOutput_t *out)
{
    if (out == NULL)
    {
        return;
    }

    //输出角速度清零
    out->target_roll_rate_dps = 0.0f;
    out->target_pitch_rate_dps = 0.0f;
    out->target_yaw_rate_dps = 0.0f;

    //输出角度清零
    out->roll_pid = 0.0f;
    out->pitch_pid = 0.0f;
    out->yaw_pid = 0.0f;

    //高度输出清零
    out->throttle_us = CONTROL_MIN_RC_US;

    //电机输出清零
    out->motor[0] = 0U;
    out->motor[1] = 0U;
    out->motor[2] = 0U;
    out->motor[3] = 0U;

    //失能
    out->enabled = false;
}

//运行安全检查，所有条件都满足才允许解锁
static bool Control_RuntimeSafe(const ControlRcInput_t *rc,
                                const ControlSensorInput_t *sensor,
                                float dt)
{
    //数据是否为空指针
    if ((rc == NULL) || (sensor == NULL))
    {
        return false;
    }

    //dt是否合理
    if (dt <= 0.0f)
    {
        return false;
    }

    //是否安全
    if (rc->failsafe)
    {
        return false;
    }

    //是否解锁
    if (!rc->arm_switch)
    {
        return false;
    }

    //是否允许控制
    if (!rc->level_switch)
    {
        return false;
    }

    //imu是否校准
    if (!sensor->imu_valid)
    {
        return false;
    }

    //现在的角度是否在安全范围内
    if (Control_AngleUnsafe(sensor->roll_deg) ||
        Control_AngleUnsafe(sensor->pitch_deg))
    {
        return false;
    }

    return true;
}

//油门转Dshot函数
static float Control_MapThrottleToDshot(uint16_t throttle_us)
{
    //油门
    uint16_t throttle;
    //输入（输入的是啥不知道）
    uint32_t input_range;
    //输出（输出的是啥不知道）
    uint32_t output_range;
    //输入（输入的是啥不知道）
    uint32_t input_offset;

    //先把高度合理化
    throttle = Control_ClampU16(throttle_us,
                                CONTROL_MIN_RC_US,
                                CONTROL_MAX_RC_US);

    //往下不知道
    input_range = CONTROL_MAX_RC_US - CONTROL_MIN_RC_US;
    output_range = CONTROL_MOTOR_MAX_DSHOT - CONTROL_MOTOR_MIN_DSHOT;
    input_offset = throttle - CONTROL_MIN_RC_US;

    return (float)(CONTROL_MOTOR_MIN_DSHOT +
                   (input_offset * output_range) / input_range);
}

//混控器
static void Control_MixToMotors(float base_throttle,
                                float roll,
                                float pitch,
                                float yaw,
                                ControlOutput_t *out)
{
    float m0;
    float m1;
    float m2;
    float m3;

    if (out == NULL)
    {
        return;
    }

    m0 = base_throttle - roll + pitch + yaw;
    m1 = base_throttle - roll - pitch - yaw;
    m2 = base_throttle + roll + pitch - yaw;
    m3 = base_throttle + roll - pitch + yaw;

    //输出限幅
    m0 = Control_ClampFloat(m0, CONTROL_MOTOR_MIN_DSHOT, CONTROL_MOTOR_MAX_DSHOT);
    m1 = Control_ClampFloat(m1, CONTROL_MOTOR_MIN_DSHOT, CONTROL_MOTOR_MAX_DSHOT);
    m2 = Control_ClampFloat(m2, CONTROL_MOTOR_MIN_DSHOT, CONTROL_MOTOR_MAX_DSHOT);
    m3 = Control_ClampFloat(m3, CONTROL_MOTOR_MIN_DSHOT, CONTROL_MOTOR_MAX_DSHOT);

    //赋值
    out->motor[0] = (uint16_t)m0;
    out->motor[1] = (uint16_t)m1;
    out->motor[2] = (uint16_t)m2;
    out->motor[3] = (uint16_t)m3;
}

//控制器初始化
void Control_Init(void)
{
    //初始化所有环
    PID_Init(&roll_angle_pid,
             8.5f,
             0.8f,
             0.6f,
             20.0f,
             400,
             0.0f);

    PID_Init(&pitch_angle_pid,
             8.0f,
             0.8f,
             0.6f,
             20.0f,
             CONTROL_MAX_LEVEL_RATE_DPS,
             0.0f);

    PID_Init(&roll_rate_pid,
             1.5f,
             0.0f,
             0.0f,
             20.0f,
             400.0f,
             0.02f);

    PID_Init(&pitch_rate_pid,
             2.0f,
             0.0f,
             0.0f,
             20.0f,
             40.0f,
             0.02f);
    //
    // PID_Init(&yaw_rate_pid,
    //          0.02f,
    //          0.0f,
    //          0.0f,
    //          20.0f,
    //          20.0f,
    //          0.02f);

    //复位所有值
    Control_Reset();
}

void Control_Reset(void)
{
    PID_Reset(&roll_angle_pid);
    PID_Reset(&pitch_angle_pid);

    PID_Reset(&roll_rate_pid);
    PID_Reset(&pitch_rate_pid);
    PID_Reset(&yaw_rate_pid);

    control_enabled_latch = false;
}

//更新函数，每周期调用一次，完成安全检查，油门映射，内外环和混控输出
void Control_Update(const ControlRcInput_t *rc,
                    const ControlSensorInput_t *sensor,
                    float dt,
                    ControlOutput_t *out)
{
    //基准油门，后续在这个值上修正
    float base_throttle;

    if (out == NULL)
    {
        return;
    }
    //先把输出清到安全状态，避免赋值紊乱，不会残留上一轮的值
    Control_ClearOutput(out);

    //运行安全检查，Control_RuntimeSafe检查运行状态
    if (!Control_RuntimeSafe(rc, sensor, dt))
    {
        Control_Reset();
        return;
    }
    //遥控器油门限幅
    out->throttle_us = Control_ClampU16(rc->throttle_us,
                                        CONTROL_MIN_RC_US,
                                        CONTROL_MAX_RC_US);

    /*control_enabled_latch是软件使能锁存，它防止的是
     *插电时解锁开关开着，油门也不低，容易发生危险
     *这里规定了第一次允许控制时，油门必须小于1050
     */
    if (!control_enabled_latch)
    {
        //油门过高，复位直接结束
        if (out->throttle_us > CONTROL_ARM_THROTTLE_MAX_US)
        {
            Control_Reset();
            return;
        }

        //油门地位检查通过时，软件锁打开
        control_enabled_latch = true;

        //第一次允许控制时清PID，从干净状态开始控制
        PID_Reset(&roll_angle_pid);
        PID_Reset(&pitch_angle_pid);
        PID_Reset(&roll_rate_pid);
        PID_Reset(&pitch_rate_pid);
        PID_Reset(&yaw_rate_pid);
    }

    //把遥控器油门转换成Dshot油门
    base_throttle = Control_MapThrottleToDshot(out->throttle_us);

    //roll外环
     out->target_roll_rate_dps = PID_Update(&roll_angle_pid,
                                            CONTROL_LEVEL_ROLL_DEG,
                                            sensor->roll_deg,
                                            dt);

    //pitch外环
    out->target_pitch_rate_dps = PID_Update(&pitch_angle_pid,
                                            CONTROL_LEVEL_PITCH_DEG,
                                            sensor->pitch_deg,
                                            dt);

    /* 第一版不做角度保持，只做阻尼。
     * 目标 yaw 角速度固定为：
     * 0 deg/s
     */
    // out->target_yaw_rate_dps = CONTROL_TARGET_YAW_RATE_DPS;

    //内环
    out->roll_pid = PID_Update(&roll_rate_pid,
                               out->target_roll_rate_dps,
                               sensor->gyro_x_dps,
                               dt);

    out->pitch_pid = PID_Update(&pitch_rate_pid,
                                out->target_pitch_rate_dps,
                                sensor->gyro_y_dps,
                                dt);
    // //yaw阻尼环
    // out->yaw_pid = PID_Update(&yaw_rate_pid,
    //                           out->target_yaw_rate_dps,
    //                           sensor->gyro_z_dps,
    //                           dt);

    //混控
    Control_MixToMotors(base_throttle,
                        out->roll_pid,
                        out->pitch_pid,
                        out->yaw_pid,
                        out);

    //到这说明安全检查通过，PID，混控全部完成
    out->enabled = true;
}
