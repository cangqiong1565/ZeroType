//
// Created by l on 2026/7/25.
//

#ifndef ZEROTYPE_CONTROL_H
#define ZEROTYPE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 控制器第一版：调试架用水平控制器。
 *
 * 当前阶段不做定高。
 * 油门仍然由遥控器直接控制。
 * 控制器负责让 roll/pitch 保持水平，并让 yaw 保持解锁时的相对航向。
 */

#define CONTROL_MOTOR_COUNT              4U

/* roll 或 pitch 超过这个角度时，控制输出立刻关闭。 */
#define CONTROL_MAX_SAFE_ANGLE_DEG       25.0f

/* 水平模式目标姿态：让机体保持水平。 */
#define CONTROL_LEVEL_ROLL_DEG           0.0f
#define CONTROL_LEVEL_PITCH_DEG          0.0f
#define CONTROL_LEVEL_YAW_DEG          0.0f

/* roll/pitch 角度外环输出限幅，单位 deg/s。 */
#define CONTROL_MAX_LEVEL_RATE_DPS       80.0f

/* yaw 航向保持外环输出限幅，单位 deg/s。 */
#define CONTROL_MAX_YAW_HOLD_RATE_DPS    80.0f

/*
 * 遥控器输入范围。
 * 这里使用的是 CRSF_MapRawToUs() 转换后的 1000~2000 us 值，
 * Control 层不直接处理 CRSF 原始通道值。
 */
#define CONTROL_MIN_RC_US                1000U
#define CONTROL_MID_RC_US                1500U
#define CONTROL_MAX_RC_US                2000U

/*
 * 第一次允许控制时，油门必须处于低位。
 * 控制使能后，油门可以正常变化，直到锁定、失控保护或姿态不安全。
 */
#define CONTROL_ARM_THROTTLE_MAX_US      1050U

/*
 * 调试架阶段使用的保守 DShot 输出范围。
 * 只有在无桨方向测试完全正确后，才逐步提高 CONTROL_MOTOR_MAX_DSHOT。
 */
#define CONTROL_MOTOR_MIN_DSHOT          150U
#define CONTROL_MOTOR_MAX_DSHOT          1500U

#define CONTROL_STICK_DEADBAND_US        20U
#define CONTROL_MAX_STICK_ANGLE_DEG      5.0f
#define CONTROL_MAX_YAW_RATE_DPS         120.0f

typedef struct
{
    uint16_t roll_us;        // CH1 Ail
    uint16_t pitch_us;       // CH2 Ele
    uint16_t throttle_us;    // CH3 Thr
    uint16_t yaw_us;         // CH4 Rud

    bool arm_switch;          // true 表示遥控器请求解锁并允许输出。
    bool level_switch;        // true 表示允许水平控制器运行。
    bool failsafe;            // true 表示遥控链路或接收机数据不安全。
} ControlRcInput_t;

typedef struct
{
    float roll_deg;           // 当前 roll 角，单位 degree。
    float pitch_deg;          // 当前 pitch 角，单位 degree。
    float yaw_deg;            // 当前相对 yaw 角，单位 degree。

    float gyro_x_dps;         // roll 轴角速度，单位 deg/s。
    float gyro_y_dps;         // pitch 轴角速度，单位 deg/s。
    float gyro_z_dps;         // yaw 轴角速度，单位 deg/s。

    bool imu_valid;           // false 表示 IMU 数据不可用，控制器会关闭输出并复位 PID。
} ControlSensorInput_t;

typedef struct
{
    float target_roll_rate_dps;   // roll 角度外环输出的目标 roll 角速度。
    float target_pitch_rate_dps;  // pitch 角度外环输出的目标 pitch 角速度。
    float target_yaw_rate_dps;    // yaw 航向外环输出的目标 yaw 角速度。
    float target_yaw_deg;         // 控制使能瞬间锁定的 yaw 目标角。
    float yaw_error_deg;          // 已处理 ±180 跳变后的 yaw 误差，target - current。

    float roll_pid;               // roll 角速度内环 PID 输出。
    float pitch_pid;              // pitch 角速度内环 PID 输出。
    float yaw_pid;                // yaw 角速度内环 PID 输出。

    uint16_t throttle_us;         // 限幅后的手动油门输入。
    uint16_t motor[CONTROL_MOTOR_COUNT]; // 混控后的四路 DShot 目标值。

    bool enabled;                 // true 表示本轮安全检查通过，控制输出有效。
} ControlOutput_t;

void Control_Init(void);

void Control_Reset(void);

void Control_Update(const ControlRcInput_t *rc,
                    const ControlSensorInput_t *sensor,
                    float dt,
                    ControlOutput_t *out);

#endif // ZEROTYPE_CONTROL_H
