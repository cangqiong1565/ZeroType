//
// Created by l on 2026/7/25.
//

#ifndef ZEROTYPE_CONTROL_H
#define ZEROTYPE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Control v1: test-stand level controller.
 *
 * This stage does not do altitude hold.
 * Throttle is controlled manually by the transmitter.
 * The controller only tries to keep roll/pitch level and damp yaw rate.
 */

#define CONTROL_MOTOR_COUNT              4U

/* If roll/pitch exceeds this angle, output is disabled immediately. */
#define CONTROL_MAX_SAFE_ANGLE_DEG       25.0f

/* Level mode target attitude: keep the frame horizontal. */
#define CONTROL_LEVEL_ROLL_DEG           0.0f
#define CONTROL_LEVEL_PITCH_DEG          0.0f

/* Angle outer loop output limit. */
#define CONTROL_MAX_LEVEL_RATE_DPS       80.0f

/* Yaw is not angle-controlled in v1; it only damps gyro_z toward zero. */
#define CONTROL_TARGET_YAW_RATE_DPS      0.0f

/*
 * RC input range after CRSF_MapRawToUs().
 * Control never sees raw CRSF values.
 */
#define CONTROL_MIN_RC_US                1000U
#define CONTROL_MID_RC_US                1500U
#define CONTROL_MAX_RC_US                2000U

/*
 * First enable is allowed only when throttle is low.
 * After enabled, throttle can move normally until disarm/failsafe/unsafe.
 */
#define CONTROL_ARM_THROTTLE_MAX_US      1050U

/*
 * Conservative DShot output range for test-stand work.
 * Raise CONTROL_MOTOR_MAX_DSHOT only after no-prop direction tests are correct.
 */
#define CONTROL_MOTOR_MIN_DSHOT          150U
#define CONTROL_MOTOR_MAX_DSHOT          1500U

typedef struct
{
    uint16_t throttle_us;     // Manual throttle, 1000..2000.

    bool arm_switch;          // true means the transmitter requests control output.
    bool level_switch;        // true means level controller is allowed to run.
    bool failsafe;            // true means RC link or receiver data is unsafe.
} ControlRcInput_t;

typedef struct
{
    float roll_deg;           // Current roll angle, degrees.
    float pitch_deg;          // Current pitch angle, degrees.

    float gyro_x_dps;         // Roll-axis angular rate, deg/s.
    float gyro_y_dps;         // Pitch-axis angular rate, deg/s.
    float gyro_z_dps;         // Yaw-axis angular rate, deg/s.

    bool imu_valid;           // false disables output and resets PID state.
} ControlSensorInput_t;

typedef struct
{
    float target_roll_rate_dps;   // Angle loop output for roll.
    float target_pitch_rate_dps;  // Angle loop output for pitch.
    float target_yaw_rate_dps;    // v1 target is always zero.

    float roll_pid;               // Roll rate PID output.
    float pitch_pid;              // Pitch rate PID output.
    float yaw_pid;                // Yaw rate damping output.

    uint16_t throttle_us;         // Clamped manual throttle input.
    uint16_t motor[CONTROL_MOTOR_COUNT]; // Mixed DShot targets.

    bool enabled;                 // true means safety gates passed this update.
} ControlOutput_t;

void Control_Init(void);

void Control_Reset(void);

void Control_Update(const ControlRcInput_t *rc,
                    const ControlSensorInput_t *sensor,
                    float dt,
                    ControlOutput_t *out);

#endif // ZEROTYPE_CONTROL_H
