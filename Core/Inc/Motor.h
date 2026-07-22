#ifndef ZEROTYPE_MOTOR_H
#define ZEROTYPE_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Motor 是飞控的电机业务层。
 *
 * Dshot.c 只负责把 0..2047 编码成 DShot 波形；
 * Motor.c 负责软件解锁、失控保护、遥控油门映射和四电机输出。
 */
#define MOTOR_COUNT        4U
#define MOTOR_IDLE_DSHOT   100U
#define RC_THROTTLE_MIN    1000U
#define RC_THROTTLE_MAX    2000U

typedef enum
{
    MOTOR_STATE_DISARMED = 0,    // 软件锁定：四路强制输出 DShot 0
    MOTOR_STATE_ARMED,           // 软件解锁：最低油门也输出怠速
    MOTOR_STATE_FAILSAFE         // 失控保护：四路强制输出 DShot 0
} MotorState_t;

typedef struct
{
    MotorState_t state;          // 当前电机状态
    uint16_t rc_throttle_us;     // 当前遥控油门，统一用 1000..2000 表示
    uint16_t output[MOTOR_COUNT];// 当前实际发给四个电调的 DShot 值
} MotorStatus_t;

typedef struct
{
    uint16_t throttle;          //遥控器油门通道
    bool arm_switch;            //遥控器解锁开关
    bool failsafe;              //遥控器失控标志
}MotorRcInput_t;

void Motor_Arm(void);
void Motor_Disarm(void);
void Motor_SetFailsafe(bool enabled);
void Motor_SetRcThrottle(uint16_t value);
void Motor_Update(void);
void Motor_GetStatus(MotorStatus_t *status);
void Motor_SetRcInput(const MotorRcInput_t *input);
#endif //ZEROTYPE_MOTOR_H
