#include "Motor.h"

#include "Dshot.h"
#include "Task.h"

//初始化为上锁状态
static volatile MotorState_t motor_state = MOTOR_STATE_DISARMED;

//用于rc_throttle_us 保存遥控器油门通道值。。
static volatile uint16_t rc_throttle_us = RC_THROTTLE_MIN;

/*
 * motor_output 保存最近一次真正发给电调的四路 DShot 值。
 *
 * 它主要用于 status 打印和调试观察。
 * 真正输出仍然发生在 Motor_Update() 里的 Dshot_WriteAll()。
 */
static volatile uint16_t motor_output[MOTOR_COUNT] = {0U, 0U, 0U, 0U};

//油门限幅函数
static uint16_t MotorClampRcThrottle(uint16_t value)
{

    //遥控油门低于 1000 时，按最低油门处理。
    if (value < RC_THROTTLE_MIN)
    {
        return RC_THROTTLE_MIN;
    }

    //遥控油门高于 2000 时，按最高油门处理。
    if (value > RC_THROTTLE_MAX)
    {
        return RC_THROTTLE_MAX;
    }

    //输入已经在 1000..2000 范围内，直接返回。
    return value;
}

static uint16_t MotorMapRcThrottleToDshot(uint16_t rc_value)
{
    uint32_t input_range;
    uint32_t output_range;
    uint32_t input_offset;
    uint32_t dshot_value;

    //输入进来先限幅
    rc_value = MotorClampRcThrottle(rc_value);

    input_range = RC_THROTTLE_MAX - RC_THROTTLE_MIN;
    //规定的输出最高限幅到认为规定的最低油门的跨度
    output_range = DSHOT_THROTTLE_MAX - MOTOR_IDLE_DSHOT;

    /*
     * input_offset:
     * 当前油门比最低油门高多少。
     * 例如 rc=1100 时，offset=100。
     */
    input_offset = rc_value - RC_THROTTLE_MIN;

    /*
     * ARM 后的油门映射：
     * 1000 -> MOTOR_IDLE_DSHOT，表示怠速旋转；
     * 2000 -> DSHOT_THROTTLE_MAX，表示最高油门。
     */
    dshot_value = MOTOR_IDLE_DSHOT +
                  (input_offset * output_range) / input_range;

    return (uint16_t)dshot_value;
}

//解锁函数
void Motor_Arm(void)
{
    taskENTER_CRITICAL();
    motor_state = MOTOR_STATE_ARMED;
    rc_throttle_us = RC_THROTTLE_MIN;

    taskEXIT_CRITICAL();
}

void Motor_Disarm(void)
{
    /*
     * DISARM 是安全动作，必须一次性把状态和输出缓存都清掉。
     */
    taskENTER_CRITICAL();

    /*
     * DISARM 后，Motor_Update 会持续发送 DShot 0。
     */
    motor_state = MOTOR_STATE_DISARMED;
    rc_throttle_us = RC_THROTTLE_MIN;

    /*
     * motor_output 是调试缓存。
     * 这里立即清零，可以让 status 打印立刻看到停机状态。
     */
    motor_output[0] = 0U;
    motor_output[1] = 0U;
    motor_output[2] = 0U;
    motor_output[3] = 0U;

    taskEXIT_CRITICAL();
}

void Motor_SetFailsafe(bool enabled)
{

    taskENTER_CRITICAL();

    if (enabled)
    {
        motor_state = MOTOR_STATE_FAILSAFE;
    }
    else if (motor_state == MOTOR_STATE_FAILSAFE)
    {
        motor_state = MOTOR_STATE_DISARMED;
    }

    taskEXIT_CRITICAL();
}

void Motor_SetRcThrottle(uint16_t value)
{
    /*
     * 这个函数是 USB 测试入口使用的。
     * 真实遥控器接入后，更推荐 CRSF 任务调用 Motor_SetRcInput()。
     */
    taskENTER_CRITICAL();

    rc_throttle_us = MotorClampRcThrottle(value);

    taskEXIT_CRITICAL();
}

void Motor_SetRcInput(const MotorRcInput_t *input)
{
    /*
     * 空指针保护。
     * 如果调用者传错参数，直接返回，不让系统崩溃。
     */
    if (input == NULL)
    {
        return;
    }

    /*
     * 更新遥控输入和电机状态要放在同一个临界区。
     * 否则可能出现“油门已经更新，但状态还没更新”的中间状态。
     */
    taskENTER_CRITICAL();

    /*
     * 保存遥控油门。
     * 注意保存的是 1000..2000 的遥控值，不是 DShot 值。
     */
    rc_throttle_us = MotorClampRcThrottle(input->throttle);

    /*
     * 优先级 1：failsafe。
     * 只要遥控器失控，直接进入 FAILSAFE。
     */
    if (input->failsafe)
    {
        motor_state = MOTOR_STATE_FAILSAFE;
    }

    /*
     * 优先级 2：ARM 开关关闭。
     * 遥控器正常，但飞手没有解锁，保持 DISARMED。
     */
    else if (!input->arm_switch)
    {
        motor_state = MOTOR_STATE_DISARMED;
    }

    /*
     * 优先级 3：遥控正常，并且 ARM 开关打开。
     * 这时才允许 Motor_Update() 输出怠速或油门。
     */
    else
    {
        motor_state = MOTOR_STATE_ARMED;
    }

    taskEXIT_CRITICAL();
}

void Motor_Update(void)
{
    MotorState_t state_snapshot;
    uint16_t throttle_snapshot;
    uint16_t m0;
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;

    /*
     * Motor_Update 是唯一真正调用 Dshot_WriteAll() 的地方。
     *
     * 这样做的目的：
     * - USB/CRSF/PID 只改目标状态，不直接碰 DShot DMA。
     * - DShot 的发送节奏固定由 Dshot_Task 控制。
     * - 避免多个任务同时启动同一个 TIM8 DMA。
     */

    taskENTER_CRITICAL();

    /*
     * 先把共享状态复制出来，后面的映射和 DShot 发送不占用临界区。
     */
    state_snapshot = motor_state;
    throttle_snapshot = rc_throttle_us;

    taskEXIT_CRITICAL();

    if (state_snapshot == MOTOR_STATE_ARMED)
    {
        /*
         * ARMED 时，把遥控油门转换成 DShot 输出。
         *
         * throttle_snapshot = 1000 -> dshot = MOTOR_IDLE_DSHOT
         * throttle_snapshot = 2000 -> dshot = DSHOT_THROTTLE_MAX
         */
        uint16_t dshot = MotorMapRcThrottleToDshot(throttle_snapshot);

        /*
         * 当前阶段还没做 PID 和混控。
         * 所以四个电机先同速输出，用来验证：
         * - ARM 开关有效
         * - throttle 能控制电机转速
         * - DShot 四路输出稳定
         */
        m0 = dshot;
        m1 = dshot;
        m2 = dshot;
        m3 = dshot;
    }
    else
    {
        /*
         * DISARMED 和 FAILSAFE 都是停机输出。
         */
        m0 = 0U;
        m1 = 0U;
        m2 = 0U;
        m3 = 0U;
    }

    taskENTER_CRITICAL();

    /*
     * 保存本次实际输出，供 Motor_GetStatus() 打印。
     */
    motor_output[0] = m0;
    motor_output[1] = m1;
    motor_output[2] = m2;
    motor_output[3] = m3;

    taskEXIT_CRITICAL();

    /*
     * 离开临界区后再发送 DShot。
     * Dshot_WriteAll() 会启动 TIM8 DMA，不应该放在长临界区里。
     */
    Dshot_WriteAll(m0, m1, m2, m3);
}

void Motor_GetStatus(MotorStatus_t *status)
{
    /*
     * status 是调用者提供的输出结构体。
     * 传 NULL 说明调用者不需要结果，直接返回。
     */
    if (status == NULL)
    {
        return;
    }

    /*
     * 把当前状态复制出去。
     * 复制过程进入临界区，避免读到一半 Motor_Update() 正在改 motor_output[]。
     */
    taskENTER_CRITICAL();

    status->state = motor_state;
    status->rc_throttle_us = rc_throttle_us;
    status->output[0] = motor_output[0];
    status->output[1] = motor_output[1];
    status->output[2] = motor_output[2];
    status->output[3] = motor_output[3];

    taskEXIT_CRITICAL();
}
