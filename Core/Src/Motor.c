#include "Motor.h"

#include "Dshot.h"
#include "Task.h"

#define MOTOR_TEST_MAX_DSHOT 300U

#define MOTOR_DSHOT_CMD_STOP                    0U
#define MOTOR_DSHOT_CMD_SPIN_DIRECTION_1        7U
#define MOTOR_DSHOT_CMD_SPIN_DIRECTION_2        8U
#define MOTOR_DSHOT_CMD_SAVE_SETTINGS           12U
#define MOTOR_DSHOT_CMD_SPIN_DIRECTION_NORMAL   20U
#define MOTOR_DSHOT_CMD_SPIN_DIRECTION_REVERSED 21U

#define MOTOR_ESC_PROGRAM_INITIAL_DELAY_TICKS   500U
#define MOTOR_ESC_PROGRAM_REPEAT_COUNT          10U
#define MOTOR_ESC_PROGRAM_SAVE_DELAY_TICKS      100U

typedef enum
{
    MOTOR_ESC_PROGRAM_IDLE = 0,
    MOTOR_ESC_PROGRAM_INITIAL_DELAY,
    MOTOR_ESC_PROGRAM_DIRECTION,
    MOTOR_ESC_PROGRAM_SAVE,
    MOTOR_ESC_PROGRAM_SAVE_DELAY,
} MotorEscProgramState_t;

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

/*
 * USB 单电机测试输出。
 *
 * motor_test_enabled = true 时，Motor_Update() 不再使用遥控器/ARM 状态，
 * 而是直接发送 motor_test_output[]。
 *
 * 这个模式只用于确认电机编号：
 *   m0 100 -> 只有 0 号电机输出 DShot 100
 *   stop   -> 退出测试并四路清零
 */
static volatile bool motor_test_enabled = false;
static volatile uint16_t motor_test_output[MOTOR_COUNT] = {0U, 0U, 0U, 0U};

/*
 * ESC 方向写入状态机。
 *
 * 这个状态机由 Motor_Update() 每 1ms 推进一次。
 * 这样 DShot 特殊命令仍然从 Dshot_Task 这条唯一通道发出去，
 * USB 命令只负责设置请求，不直接碰 TIM8/DMA。
 */
static volatile MotorEscProgramState_t motor_esc_program_state = MOTOR_ESC_PROGRAM_IDLE;
static volatile uint16_t motor_esc_program_delay = 0U;
static volatile uint16_t motor_esc_program_repeat = 0U;
static volatile uint16_t motor_esc_program_direction[MOTOR_COUNT] = {0U, 0U, 0U, 0U};
static volatile uint16_t motor_esc_program_save[MOTOR_COUNT] = {0U, 0U, 0U, 0U};

static volatile uint8_t motor_control_enabled = 0U;
static volatile uint16_t motor_control_output[MOTOR_COUNT] = {0U, 0U, 0U, 0U};

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

static void Motor_StartEscProgramLocked(void)
{
    /*
     * 调用者必须已经进入临界区。
     * 写 ESC 设置期间强制退出普通电机测试/ARM 输出。
     */
    motor_state = MOTOR_STATE_DISARMED;
    rc_throttle_us = RC_THROTTLE_MIN;

    motor_test_enabled = false;
    motor_test_output[0] = 0U;
    motor_test_output[1] = 0U;
    motor_test_output[2] = 0U;
    motor_test_output[3] = 0U;

    motor_output[0] = 0U;
    motor_output[1] = 0U;
    motor_output[2] = 0U;
    motor_output[3] = 0U;

    /*
     * 先等待约 500ms 的 0 输出，再开始发方向命令。
     * BLHeli_S 特殊命令只在电机停止状态执行，刚测完电机时要给它足够时间停稳。
     */
    motor_esc_program_state = MOTOR_ESC_PROGRAM_INITIAL_DELAY;
    motor_esc_program_delay = MOTOR_ESC_PROGRAM_INITIAL_DELAY_TICKS;
    motor_esc_program_repeat = 0U;
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
     * DISARM/STOP 必须同时退出 USB 单电机测试模式。
     */
    motor_test_enabled = false;
    motor_test_output[0] = 0U;
    motor_test_output[1] = 0U;
    motor_test_output[2] = 0U;
    motor_test_output[3] = 0U;

    /*
     * 同时取消还没完成的 ESC 方向写入请求。
     */
    motor_esc_program_state = MOTOR_ESC_PROGRAM_IDLE;
    motor_esc_program_delay = 0U;
    motor_esc_program_repeat = 0U;
    motor_esc_program_direction[0] = 0U;
    motor_esc_program_direction[1] = 0U;
    motor_esc_program_direction[2] = 0U;
    motor_esc_program_direction[3] = 0U;
    motor_esc_program_save[0] = 0U;
    motor_esc_program_save[1] = 0U;
    motor_esc_program_save[2] = 0U;
    motor_esc_program_save[3] = 0U;

    /*
     * motor_output 是调试缓存。
     * 这里立即清零，可以让 status 打印立刻看到停机状态。
     */
    motor_output[0] = 0U;
    motor_output[1] = 0U;
    motor_output[2] = 0U;
    motor_output[3] = 0U;

    motor_control_enabled = 0U;
    motor_control_output[0] = 0U;
    motor_control_output[1] = 0U;
    motor_control_output[2] = 0U;
    motor_control_output[3] = 0U;

    taskEXIT_CRITICAL();
}

void Motor_ClearTestOutput(void)
{
    /*
     * 退出 USB 单电机测试模式，并把测试缓存清零。
     */
    taskENTER_CRITICAL();

    motor_test_enabled = false;
    motor_test_output[0] = 0U;
    motor_test_output[1] = 0U;
    motor_test_output[2] = 0U;
    motor_test_output[3] = 0U;

    motor_esc_program_state = MOTOR_ESC_PROGRAM_IDLE;
    motor_esc_program_delay = 0U;
    motor_esc_program_repeat = 0U;
    motor_esc_program_direction[0] = 0U;
    motor_esc_program_direction[1] = 0U;
    motor_esc_program_direction[2] = 0U;
    motor_esc_program_direction[3] = 0U;
    motor_esc_program_save[0] = 0U;
    motor_esc_program_save[1] = 0U;
    motor_esc_program_save[2] = 0U;
    motor_esc_program_save[3] = 0U;

    taskEXIT_CRITICAL();
}

void Motor_SetSingleTestOutput(uint8_t motor_index, uint16_t dshot_value)
{
    /*
     * 只允许 0..3 号电机。
     */
    if (motor_index >= MOTOR_COUNT)
    {
        return;
    }

    /*
     * 单电机编号测试不需要大油门。
     * 这里限制到 300，避免误输入 1000/2000 这种危险值。
     */
    if (dshot_value > MOTOR_TEST_MAX_DSHOT)
    {
        dshot_value = MOTOR_TEST_MAX_DSHOT;
    }

    taskENTER_CRITICAL();

    /*
     * 每次 m0/m1/m2/m3 命令都只让一个电机转。
     * 这样确认编号时不会被上一条命令残留影响。
     */
    motor_test_output[0] = 0U;
    motor_test_output[1] = 0U;
    motor_test_output[2] = 0U;
    motor_test_output[3] = 0U;

    motor_test_output[motor_index] = dshot_value;
    motor_test_enabled = true;

    taskEXIT_CRITICAL();
}

void Motor_RequestEscDirectionFix(void)
{
    /*
     * Props Out 最终方向写入：
     *
     * 物理位置：
     *   m1 左前
     *   m3 右前
     *   m0 左后
     *   m2 右后
     *
     * 目标转向：
     *   m0 左后 -> CCW -> direction 2
     *   m1 左前 -> CW  -> direction 2
     *   m2 右后 -> CW  -> direction 2
     *   m3 右前 -> CCW -> direction 1
     *
     * BLHeli_S 的 DShot 方向命令主要是 7/8：
     * 7 = spin direction 1
     * 8 = spin direction 2
     *
     * 这里只设置请求，真正发送 DShot 命令由 Motor_Update() 完成。
     */
    taskENTER_CRITICAL();

    /*
     * 这是“绝对设置”，不是“翻转一次”。
     */
    motor_esc_program_direction[0] = MOTOR_DSHOT_CMD_SPIN_DIRECTION_2;
    motor_esc_program_direction[1] = MOTOR_DSHOT_CMD_SPIN_DIRECTION_2;
    motor_esc_program_direction[2] = MOTOR_DSHOT_CMD_SPIN_DIRECTION_2;
    motor_esc_program_direction[3] = MOTOR_DSHOT_CMD_SPIN_DIRECTION_1;

    motor_esc_program_save[0] = MOTOR_DSHOT_CMD_SAVE_SETTINGS;
    motor_esc_program_save[1] = MOTOR_DSHOT_CMD_SAVE_SETTINGS;
    motor_esc_program_save[2] = MOTOR_DSHOT_CMD_SAVE_SETTINGS;
    motor_esc_program_save[3] = MOTOR_DSHOT_CMD_SAVE_SETTINGS;

    Motor_StartEscProgramLocked();

    taskEXIT_CRITICAL();
}

void Motor_RequestEscDirection(uint8_t motor_index, bool reversed)
{
    /*
     * 设置单个 BLHeli_S ESC 的绝对方向。
     *
     * 这里沿用 USB 命令里的 n/r 名字，但实际发送的是 BLHeli_S 的：
     * n -> DSHOT_CMD_SPIN_DIRECTION_1 (7)
     * r -> DSHOT_CMD_SPIN_DIRECTION_2 (8)
     */
    if (motor_index >= MOTOR_COUNT)
    {
        return;
    }

    taskENTER_CRITICAL();

    motor_esc_program_direction[0] = MOTOR_DSHOT_CMD_STOP;
    motor_esc_program_direction[1] = MOTOR_DSHOT_CMD_STOP;
    motor_esc_program_direction[2] = MOTOR_DSHOT_CMD_STOP;
    motor_esc_program_direction[3] = MOTOR_DSHOT_CMD_STOP;

    motor_esc_program_save[0] = MOTOR_DSHOT_CMD_STOP;
    motor_esc_program_save[1] = MOTOR_DSHOT_CMD_STOP;
    motor_esc_program_save[2] = MOTOR_DSHOT_CMD_STOP;
    motor_esc_program_save[3] = MOTOR_DSHOT_CMD_STOP;

    motor_esc_program_direction[motor_index] =
        reversed ? MOTOR_DSHOT_CMD_SPIN_DIRECTION_2
                 : MOTOR_DSHOT_CMD_SPIN_DIRECTION_1;

    motor_esc_program_save[motor_index] = MOTOR_DSHOT_CMD_SAVE_SETTINGS;

    Motor_StartEscProgramLocked();

    taskEXIT_CRITICAL();
}

void Motor_RequestEscRawCommand(uint8_t motor_index, uint16_t command, bool save_after)
{
    /*
     * 给单个 ESC 发送原始 DShot 特殊命令。
     *
     * 只用于调试 ESC 支持情况：
     *   cmd0 7
     *   cmd0 8
     *   cmd0 20
     *   cmd0 21
     *
     * 0..47 是 DShot 特殊命令区，超过范围直接忽略。
     */
    if ((motor_index >= MOTOR_COUNT) || (command > 47U))
    {
        return;
    }

    taskENTER_CRITICAL();

    motor_esc_program_direction[0] = MOTOR_DSHOT_CMD_STOP;
    motor_esc_program_direction[1] = MOTOR_DSHOT_CMD_STOP;
    motor_esc_program_direction[2] = MOTOR_DSHOT_CMD_STOP;
    motor_esc_program_direction[3] = MOTOR_DSHOT_CMD_STOP;

    motor_esc_program_save[0] = MOTOR_DSHOT_CMD_STOP;
    motor_esc_program_save[1] = MOTOR_DSHOT_CMD_STOP;
    motor_esc_program_save[2] = MOTOR_DSHOT_CMD_STOP;
    motor_esc_program_save[3] = MOTOR_DSHOT_CMD_STOP;

    motor_esc_program_direction[motor_index] = command;

    if (save_after)
    {
        motor_esc_program_save[motor_index] = MOTOR_DSHOT_CMD_SAVE_SETTINGS;
    }

    Motor_StartEscProgramLocked();

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
    bool test_enabled_snapshot;
    bool command_frame = false;
    uint16_t m0;
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;

    uint8_t control_enabled_snapshot;
    uint16_t control_m0;
    uint16_t control_m1;
    uint16_t control_m2;
    uint16_t control_m3;


    /*
     * Motor_Update 是唯一真正调用 Dshot_WriteAll() 的地方。
     *
     * 这样做的目的：
     * - USB/CRSF/PID 只改目标状态，不直接碰 DShot DMA。
     * - DShot 的发送节奏固定由 Dshot_Task 控制。
     * - 避免多个任务同时启动同一个 TIM8 DMA。
     */
    if (!Dshot_Ready())
    {
        return;
    }

    taskENTER_CRITICAL();

    /*
     * 先把共享状态复制出来，后面的映射和 DShot 发送不占用临界区。
     */
    state_snapshot = motor_state;
    throttle_snapshot = rc_throttle_us;
    test_enabled_snapshot = motor_test_enabled;

    control_enabled_snapshot = motor_control_enabled;
    control_m0 = motor_control_output[0];
    control_m1 = motor_control_output[1];
    control_m2 = motor_control_output[2];
    control_m3 = motor_control_output[3];

    if (motor_esc_program_state != MOTOR_ESC_PROGRAM_IDLE)
    {
        /*
         * ESC 方向写入流程优先级最高。
         *
         * 流程：
         * 1. 先发 0，等待约 500ms，让 ESC 确认电机已停止；
         * 2. 连续 10 次发送方向命令；
         * 3. 连续 10 次发送保存命令：m0/m1/m2 save，m3 stop；
         * 4. 保存后继续发 0，等待约 100ms。
         */
        switch (motor_esc_program_state)
        {
            case MOTOR_ESC_PROGRAM_INITIAL_DELAY:
                m0 = MOTOR_DSHOT_CMD_STOP;
                m1 = MOTOR_DSHOT_CMD_STOP;
                m2 = MOTOR_DSHOT_CMD_STOP;
                m3 = MOTOR_DSHOT_CMD_STOP;

                if (motor_esc_program_delay > 0U)
                {
                    motor_esc_program_delay--;
                }
                else
                {
                    motor_esc_program_state = MOTOR_ESC_PROGRAM_DIRECTION;
                    motor_esc_program_repeat = MOTOR_ESC_PROGRAM_REPEAT_COUNT;
                }
                break;

            case MOTOR_ESC_PROGRAM_DIRECTION:
                command_frame = true;
                m0 = motor_esc_program_direction[0];
                m1 = motor_esc_program_direction[1];
                m2 = motor_esc_program_direction[2];
                m3 = motor_esc_program_direction[3];

                if (motor_esc_program_repeat > 0U)
                {
                    motor_esc_program_repeat--;
                }

                if (motor_esc_program_repeat == 0U)
                {
                    motor_esc_program_state = MOTOR_ESC_PROGRAM_SAVE;
                    motor_esc_program_repeat = MOTOR_ESC_PROGRAM_REPEAT_COUNT;
                }
                break;

            case MOTOR_ESC_PROGRAM_SAVE:
                command_frame = true;
                m0 = motor_esc_program_save[0];
                m1 = motor_esc_program_save[1];
                m2 = motor_esc_program_save[2];
                m3 = motor_esc_program_save[3];

                if (motor_esc_program_repeat > 0U)
                {
                    motor_esc_program_repeat--;
                }

                if (motor_esc_program_repeat == 0U)
                {
                    motor_esc_program_state = MOTOR_ESC_PROGRAM_SAVE_DELAY;
                    motor_esc_program_delay = MOTOR_ESC_PROGRAM_SAVE_DELAY_TICKS;
                }
                break;

            case MOTOR_ESC_PROGRAM_SAVE_DELAY:
                m0 = MOTOR_DSHOT_CMD_STOP;
                m1 = MOTOR_DSHOT_CMD_STOP;
                m2 = MOTOR_DSHOT_CMD_STOP;
                m3 = MOTOR_DSHOT_CMD_STOP;

                if (motor_esc_program_delay > 0U)
                {
                    motor_esc_program_delay--;
                }
                else
                {
                    motor_esc_program_state = MOTOR_ESC_PROGRAM_IDLE;
                    motor_esc_program_direction[0] = 0U;
                    motor_esc_program_direction[1] = 0U;
                    motor_esc_program_direction[2] = 0U;
                    motor_esc_program_direction[3] = 0U;
                    motor_esc_program_save[0] = 0U;
                    motor_esc_program_save[1] = 0U;
                    motor_esc_program_save[2] = 0U;
                    motor_esc_program_save[3] = 0U;
                }
                break;

            default:
                m0 = 0U;
                m1 = 0U;
                m2 = 0U;
                m3 = 0U;
                motor_esc_program_state = MOTOR_ESC_PROGRAM_IDLE;
                break;
        }
    }
    else if (test_enabled_snapshot)
    {
        /*
         * USB 单电机测试模式优先级最高。
         * 这里直接取测试缓存，不看 ARM/CRSF/failsafe。
         */
        m0 = motor_test_output[0];
        m1 = motor_test_output[1];
        m2 = motor_test_output[2];
        m3 = motor_test_output[3];
    }
    else if (state_snapshot == MOTOR_STATE_ARMED)
    {
        /*
         * ARMED 只表示“遥控器允许电机输出”。
         *
         * 真正输出什么，由 Control_Task 算好的 control output 决定。
         *
         * 如果 control_enabled_snapshot == 1：
         *   说明 IMU 正常、角度安全、LEVEL 允许、油门低位解锁已经通过。
         *   这时发送 Control_Update() 混控后的四路输出。
         *
         * 如果 control_enabled_snapshot == 0：
         *   说明控制器认为当前不安全。
         *   比如角度超过 25 度、IMU 无效、failsafe、LEVEL 关闭。
         *   这时必须四路输出 0。
         */
        if (control_enabled_snapshot != 0U)
        {
            m0 = control_m0;
            m1 = control_m1;
            m2 = control_m2;
            m3 = control_m3;
        }
        else
        {
            m0 = 0U;
            m1 = 0U;
            m2 = 0U;
            m3 = 0U;
        }
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
    if (command_frame)
    {
        Dshot_WriteAllCommand(m0, m1, m2, m3);
    }
    else
    {
        Dshot_WriteAll(m0, m1, m2, m3);
    }
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

void Motor_SetControlOutput(const ControlOutput_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    if (out->enabled)
    {
        motor_control_enabled = 1U;
        motor_control_output[0] = out->motor[0];
        motor_control_output[1] = out->motor[1];
        motor_control_output[2] = out->motor[2];
        motor_control_output[3] = out->motor[3];
    }
    else
    {
        motor_control_enabled = 0U;
        motor_control_output[0] = 0;
        motor_control_output[1] = 0;
        motor_control_output[2] = 0;
        motor_control_output[3] = 0;
    }
    taskEXIT_CRITICAL();
}