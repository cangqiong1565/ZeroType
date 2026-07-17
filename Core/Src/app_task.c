#include "main.h"
#include "app_task.h"
#include "Task.h"
#include "ring_buffer.h"
#include <string.h>
#include "ICM42688.h"
#include "os_mutex.h"
#include "usbd_cdc.h"
#include "retarget.h"
#include "usb_device.h"
#include "stm32h7xx.h"
#include "MahonyAHRS.h"
#include <math.h>
#include "Queue.h"
#include <stdbool.h>

#include "Dshot.h"

#define DSHOT_ARM_TIME_MS 3000U
#define DSHOT_STEP        25U

RingBuffer_t Rx_buffer;
extern List_t pxReadyTasksLists[configMAX_PRIORITIES];
extern Mutex_t usb_tx_mutex;

#define IMU_STACK_SIZE 1024
StackType_t IMUStack[IMU_STACK_SIZE];
TCB_t IMUTaskTCB;
TaskHandle_t IMUTaskHandle = NULL;

#define USB_STACK_SIZE 1024
StackType_t USBStack[USB_STACK_SIZE];
TCB_t USBTaskTCB;
TaskHandle_t USBTaskHandle = NULL;

IMU_RawData raw_data;
IMU_SensorData sensor_data;
AttitudeData att_data;

static Queue_t imu_drdy_queue; //IMU DRDY 事件队列控制块(队列本体，里面存读写指针，队列长度，等待任务链表等信息)
static uint8_t imu_drdy_queue_storage[1];//队列存储区：队列长度1,每个元素1字节（队列存数据的地方）
static QueueHandle_t imu_drdy_queue_handle = NULL;//队列句柄，创建成功后指向imu_drdy_queue（IMU_Task和EXTIcallback通过它操纵队列）

static uint8_t dshot_test_motor;
static uint16_t dshot_target_throttle = 0;       // 串口命令设置的目标油门
static uint16_t dshot_output_throttle = 0;       // 当前实际发给电调的油门
static uint32_t dshot_arm_ticks = 0;             // 上电后持续发送 0 油门的计数
static bool dshot_armed = false;                 // true 表示已经完成 0 油门解锁阶段

static bool UsbCmdEqual(const char *cmd, const char *target)
{
    while ((*cmd != '\0') && (*target != '\0'))
    {
        char a = *cmd;
        char b = *target;

        if ((a >= 'A') && (a <= 'Z'))
        {
            a = (char)(a - 'A' + 'a');
        }

        if ((b >= 'A') && (b <= 'Z'))
        {
            b = (char)(b - 'A' + 'a');
        }

        if (a != b)
        {
            return false;
        }

        cmd++;
        target++;
    }

    return (*cmd == '\0') && (*target == '\0');
}

static bool UsbCmdParseUint16(const char *cmd, uint16_t *value)
{
    uint32_t result = 0;
    bool has_digit = false;

    while ((*cmd == ' ') || (*cmd == '\t'))
    {
        cmd++;
    }

    while ((*cmd >= '0') && (*cmd <= '9'))
    {
        has_digit = true;
        result = result * 10U + (uint32_t)(*cmd - '0');

        if (result > 65535U)
        {
            return false;
        }

        cmd++;
    }

    while ((*cmd == ' ') || (*cmd == '\t'))
    {
        cmd++;
    }

    if ((!has_digit) || (*cmd != '\0') || (value == NULL))
    {
        return false;
    }

    *value = (uint16_t)result;
    return true;
}

static uint16_t DshotClampThrottle(uint16_t value)
{
    if (value == 0U)
    {
        return 0U;
    }

    if (value < DSHOT_THROTTLE_MIN)
    {
        return DSHOT_THROTTLE_MIN;
    }

    if (value > DSHOT_THROTTLE_MAX)
    {
        return DSHOT_THROTTLE_MAX;
    }

    return value;
}

static void DshotSetTargetThrottle(uint16_t value)
{
    if ((!dshot_armed) && (value != 0U))
    {
        usb_log_printf("DShot not armed yet, keep throttle 0");
        return;
    }

    dshot_target_throttle = DshotClampThrottle(value);

    usb_log_printf("DShot target: %u", (unsigned)dshot_target_throttle);
}

static void DshotHandleCommand(const char *cmd)
{
    uint8_t motor;
    uint16_t value;

    if ((cmd == NULL) || (*cmd == '\0'))
    {
        return;
    }

    if ((cmd[0] == 'm') || (cmd[0] == 'M'))
    {

        if ((cmd[1] < '0') || (cmd[1] > '3'))
        {
            usb_log_printf("Bad motor");
            return;
        }

        if (cmd[2] != ' ')
        {
            usb_log_printf("Use: m0 100");
            return;
        }

        motor = (uint8_t)(cmd[1] - '0');

        if (!UsbCmdParseUint16(&cmd[3], &value))
        {
            usb_log_printf("Use:m0 100");
            return;
        }

        dshot_test_motor = motor;
        DshotSetTargetThrottle(value);

        usb_log_printf("Motor %u target %u",(unsigned)dshot_test_motor,(unsigned)dshot_target_throttle);

        return;

    }

    if (UsbCmdEqual(cmd, "help") || UsbCmdEqual(cmd, "?"))
    {
        usb_log_printf("Commands: arm, stop, 0..2047, +, -");
        return;
    }

    if (UsbCmdEqual(cmd, "arm"))
    {
        /*
         * 重新进入解锁流程：目标油门清零，然后连续发送一段时间 0 油门。
         * 电调看到稳定合法的 DShot 0 后，才会接受后续油门。
         */
        dshot_target_throttle = 0U;
        dshot_output_throttle = 0U;
        dshot_arm_ticks = 0U;
        dshot_armed = false;
        usb_log_printf("DShot arming: sending zero throttle");
        return;
    }

    if (UsbCmdEqual(cmd, "stop") || UsbCmdEqual(cmd, "disarm"))
    {
        DshotSetTargetThrottle(0U);
        return;
    }

    if (UsbCmdEqual(cmd, "+"))
    {
        if (dshot_target_throttle == 0U)
        {
            DshotSetTargetThrottle(DSHOT_THROTTLE_MIN);
        }
        else if (dshot_target_throttle <= (DSHOT_THROTTLE_MAX - DSHOT_STEP))
        {
            DshotSetTargetThrottle((uint16_t)(dshot_target_throttle + DSHOT_STEP));
        }
        else
        {
            DshotSetTargetThrottle(DSHOT_THROTTLE_MAX);
        }

        return;
    }

    if (UsbCmdEqual(cmd, "-"))
    {
        if (dshot_target_throttle <= DSHOT_THROTTLE_MIN)
        {
            DshotSetTargetThrottle(0U);
        }
        else if (dshot_target_throttle < (DSHOT_THROTTLE_MIN + DSHOT_STEP))
        {
            DshotSetTargetThrottle(DSHOT_THROTTLE_MIN);
        }
        else
        {
            DshotSetTargetThrottle((uint16_t)(dshot_target_throttle - DSHOT_STEP));
        }

        return;
    }

    if (UsbCmdParseUint16(cmd, &value))
    {
        DshotSetTargetThrottle(value);
        return;
    }

    usb_log_printf("Bad command: %s", cmd);
}

static void DshotProcessUsbRx(void)
{
    static char line[24];
    static uint8_t line_len = 0;
    uint8_t ch;

    while (usb_rx_get_byte(&ch) != 0U)
    {
        if ((ch == '\r') || (ch == '\n'))
        {
            if (line_len > 0U)
            {
                line[line_len] = '\0';
                DshotHandleCommand(line);
                line_len = 0U;
            }
        }
        else if ((ch == '\b') || (ch == 0x7FU))
        {
            if (line_len > 0U)
            {
                line_len--;
            }
        }
        else if ((ch >= 32U) && (ch <= 126U))
        {
            if (line_len < (sizeof(line) - 1U))
            {
                line[line_len] = (char)ch;
                line_len++;
            }
            else
            {
                line_len = 0U;
                usb_log_printf("Command too long");
            }
        }
    }
}

static void DshotUpdateOutput(void)
{
    uint16_t motor[4] = {0U, 0U, 0U, 0U};

    if (!dshot_armed)
    {
        if (dshot_arm_ticks < DSHOT_ARM_TIME_MS)
        {
            dshot_arm_ticks++;
        }
        else
        {
            /*
             * 3 秒 0 油门完成后，认为电调已经识别到合法 DShot 信号。
             */
            dshot_armed = true;
            usb_log_printf("DShot armed, input throttle by USB");
        }
    }
    else
    {
        motor[dshot_test_motor] = dshot_target_throttle;
    }

    dshot_output_throttle = motor[dshot_test_motor];

    if (Dshot_Ready())
    {
        Dshot_WriteAll(motor[0],motor[1],motor[2],motor[3]);
    }
}

static bool ImuSensorDataValid(const IMU_SensorData *s) {
    if (s == NULL) {
        return false;
    }

    float acc_sq = s->accel_x * s->accel_x + s->accel_y * s->accel_y +s->accel_z * s->accel_z;

    if (acc_sq < 0.25f || acc_sq > 2.25f) {
        return false;
    }

    return true;
}

static void ComputeEuler(AttitudeData *a)
{
    float q0_=q0,q1_=q1,q2_=q2,q3_=q3;
    a->roll =atan2f(2*q0_*q1_+2*q2_*q3_,1-2*q1_*q1_-2*q2_*q2_)*57.29578f;
    float sp=2*(q0_*q2_-q3_*q1_);if(sp>1)sp=1;else if(sp<-1)sp=-1;
    a->pitch=asinf(sp)*57.29578f;
    a->yaw  =atan2f(2*q0_*q3_+2*q1_*q2_,1-2*q2_*q2_-2*q3_*q3_)*57.29578f;
}


void IMU_Task(void *pvParameters)
{
    uint8_t imu_drdy_event = 0;

    for (;;)
    {
        if (xQueueGenericReceive(imu_drdy_queue_handle,
                                 &imu_drdy_event,
                                 portMAX_DELAY) == pdPASS)
        {
            if (ICM42688_ReadRaw(&raw_data) != ICM42688_OK)
            {
                continue;
            }

            ICM42688_UpdateBias(&raw_data);

            ICM42688_ConvertRaw(&raw_data, &sensor_data);

            if (!ImuSensorDataValid(&sensor_data))
            {
                continue;
            }

            MahonyAHRS_update(sensor_data.gyro_x,
                              sensor_data.gyro_y,
                              sensor_data.gyro_z,
                              sensor_data.accel_x,
                              sensor_data.accel_y,
                              sensor_data.accel_z,
                              0.001f);
            ComputeEuler(&att_data);
            /* 后面接欧拉角计算 / PID */
        }
    }
}


void USB_Task(void *pvParameters)
{
    static uint32_t print_tick = 0;

    for (;;)
    {
        Betaflight_USB_Server();
        DshotProcessUsbRx();
        DshotUpdateOutput();

        print_tick++;

        if (print_tick>=100)
        {
            print_tick = 0;

            usb_log_printf("P:%.1f R:%.1f Y:%.1f OUT:%u SET:%u ARM:%u",
                att_data.pitch,
                att_data.roll,
                att_data.yaw,
                (unsigned)dshot_output_throttle,
                (unsigned)dshot_target_throttle,
                (unsigned)dshot_armed);
        }

        HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_3);
        vTaskDelay(1);
    }
}

void AppTaskInit(void)
{
    imu_drdy_queue_handle = xQueueCreateStatic(
        1,//队列长度
        sizeof(uint8_t),//每个队列元素大小
        imu_drdy_queue_storage,//队列数据存储区
        &imu_drdy_queue//队列控制块
        );

    IMUTaskHandle = xTaskCreateStatic(
        IMU_Task,
        "IMUTask",
        IMU_STACK_SIZE,
        NULL,
        IMUStack,
        &IMUTaskTCB
    );
    IMUTaskTCB.uxPriority = 3;

    USBTaskHandle = xTaskCreateStatic(
    USB_Task,
    "USBTask",
    USB_STACK_SIZE,
    NULL,
    USBStack,
    &USBTaskTCB
    );
    USBTaskTCB.uxPriority = 2;

    vListInsertEnd(&(pxReadyTasksLists[2]), &(USBTaskTCB.xStateListItem));
    vListInsertEnd(&(pxReadyTasksLists[3]), &(IMUTaskTCB.xStateListItem));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_6)                                      // 只处理 PD6 对应的 EXTI 线，也就是 ICM42688 DRDY
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;               // 记录这次发队列是否唤醒了需要立刻运行的任务

        uint8_t imu_drdy_event = 1;                                  // 队列里发送的事件值，1 表示 IMU 数据就绪

        if (imu_drdy_queue_handle != NULL)                           // 确认队列已经创建，避免系统初始化早期误进中断
        {
            (void)xQueueGenericSendFromISR(
                imu_drdy_queue_handle,                               // 目标队列：IMU DRDY 事件队列
                &imu_drdy_event,                                     // 要发送的数据地址
                &xHigherPriorityTaskWoken                            // 如果唤醒了等待队列的高优先级任务，这里会被置 pdTRUE
            );

            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);            // 如果需要任务切换，就在中断退出时触发 PendSV
        }
    }
}
