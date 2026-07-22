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
#include "Motor.h"
#include "UsbCommand.h"
#include "crsf.h"

RingBuffer_t Rx_buffer;
extern List_t pxReadyTasksLists[configMAX_PRIORITIES];
extern Mutex_t usb_tx_mutex;
extern volatile uint32_t g_boot_stage;

#define IMU_STACK_SIZE 2048
StackType_t IMUStack[IMU_STACK_SIZE];
TCB_t IMUTaskTCB;
TaskHandle_t IMUTaskHandle = NULL;

#define USB_STACK_SIZE 4096
StackType_t USBStack[USB_STACK_SIZE];
TCB_t USBTaskTCB;
TaskHandle_t USBTaskHandle = NULL;

#define TEST_STACK_SIZE 2048
StackType_t TestStack[TEST_STACK_SIZE];
TCB_t TestTaskTCB;
TaskHandle_t TestTaskHandle = NULL;

#define DSHOT_STACK_SIZE 2048
StackType_t DshotStack[DSHOT_STACK_SIZE];
TCB_t DshotTaskTCB;
TaskHandle_t DshotTaskHandle = NULL;

#define CRSF_STACK_SIZE 2048
StackType_t CRSFStack[CRSF_STACK_SIZE];
TCB_t CRSFTaskTCB;
TaskHandle_t CRSFTaskHandle = NULL;

IMU_RawData raw_data;
IMU_SensorData sensor_data;
AttitudeData att_data;

static Queue_t imu_drdy_queue; //IMU DRDY 事件队列控制块(队列本体，里面存读写指针，队列长度，等待任务链表等信息)
static uint8_t imu_drdy_queue_storage[1];//队列存储区：队列长度1,每个元素1字节（队列存数据的地方）
static QueueHandle_t imu_drdy_queue_handle = NULL;//队列句柄，创建成功后指向imu_drdy_queue（IMU_Task和EXTIcallback通过它操纵队列）

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
     MotorStatus_t status;
     uint16_t ch[CRSF_NUM_CHANNELS];
     const CRSF_Stats *crsf_stats;

    (void)pvParameters;

     MX_USB_DEVICE_Init();

    for (;;)
     {
        Betaflight_USB_Server();
         UsbCommand_ProcessRx();

         print_tick++;

         if (print_tick>=100)
         {
             print_tick = 0;

             Motor_GetStatus(&status);

             CRSF_GetChannels(ch);

             crsf_stats = CRSF_GetStats();

             // usb_log_printf("STATE:%u THR:%u M:%u %u %u %u",
             //     (unsigned)status.state,
             //     (unsigned)status.rc_throttle_us,
             //     (unsigned)status.output[0],
             //     (unsigned)status.output[1],
             //     (unsigned)status.output[2],
             //     (unsigned)status.output[3]);

             usb_log_printf(
     "CRSF link:%u ch:%u %u %u %u %u %u %u %u %u %u %u %u ok:%lu crc:%lu len:%lu drop:%lu",
     (unsigned)CRSF_IsLinkUp(),

     (unsigned)ch[0],
     (unsigned)ch[1],
     (unsigned)ch[2],
     (unsigned)ch[3],
     (unsigned)ch[4],
     (unsigned)ch[5],
     (unsigned)ch[6],
     (unsigned)ch[7],
     (unsigned)ch[8],
     (unsigned)ch[9],
     (unsigned)ch[10],
     (unsigned)ch[11],

     (unsigned long)crsf_stats->frame_ok,
     (unsigned long)crsf_stats->frame_crc_err,
     (unsigned long)crsf_stats->frame_len_err,
     (unsigned long)usb_log_dropped_count()
           );
         }

         HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_3);
         vTaskDelay(1);
    }
}

void Test_Task(void *pvParameters)
{
    for (;;)
    {
        HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_0);
        vTaskDelay(200);
    }

}

void Dshot_Task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        Motor_Update();
        vTaskDelay(1);
    }
}

void CRSF_Task(void *pvParameters)
{
    (void)pvParameters;               // 当前没用到任务参数，避免编译警告

    uint16_t ch[CRSF_NUM_CHANNELS];   // 保存一次读取出来的 CRSF 通道值

    MotorRcInput_t input;             // 准备喂给 Motor 层的遥控输入结构体

    for (;;)
    {
        CRSF_GetChannels(ch);         // 从 CRSF 驱动里复制当前所有通道

        input.throttle = CRSF_MapRawToUs(ch[2]);
        // 一般 ch[2] 是油门，具体要看你的遥控器通道映射

        input.arm_switch = ch[4] > CRSF_CHANNEL_MID;
        // 假设 ch[4] 是 ARM 开关
        // 大于中点表示解锁，小于中点表示锁定

        input.failsafe = !CRSF_IsLinkUp();
        // 如果 CRSF 超时，直接进入 failsafe

        Motor_SetRcInput(&input);     // 把遥控器状态交给 Motor 层
        // Motor 层决定输出 0、怠速，还是油门值

        vTaskDelay(1);                // 1ms 更新一次遥控器输入，足够当前阶段测试
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

    // IMUTaskHandle = xTaskCreateStatic(
    //     IMU_Task,
    //     "IMUTask",
    //     IMU_STACK_SIZE,
    //     NULL,
    //     IMUStack,
    //     &IMUTaskTCB
    // );
    // IMUTaskTCB.uxPriority = 3;
    //
    USBTaskHandle = xTaskCreateStatic(
    USB_Task,
    "USBTask",
    USB_STACK_SIZE,
    NULL,
    USBStack,
    &USBTaskTCB
    );
    USBTaskTCB.uxPriority = 2;

    TestTaskHandle = xTaskCreateStatic(
    Test_Task,
    "TestTask",
    TEST_STACK_SIZE,
    NULL,
    TestStack,
    &TestTaskTCB
    );
    TestTaskTCB.uxPriority = 1;

    DshotTaskHandle = xTaskCreateStatic(
    Dshot_Task,
    "DshotTask",
    DSHOT_STACK_SIZE,
    NULL,
    DshotStack,
    &DshotTaskTCB
    );
    DshotTaskTCB.uxPriority = 3;

    CRSFTaskHandle = xTaskCreateStatic(
    CRSF_Task,
    "CRSFTask",
    CRSF_STACK_SIZE,
    NULL,
    CRSFStack,
    &CRSFTaskTCB
);
    CRSFTaskTCB.uxPriority = 4;

    vListInsertEnd(&(pxReadyTasksLists[2]), &(USBTaskTCB.xStateListItem));
    // vListInsertEnd(&(pxReadyTasksLists[3]), &(IMUTaskTCB.xStateListItem));
    vListInsertEnd(&(pxReadyTasksLists[1]), &(TestTaskTCB.xStateListItem));
    vListInsertEnd(&(pxReadyTasksLists[3]), &(DshotTaskTCB.xStateListItem));
    vListInsertEnd(&(pxReadyTasksLists[4]), &(CRSFTaskTCB.xStateListItem));
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
