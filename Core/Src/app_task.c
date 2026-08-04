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
#include "SPL06.h"
#include "Control.h"

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

#define CONTROL_STACK_SIZE 2048
StackType_t ControlStack[CONTROL_STACK_SIZE];
TCB_t ControlTaskTCB;
TaskHandle_t ControlTaskHandle = NULL;

#define BARO_STACK_SIZE 2048
StackType_t BaroStack[BARO_STACK_SIZE];
TCB_t BaroTaskTCB;
TaskHandle_t BaroTaskHandle = NULL;

IMU_RawData raw_data;
IMU_SensorData sensor_data;
AttitudeData att_data;

static Queue_t imu_drdy_queue; //IMU DRDY 事件队列控制块(队列本体，里面存读写指针，队列长度，等待任务链表等信息)
static uint8_t imu_drdy_queue_storage[1];//队列存储区：队列长度1,每个元素1字节（队列存数据的地方）
static QueueHandle_t imu_drdy_queue_handle = NULL;//队列句柄，创建成功后指向imu_drdy_queue（IMU_Task和EXTIcallback通过它操纵队列）

static ControlOutput_t control_output;

static SPL06_t baro_dev;
static SPL06_Data_t baro_data;

static volatile uint8_t baro_inited = 0U;
static volatile uint8_t baro_valid = 0U;
static volatile SPL06_Status_t baro_status = SPL06_ERROR;

static float baro_ref_pressure_pa = 0.0f;
static float baro_altitude_m = 0.0f;

//读取控制输出是否使能，IMU_Task用它判断现在能不能做动态gyro零偏修正
static bool ControlOutputEnabledSnapshot(void)
{
    bool enabled;

    taskENTER_CRITICAL();
    enabled = control_output.enabled;
    taskEXIT_CRITICAL();

    return enabled;
}

static bool ImuSensorDataValid(const IMU_SensorData *s) {
    if (s == NULL) {
        return false;
    }

    // float acc_sq = s->accel_x * s->accel_x + s->accel_y * s->accel_y +s->accel_z * s->accel_z;
    //
    // if (acc_sq < 0.25f || acc_sq > 2.25f) {
    //     return false;
    // }

    return true;
}

static void ComputeEuler(AttitudeData *a)
{
    float q0_ = q0,q1_=q1,q2_=q2,q3_=q3;
    a->roll = atan2f(2*q0_*q1_+2*q2_*q3_,1-2*q1_*q1_-2*q2_*q2_)*57.29578f;
    a->roll = -a->roll;
    float sp = 2*(q0_*q2_-q3_*q1_);if(sp>1)sp=1;else if(sp<-1)sp=-1;
    a->pitch = asinf(sp)*57.29578f;
    a->yaw = atan2f(2*q0_*q3_+2*q1_*q2_,1-2*q2_*q2_-2*q3_*q3_)*57.29578f;
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

            if (!ControlOutputEnabledSnapshot())
            {
                //只有未解锁/未输出时才允许动态修gyro零偏，避免飞行中把真实动作学成bias
                ICM42688_UpdateBias(&raw_data);
            }

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

void Control_Task(void *pvParameters)
{
    (void)pvParameters;

    uint16_t ch[CRSF_NUM_CHANNELS];

    ControlRcInput_t rc;
    ControlSensorInput_t sensor;
    ControlOutput_t out;

    for (;;)
    {
        //读取当前CRSF通道，读出来的是原始值
        CRSF_GetChannels(ch);

        rc.roll_us = CRSF_MapRawToUs(ch[0]);
        rc.pitch_us = CRSF_MapRawToUs(ch[1]);
        rc.throttle_us = CRSF_MapRawToUs(ch[2]);
        rc.yaw_us = CRSF_MapRawToUs(ch[3]);

        rc.arm_switch = ch[4] > CRSF_CHANNEL_MID;
        rc.level_switch = ch[5] > CRSF_CHANNEL_MID;
        rc.failsafe = !CRSF_IsLinkUp();

        // rc.throttle_us = 1000;
        // rc.arm_switch = true;
        // rc.level_switch = true;
        // rc.failsafe = false;

        sensor.roll_deg = -att_data.roll;
        sensor.pitch_deg = att_data.pitch;
        sensor.yaw_deg = att_data.yaw;

        sensor.gyro_x_dps = sensor_data.gyro_x * 57.29587f;
        sensor.gyro_y_dps = sensor_data.gyro_y * 57.29587f;
        sensor.gyro_z_dps = sensor_data.gyro_z * 57.29587f;

        sensor.imu_valid = ImuSensorDataValid(&sensor_data);

        Control_Update(&rc,&sensor,0.001f,&out);
        Motor_SetControlOutput(&out);

        taskENTER_CRITICAL();
        control_output = out;
        taskEXIT_CRITICAL();

        vTaskDelay(1);
    }
}

void Baro_Task(void *pvParameters)
{
    (void)pvParameters;

    SPL06_Data_t data;
    float altitude;

    baro_status = SPL06_Init(&baro_dev,&hi2c2,SPL06_I2C_ADDR_HAL);

    if (baro_status == SPL06_OK)
    {
        baro_inited = 1U;
        baro_valid = 0U;
    }
    else
    {
        baro_inited = 0U;
        baro_valid = 0U;
    }

    for (;;)
    {
        if (baro_inited)
        {
            baro_status = SPL06_Read(&baro_dev,&data);

            if (baro_status == SPL06_OK)
            {
                //第一次读取成功时记录下参考气压
                if (baro_ref_pressure_pa <= 0.0f)
                {
                    baro_ref_pressure_pa = data.pressure_pa;
                }

                altitude = SPL06_PressureToAltitude(data.pressure_pa,baro_ref_pressure_pa);

                taskENTER_CRITICAL();

                baro_data = data;
                baro_altitude_m = altitude;
                baro_valid = 1U;

                taskEXIT_CRITICAL();
            }
            else
            {
                baro_valid = 0U;
            }
        }

        vTaskDelay(30);
    }
}

void USB_Task(void *pvParameters)
{
    (void)pvParameters;
    static uint32_t print_tick = 0U;

    ControlOutput_t ctrl;

    IMU_SensorData imu;        // 用来保存一份 IMU 数据快照
    AttitudeData att;          // 用来保存一份姿态角快照
    int gyro_z_dps;            // yaw角速度
    /*
     * USB CDC 初始化。
     *
     * 如果 main.c 里已经调用 MX_USB_DEVICE_Init()，这里理论上可以不再调用。
     * 但你原来就是这里也调了一次，先保留，后面再统一。
     */
    MX_USB_DEVICE_Init();

    uint16_t ch[CRSF_NUM_CHANNELS];
    uint16_t ch_us[8];
    uint8_t i;

    for (;;)
    {
        /*
         * 把日志ring里的数据真正发到USB CDC。
         * usb_log_printf()本身只是把字符串塞进ring，不直接发USB。
         */
        Betaflight_USB_Server();

        /*
         * 当前阶段只看 IMU。
         * 现在为了确认电机编号，重新打开 USB 命令处理。
         * 例如 m0 100 / m1 100 / stop。
         */
        UsbCommand_ProcessRx();

        print_tick++;

        CRSF_GetChannels(ch);

        for (i = 0U; i < 8U; i++)
        {
            ch_us[i] = CRSF_MapRawToUs(ch[i]);
        }

        if (print_tick >= 100U)
        {
            print_tick = 0U;

            taskENTER_CRITICAL();
            ctrl = control_output;

            // 拷贝当前 IMU 数据，避免打印时数据被 IMU_Task 更新
            imu = sensor_data;

            // 拷贝当前姿态角，方便同时看 yaw 角
            att = att_data;

            taskEXIT_CRITICAL();

            gyro_z_dps = (int)(imu.gyro_z * 57.2958f);


            usb_log_printf(
            "CTRL en:%u r:%d p:%d y:%d yt:%d ye:%d gz:%d ty:%d yp:%d m:%u %u %u %u",
            (unsigned)ctrl.enabled,
            (int)att.roll,
            (int)att.pitch,
            (int)att.yaw,
            (int)ctrl.target_yaw_deg,
            (int)ctrl.yaw_error_deg,
            gyro_z_dps,
            (int)ctrl.target_yaw_rate_dps,
            (int)ctrl.yaw_pid,
            (unsigned)ctrl.motor[0],
            (unsigned)ctrl.motor[1],
            (unsigned)ctrl.motor[2],
            (unsigned)ctrl.motor[3]
            );

            // usb_log_printf("RC raw:%u %u %u %u %u %u %u %u",
            //    (unsigned)ch[0],
            //    (unsigned)ch[1],
            //    (unsigned)ch[2],
            //    (unsigned)ch[3],
            //    (unsigned)ch[4],
            //    (unsigned)ch[5],
            //    (unsigned)ch[6],
            //    (unsigned)ch[7]);


        }
        /*
         * USB任务活着的指示灯。
         */
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3);

        /*
         * 让出CPU。
         */
        vTaskDelay(1);
    }
}

void Test_Task(void *pvParameters)
{
    for (;;)
    {
        HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_0);
        vTaskDelay(500);
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

void AppTaskInit(void)
{
    imu_drdy_queue_handle = xQueueCreateStatic(
        1,//队列长度
        sizeof(uint8_t),//每个队列元素大小
        imu_drdy_queue_storage,//队列数据存储区
        &imu_drdy_queue//队列控制块
        );
    CRSFTaskHandle = xTaskCreateStatic(
    CRSF_Task,
    "CRSFTask",
    CRSF_STACK_SIZE,
    NULL,
    CRSFStack,
    &CRSFTaskTCB
    );
    CRSFTaskTCB.uxPriority = 8;

    IMUTaskHandle = xTaskCreateStatic(
        IMU_Task,
        "IMUTask",
        IMU_STACK_SIZE,
        NULL,
        IMUStack,
        &IMUTaskTCB
    );
    IMUTaskTCB.uxPriority = 7;

    ControlTaskHandle = xTaskCreateStatic(
    Control_Task,
    "ControlTask",
    CONTROL_STACK_SIZE,
    NULL,
    ControlStack,
    &ControlTaskTCB
    );
    ControlTaskTCB.uxPriority = 6;

    DshotTaskHandle = xTaskCreateStatic(
    Dshot_Task,
    "DshotTask",
    DSHOT_STACK_SIZE,
    NULL,
    DshotStack,
    &DshotTaskTCB
    );
    DshotTaskTCB.uxPriority = 5;

    BaroTaskHandle = xTaskCreateStatic(
    Baro_Task,
    "BaroTask",
    BARO_STACK_SIZE,
    NULL,
    BaroStack,
    &BaroTaskTCB
    );
    BaroTaskTCB.uxPriority = 4;

    USBTaskHandle = xTaskCreateStatic(
    USB_Task,
    "USBTask",
    USB_STACK_SIZE,
    NULL,
    USBStack,
    &USBTaskTCB
    );
    USBTaskTCB.uxPriority = 3;

    TestTaskHandle = xTaskCreateStatic(
    Test_Task,
    "TestTask",
    TEST_STACK_SIZE,
    NULL,
    TestStack,
    &TestTaskTCB
    );
    TestTaskTCB.uxPriority = 1;

    vListInsertEnd(&(pxReadyTasksLists[8]), &(CRSFTaskTCB.xStateListItem));
    vListInsertEnd(&(pxReadyTasksLists[7]), &(IMUTaskTCB.xStateListItem));
    vListInsertEnd(&(pxReadyTasksLists[6]), &(ControlTaskTCB.xStateListItem));
    vListInsertEnd(&(pxReadyTasksLists[5]), &(DshotTaskTCB.xStateListItem));
    vListInsertEnd(&(pxReadyTasksLists[4]), &(BaroTaskTCB.xStateListItem));
    vListInsertEnd(&(pxReadyTasksLists[3]), &(USBTaskTCB.xStateListItem));
    vListInsertEnd(&(pxReadyTasksLists[1]), &(TestTaskTCB.xStateListItem));

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
