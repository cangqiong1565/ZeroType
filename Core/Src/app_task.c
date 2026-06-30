#include "main.h"
#include "../Inc/app_task.h"
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
#include "main.h"
#include "Queue.h"


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

            MahonyAHRS_update(sensor_data.gyro_x,
                              sensor_data.gyro_y,
                              sensor_data.gyro_z,
                              sensor_data.accel_x,
                              sensor_data.accel_y,
                              sensor_data.accel_z,
                              0.001f);

            /* 后面接欧拉角计算 / PID */
        }
    }
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

void USB_Task(void *pvParameters)
{
    for (;;)
    {
        Betaflight_USB_Server();
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
