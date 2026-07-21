#ifndef CHASE_LIGHT_OS_TASK_H
#define CHASE_LIGHT_OS_TASK_H

#include "List.h"
#include "portMacro.h"
#include "Chase_Light_OS_Config.h"

#define taskYIELD() portYIELD()

#define taskSTACK_FILL_WORD ((StackType_t)0xA5A5A5A5UL)//新建任务时用这个值填满整块任务栈
#define taskSTACK_GUARD_WORDS 16U //栈底保留16个word作为保护区

#include "projdefs.h"
//任务控制块结构体定义
typedef struct tskTaskControlBlock
{
    volatile StackType_t * pxTopOfStack;        //栈顶

    ListItem_t xStateListItem;                  //任务节点
    ListItem_t xEventListItem;                  //事件节点

    UBaseType_t uxPriority;                     //当前优先级
    UBaseType_t uxBasePriority;                 //基础优先级

    StackType_t *pxStack;                       //任务栈起始地址
    StackType_t *pxEndOfStack;                  //栈地址边界，超过这里就意味着溢出

    uint32_t ulStackDepth;                      //栈深度

    char pcTaskName[configMAX_TASK_NAME_LEN];   //任务名称
    TickType_t xTicksToDelay;                   //延时节拍数

    BaseType_t xEventWasSet;                    //记录任务是不是被事件正常唤醒：pdTRUE=事件唤醒，pdFALSE=超时唤醒

}tskTCB;

typedef tskTCB TCB_t;
typedef void * TaskHandle_t;

void prvInitialiseTaskLists(void);
TaskHandle_t xTaskCreateStatic(
    void *pxTaskCode,                       //任务函数入口
    const char *const pcName,               //函数名
    const uint32_t ulStackDepth,            //栈深度
    void *const pvParameters,               //万能指针包裹，传参用
    StackType_t *const puxStackBuffer,      //uint32_t* 类型指针，栈地址
    TCB_t *const pxTaskBuffer               //任务控制块
);

void vTaskStartScheduler(void);
void xTaskIncrementTick(void);
void vTaskDelay(const TickType_t xTicksToDelay);
void vTaskPlaceOnEventList(List_t * const pxEventList , const TickType_t xTicksToWait);
BaseType_t xTaskRemoveFromEventList(const List_t * const pxEventList);

#define taskENTER_CRITICAL() portENTER_CRITICAL()  //进入临界段
#define taskENTER_CRITICAL_FROM_ISR() portSET_INTERRUPT_MASK_FROM_ISR() //用于中断函数的临界保护宏

#define taskEXIT_CRITICAL() portEXIT_CRITICAL()    //退出临界段
#define taskEXIT_CRITICAL_FROM_ISR(x) portCLEAR_INTERRUPT_MASK_FROM_ISR(x) //

#endif //CHASE_LIGHT_OS_TASK_H
