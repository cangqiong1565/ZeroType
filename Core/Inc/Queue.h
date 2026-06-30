//
// Created by zero on 2026/2/8.
//

#ifndef CHASE_LIGHT_OS_QUEUE_H
#define CHASE_LIGHT_OS_QUEUE_H

#include "portMacro.h"
#include "List.h"

typedef void *QueueHandle_t;

typedef struct QueueDefinition
{
    int8_t *pcHead; //指向队列内存起始位置
    int8_t *pcTail; //指向队列内存结束位置

    int8_t *pcWriteTo; //指向下一个写入位置
    int8_t *pcReadFrom; //指向下一个读取位置

    UBaseType_t uxMessagesWaiting;//当前队列有几个消息
    UBaseType_t uxLength;//队列能装多少消息（深度）
    UBaseType_t uxItemSize;//每个消息有多大（字节数）

    //阻塞链表
    List_t xTasksWaitingToSend;  //发送阻塞链表，队列满了想发发不出去的排这里
    List_t xTasksWaitingToReceive;  //接收阻塞链表，队列满了想读读不到的排这里


} xQueue;

typedef xQueue Queue_t;

/* --- 函数声明 --- */
QueueHandle_t xQueueCreateStatic(
    UBaseType_t uxQueueLength,     // 深度
    UBaseType_t uxItemSize,        // 每个数据的大小
    uint8_t *pucQueueStorageBuffer,// 数据存哪里 (数组地址)
    Queue_t *pxQueueBuffer         // 队列结构体存哪里
);

BaseType_t xQueueGenericSend( QueueHandle_t xQueue, const void * const pvItemToQueue, TickType_t xTicksToWait );
BaseType_t xQueueGenericSendFromISR( QueueHandle_t xQueue, const void * const pvItemToQueue, BaseType_t *pxHigherPriorityTaskWoken );
BaseType_t xQueueGenericReceive( QueueHandle_t xQueue, void * const pvBuffer, TickType_t xTicksToWait );

#endif //CHASE_LIGHT_OS_QUEUE_H
