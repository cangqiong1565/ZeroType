//
// Created by zero on 2026/2/23.
//

#ifndef CHASE_LIGHT_OS_OS_MUTEX_H
#define CHASE_LIGHT_OS_OS_MUTEX_H

#include <stdint.h>
#include <stdbool.h>

#include "Task.h"
#include "List.h"

typedef struct
{
    uint8_t is_locked;              //0空闲，1占用
    TCB_t* owner_task;              //指向当前持有锁的任务的TCB
    List_t wait_list;               //等待这把锁的任务链表
    uint8_t original_priority;      //用于解决优先级翻转的备份
}Mutex_t;

void Mutex_Init(Mutex_t *mutex);

BaseType_t xMutexTake(Mutex_t *pxMutex, TickType_t xTicksToWait);

BaseType_t xMutexGive(Mutex_t *pxMutex);

#endif //CHASE_LIGHT_OS_OS_MUTEX_H