//
// Created by zero on 2026/2/23.
//

#include "../Inc/os_mutex.h"

extern TCB_t *volatile pxCurrentTCB;
extern List_t pxReadyTasksLists[configMAX_PRIORITIES];
//锁初始化
void Mutex_Init(Mutex_t *mutex)
{
    if (mutex == NULL)
    {
        return;
    }

    mutex->is_locked = 0;
    mutex->owner_task = NULL;
    mutex->original_priority = 0;


    vListInitialise(&mutex->wait_list);
}

//优先级设置函数
void vTaskPrioritySet(TCB_t *pxTask, UBaseType_t uxNewPriority)
{
    UBaseType_t uxCurrentPriority; //记录当前优先级
    TCB_t *pxTCB;//要被修改优先级的任务

    // 1. 确定我们要操作哪个 TCB
    // 如果传入 NULL，就默认操作当前正在跑的任务
    pxTCB = (pxTask == NULL) ? pxCurrentTCB : pxTask;
    //把任务优先级赋值给记录优先级的变量（做原始变量保护）
    uxCurrentPriority = pxTCB->uxPriority;
    //比较当前优先级和期望修改的优先级是否相同，也就是是否有修改的必要
    if (uxCurrentPriority != uxNewPriority) {
        // 2. 修改 TCB 里的优先级数值
        pxTCB->uxPriority = uxNewPriority;

        // 3. 关键动作：如果任务已经在就绪链表里，必须给它“挪个窝”
        // 因为 pxReadyTasksLists 是按优先级分坑位的！
        if (listLIST_ITEM_CONTAINER(&(pxTCB->xStateListItem)) != NULL) {
            // 先从旧优先级的链表里拔出来
            if (uxListRemove(&(pxTCB->xStateListItem)) == 0)
            {
                // 如果拔掉后，旧的链表空了，可能需要更新系统的最高优先级标志位（视你实现而定）
            }
            // 按照新优先级插进对应的链表
            vListInsertEnd(&(pxReadyTasksLists[uxNewPriority]), &(pxTCB->xStateListItem));
        }

        // 4. 抢占检查：如果新优先级比当前运行的任务还高，立刻触发切换
        if (uxNewPriority > pxCurrentTCB->uxPriority) {
            taskYIELD();
        }
    }
}

// 获取互斥锁函数
// BaseType_t xMutexTake(Mutex_t *pxMutex, TickType_t xTicksToWait)
// {
//     //一个任务想获取互斥锁一般会出现三种情况
//     // 1. 检查锁是否处于空闲状态 (is_locked == 0 表示没人用)
//     if (pxMutex->is_locked == 0)
//     {
//         // 【动作：成功抢到锁】
//         pxMutex->is_locked = 1;                  // 关上锁，拔下钥匙
//         pxMutex->owner_task = pxCurrentTCB;      // 登记造册：现在是我（当前任务）拿着锁
//
//         // 关键保护：记录我现在的真实优先级，防止以后别人乱改我的优先级后，我变不回来
//         pxMutex->original_priority = pxCurrentTCB->uxPriority;
//
//         return pdPASS; // 抢锁成功，直接返回去干活
//     }
//
//     // 2. 如果锁被占了，触发“优先级继承”机制，防止优先级翻转卡死系统
//     // 判断逻辑：我的优先级是不是比现在这个占着锁的任务高
//     if ( pxMutex->is_locked == 1 && pxCurrentTCB->uxPriority > pxMutex->owner_task->uxPriority)
//     {
//         // 【动作：临时提拔占锁人】
//         // 强行把占锁人的优先级，拔高到跟我一样高！
//         // （调用你刚注释好的那个函数，把它从低优先级就绪链表拔出，插到高优先级就绪链表里）
//         vTaskPrioritySet(pxMutex->owner_task, pxCurrentTCB->uxPriority);
//     }
//
//     // 3. 没抢到锁，只能去排队睡觉了
//     // 【动作：物理挪窝】
//     // 将当前任务的“状态手(xStateListItem)”从 pxReadyTasksLists 拔出
//     // 将当前任务的“事件手(xEventListItem)”插入到 pxMutex->wait_list (排队大厅)
//     // 顺便把 xTicksToDelay 设为 xTicksToWait，开启超时倒计时
//     vTaskPlaceOnEventList(&(pxMutex->wait_list), xTicksToWait);
//
//     // 4. 交出 CPU 执行权
//     // 因为我已经不在就绪链表里了，必须立刻叫调度器来换人跑
//     taskYIELD();
//
//     // 注意：当任务再次从这里醒来时，要么是等到了锁，要么是等超时了
//     // 严格的 RTOS 这里还需要判断醒来的原因，咱们目前先默认当做 pdPASS
//     return pdPASS;
// }

BaseType_t xMutexTake(Mutex_t *pxMutex, TickType_t xTicksToWait)
{
    // 【新加】：互斥锁的操作也必须防打断！
    taskENTER_CRITICAL();

    // 1. 空闲状态
    if (pxMutex->is_locked == 0)
    {
        pxMutex->is_locked = 1;
        pxMutex->owner_task = pxCurrentTCB;
        pxMutex->original_priority = pxCurrentTCB->uxPriority;

        taskEXIT_CRITICAL(); // 拿锁成功，开锁
        return pdPASS;
    }

    // 2. 占用状态 & 优先级继承
    if ( pxMutex->is_locked == 1 && pxCurrentTCB->uxPriority > pxMutex->owner_task->uxPriority)
    {
        vTaskPrioritySet(pxMutex->owner_task, pxCurrentTCB->uxPriority);
    }

    // 3. 阻塞排队
    vTaskPlaceOnEventList(&(pxMutex->wait_list), xTicksToWait);

    // 【关键修复】：睡觉前必须开全局中断！
    taskEXIT_CRITICAL();
    taskYIELD();

    // 4. 醒来查房
    if( pxCurrentTCB->xEventWasSet == pdFALSE )
    {
        return pdFAIL; // 闹钟响了，没等到锁
    }

    pxCurrentTCB->xEventWasSet = pdFALSE;
    // 因为咱们的 xMutexGive 采用了“连夜过户”的策略（直接把 owner 改成被叫醒的任务）
    // 所以只要不是超时醒来的，就说明 100% 已经拿到锁了！
    return pdPASS;
}

// 释放互斥锁函数
BaseType_t xMutexGive(Mutex_t *pxMutex)
{
    TCB_t *pxWokenTask; // 准备被叫醒的那个幸运儿

    // 1. 身份核验：防呆设计
    // 只有登记在册的“拿锁人”才有资格放锁，别人不能乱动我的锁
    if (pxMutex->owner_task != pxCurrentTCB)
    {
        return pdFAIL; // 不是你的锁，别瞎放
    }

    // 2. 优先级“还债”机制
    // 判断逻辑：我现在的优先级，还是不是我当初抢锁时的真实优先级？
    if (pxCurrentTCB->uxPriority != pxMutex->original_priority)
    {
        // 【动作：打回原形】
        // 说明我之前被某个高优先级大佬“临时提拔”过，现在活干完了，得降回原级
        // （再次调用你的函数，把我从高优先级就绪链表挪回低优先级链表）
        vTaskPrioritySet(pxCurrentTCB, pxMutex->original_priority);
    }

    // 3. 检查有没有人在等这把锁？
    // 判断逻辑：Mutex 的排队大厅 (wait_list) 是不是空的？
    if (listLIST_IS_EMPTY(&(pxMutex->wait_list)) == pdFALSE)
    {
        // 有人在排队！
        // 【动作：唤醒队首并移交钥匙】
        // 从 wait_list 拔出排在最前面的那个任务，并把它重新插回 pxReadyTasksLists (就绪链表)
        pxWokenTask = (TCB_t*) listGET_OWNER_OF_HEAD_ENTRY(&(pxMutex->wait_list));
        BaseType_t xNeedYield = xTaskRemoveFromEventList(&(pxMutex->wait_list));

        // 连夜把钥匙过户给这个刚醒的任务，不需要把 is_locked 置 0！
        pxMutex->owner_task = pxWokenTask;

        // 更新新主人的原始优先级记录
        pxMutex->original_priority = pxWokenTask->uxPriority;

        // 4. 抢占检查 (直接用 xTaskRemoveFromEventList 的返回值判断！)
        if (xNeedYield == pdTRUE)
        {
            // 我必须立刻让出 CPU，让刚刚拿锁的大佬先跑
            taskYIELD();
        }
    }
    else
    {
        // 没人在排队！
        // 【动作：彻底释放】
        pxMutex->is_locked = 0;       // 锁状态清零
        pxMutex->owner_task = NULL;   // 销毁拿锁人登记册
    }

    return pdPASS; // 顺利放锁
}