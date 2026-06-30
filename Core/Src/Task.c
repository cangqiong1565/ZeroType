#include"Task.h"
#include "port.h"
#include "stdint.h"

//0xe000ed20所对应的32 位的寄存器，专门用来存放 SysTick（滴答定时器） 和 PendSV（可挂起系统调用） 的优先级。
#define portNVIC_SYSPRI2_REG (*(( volatile uint32_t *) 0xe000ed20))

/*SHPR3是 Cortex-M 内核的“高层干部通讯录”。
* 普通的外部中断（比如串口、定时器、ADC）的优先级，由 NVIC_IPRx 寄存器管理。
* 内核自带的系统异常（比如 SysTick、PendSV、SVC、Fault），由 SHPRx 系列寄存器管理。
* SHPR3 专门管理编号为 12~15 的系统异常优先级。
*/
//为了把中断值搬进SHPR3寄存器的PendSV位置，PendSV处在寄存器的16-23位
#define portNVIC_PENDSV_PRI (((uint32_t) configKERNEL_INTERRUPT_PRIORITY ) << 16UL)

//SHPR3寄存器里，SysTick处于31-24位
#define portNVIC_SYSTICK_PRI (((uint32_t) configKERNEL_INTERRUPT_PRIORITY ) << 24UL )

//定义就绪链表
List_t pxReadyTasksLists[configMAX_PRIORITIES];
List_t xDelayedTaskList;

TCB_t *volatile pxCurrentTCB;
extern TCB_t PIDTaskTCB;
extern TCB_t IMUTaskTCB;
TCB_t IdleTaskTCB;

static TaskHandle_t xIdleTaskHandle;
TickType_t xTickCount;


/* 3. 定义空闲任务的 栈数组 (全局变量) */
StackType_t IdleTaskStack[ configMINIMAL_STACK_SIZE ];

static void prvInitialiseNewTask
(
    void (*pxTaskCode)(void *), //任务函数入口
    const char *const pcName, //任务函数名字
    const uint32_t ulStackDepth, //栈深度
    void *const pvParameters, //万能指针包裹，传参用
    void **const pxCreatedTask, //执向不同任务地址的二级指针
    struct tskTaskControlBlock *pxNewTCB //任务控制块
)
{
    StackType_t *pxTopOfStack;
    //uint32_t类型的指针辅助变量，用来存储栈顶指针
    UBaseType_t x;
    //unsigned long 类型的辅助变量和for循环里的i一个作用

    //获取栈顶地址，因为 ARM 的栈是“向下生长”的（高地址 -> 低地址），所以初始的栈顶指针必须指向数组的【最后/最高】那个位置
    pxTopOfStack = pxNewTCB->pxStack + (ulStackDepth - (uint32_t) 1);

    //强制8字节对齐，ARM 架构（特别是涉及浮点运算时）要求栈指针必须是 8 字节对齐的。
    pxTopOfStack = (uint32_t *) ((uint32_t) pxTopOfStack & (~((uint32_t) 0x0007)));

    //循环拷贝名字，简单地把字符串从参数 pcName 搬运到 TCB 的数组里。
    for (x = (UBaseType_t) 0; x < (UBaseType_t) configMAX_TASK_NAME_LEN; x++)
    {
        pxNewTCB->pcTaskName[x] = pcName[x];
        if (pcName[x] == 0x00)
        {
            break;
        }
    }

    //任务名字后加'\0'任务过长的时候手动结尾
    pxNewTCB->pcTaskName[configMAX_TASK_NAME_LEN - 1] = '\0';

    //初始化TCB中的xStateListItem节点，把 TCB 里自带的那个链表节点擦干净（pvContainer设为NULL），准备挂到就绪列表里去。
    vListInitialiseItem(&(pxNewTCB->xStateListItem));
    //初始化任务的事件链表节点用于队列/信号量等待链表
    vListInitialiseItem(&(pxNewTCB->xEventListItem));

    //为xStateListItem分配拥有者，这样以后调度器从链表里拿到这个节点时，通过 pxOwner 指针就能反向找到这个 TCB。
    listSET_LIST_ITEM_OWNER(&(pxNewTCB->xStateListItem), pxNewTCB);
    //事件节点的 owner 也指向当前 TCB，事件唤醒时能找到任务
    listSET_LIST_ITEM_OWNER(&(pxNewTCB->xEventListItem), pxNewTCB);

    //新任务初始不处于延时状态
    pxNewTCB->xTicksToDelay = 0;

    //新任务初始没有被任何事件唤醒
    pxNewTCB->xEventWasSet = pdFALSE;

    //初始化任务栈
    pxNewTCB->pxTopOfStack = pxPortInitialiseStack(pxTopOfStack, pxTaskCode, pvParameters);

    //让任务句柄存储TCB的地址
    if ((void *) pxCreatedTask != NULL)
    {
        *pxCreatedTask = (TaskHandle_t) pxNewTCB;
    }
}

#if(configSUPPORT_STATIC_ALLOCATION == 1)
//void*类型，用于联系任务控制块
TaskHandle_t xTaskCreateStatic(
    void *pxTaskCode, //任务函数入口
    const char *const pcName, //函数名
    const uint32_t ulStackDepth, //栈深度
    void *const pvParameters, //万能指针包裹，传参用
    StackType_t *const puxStackBuffer, //uint32_t* 类型指针，栈地址
    TCB_t *const pxTaskBuffer //任务控制块
)
{
    TCB_t *pxNewTCB;
    void *xReturn;

    if ((pxTaskBuffer != NULL) && (puxStackBuffer != NULL))
    {
        pxNewTCB = (TCB_t *) pxTaskBuffer;
        pxNewTCB->pxStack = (StackType_t *) puxStackBuffer;
        //uint32_t

        prvInitialiseNewTask(pxTaskCode, pcName, ulStackDepth, pvParameters, &xReturn, pxNewTCB);
    } else
    {
        xReturn = NULL;
    }
    return xReturn;
}
#endif

//就绪列表初始化
void prvInitialiseTaskLists(void)
{
    //unsigned long
    UBaseType_t uxPriority;

    for (uxPriority = (UBaseType_t) 0U; uxPriority < (UBaseType_t) configMAX_PRIORITIES; uxPriority++)
    {
        vListInitialise(&(pxReadyTasksLists[uxPriority]));
    }

    vListInitialise(&xDelayedTaskList);
}

BaseType_t xPortStartScheduler(void)
{
    portNVIC_SYSPRI2_REG |= portNVIC_PENDSV_PRI;
    portNVIC_SYSPRI2_REG |= portNVIC_SYSTICK_PRI;
    /*经过这样的操作以后SHPR3 寄存器里的值变成了 0xFFxxxxxx（假设保留位不变）。
    * PendSV 的优先级被设为了 240（最低）。
    * SysTick 的优先级被设为了 240（最低）
    * 确保 RTOS 的内核任务切换（PendSV）和 心跳时钟（SysTick）永远不会打断用户的外部硬件中断（如串口等）。只有当 CPU 空闲，没有外部紧急事件时，RTOS 才会切换任务
    */
    vPortSetupTimerInterrupt();
    /* 启动第一个任务，不再返回 */
    prvStartFirstTask();

    /* 不应该运行到这里 */
    return 0;
}

//调度器启动（教学1）
// void vTaskStartScheduler(void)
// {
//
//     pxCurrentTCB = &Task1TCB;
//
//     if ( xPortStartScheduler() != pdFALSE )
//     {
//         /* 调度器启动成功，则不会返回，即不会来到这里 */
//     }
// }

#if 0
//手动任务切换任务（教学版）
 void vTaskSwitchContext(void)
 {
     /* 两个任务轮流切换 */
     if (pxCurrentTCB == &Task1TCB)
     {
         pxCurrentTCB = &Task2TCB;
     } else
     {
         pxCurrentTCB = &Task1TCB;
     }
 }
#else

//手动切换任务教学（2）
// void vTaskSwitchContext(void)
// {
//     if (pxCurrentTCB == &IdleTaskTCB)
//     {
//         if (Task1TCB.xTicksToDelay == 0)
//         {
//             pxCurrentTCB = &Task1TCB;
//         }
//         else if (Task2TCB.xTicksToDelay == 0)
//         {
//             pxCurrentTCB = &Task2TCB;
//         }
//         else
//         {
//             return; //任务延时均没有到则返回
//         }
//     }
//     else
//     {
//         if (pxCurrentTCB == &Task1TCB)
//         {
//             if (Task2TCB.xTicksToDelay == 0)
//             {
//                 pxCurrentTCB = &Task2TCB;
//             }
//             else if (pxCurrentTCB->xTicksToDelay != 0)
//             {
//                 pxCurrentTCB = &IdleTaskTCB;
//             }
//             else
//             {
//                 return;
//             }
//         }
//         else if (pxCurrentTCB == &Task2TCB)
//         {
//             if (Task1TCB.xTicksToDelay == 0)
//             {
//                 pxCurrentTCB = &Task1TCB;
//             }
//             else if (pxCurrentTCB->xTicksToDelay != 0)
//             {
//                 pxCurrentTCB = &IdleTaskTCB;
//             }
//             else
//             {
//                 return;
//             }
//         }
//     }
// }
/* 放在 Task.c 中 */
// void vTaskSwitchContext( void )
// {
//     UBaseType_t uxTopPriority = configMAX_PRIORITIES - 1;
//
//     /* 1. 寻找最高优先级 */
//     /* 从最高优先级开始往下找，直到找到一个“不为空”的列表 */
//     while( listLIST_IS_EMPTY( &( pxReadyTasksLists[ uxTopPriority ] ) ) )
//     {
//         /* 避免死循环：如果所有列表都空了（理论上不可能，因为有空闲任务），就停在 0 */
//         if( uxTopPriority == 0 )
//         {
//             break;
//         }
//         --uxTopPriority;
//     }
//
//     /* 2. 获取该优先级列表中的下一个任务 (实现了同优先级轮转) */
//     listGET_OWNER_OF_NEXT_ENTRY( pxCurrentTCB, &( pxReadyTasksLists[ uxTopPriority ] ) );
// }

/* 放在 Task.c 中 */
// void vTaskSwitchContext( void )
// {
//     UBaseType_t uxPriority;
//
//     /* 从最高优先级开始往下找 */
//     for( uxPriority = configMAX_PRIORITIES - 1; uxPriority > 0; uxPriority-- )
//     {
//         /* 1. 先看列表是不是空的 */
//         if( listLIST_IS_EMPTY( &( pxReadyTasksLists[ uxPriority ] ) ) == pdFALSE )
//         {
//             /* 2. 【关键补丁】获取该列表头部的任务 TCB */
//             TCB_t *pxTCB = (TCB_t*)listGET_OWNER_OF_HEAD_ENTRY((&pxReadyTasksLists[uxPriority]));
//
//             /* 3. 【关键补丁】虽然你在列表里，但我得检查你是不是在睡觉！ */
//             if( pxTCB->xTicksToDelay == 0 )
//             {
//                 /* 找到了！优先级最高，且没在睡觉的任务 */
//                 pxCurrentTCB = pxTCB;
//                 return; /* 直接结束调度 */
//             }
//         }
//     }
//
//     /* 如果找了一圈，高优先级的都在睡觉，或者是空的，那就跑空闲任务 */
//     pxCurrentTCB = &IdleTaskTCB;
// }

// void vTaskSwitchContext( void )
// {
//     UBaseType_t uxPriority;
//
//     // 从最高优先级往 0 找 (包含 0，因为 0 是空闲任务)
//     for( uxPriority = configMAX_PRIORITIES - 1; uxPriority >= 0; uxPriority-- )
//     {
//         if( listLIST_IS_EMPTY( &( pxReadyTasksLists[ uxPriority ] ) ) == pdFALSE )
//         {
//             // 获取该优先级下的“下一个”任务，实现同优先级轮转
//             listGET_OWNER_OF_NEXT_ENTRY( pxCurrentTCB, &( pxReadyTasksLists[ uxPriority ] ) );
//
//             // 只有当任务不在延时状态时，才真正切换
//             if( pxCurrentTCB->xTicksToDelay == 0 )
//             {
//                 return;
//             }
//         }
//
//         if(uxPriority == 0) break; // 防止死循环
//     }
// }
// void vTaskSwitchContext(void)
// {
//     // 从最高优先级 4 开始向下寻找
//     int uxPriority = configMAX_PRIORITIES - 1;
//
//     do
//     {
//         if (listLIST_IS_EMPTY(&(pxReadyTasksLists[uxPriority])) == pdFALSE)
//         {
//             TCB_t *pxFirstTCB;
//             pxFirstTCB = (TCB_t*)listGET_OWNER_OF_HEAD_ENTRY((&pxReadyTasksLists[uxPriority]));
//
//             // 只有当任务没在睡觉（延时为0）时，才允许切过去跑
//             if (pxFirstTCB->xTicksToDelay == 0)
//             {
//                 pxCurrentTCB = pxFirstTCB;
//                 return; // 找到了，完美退出！
//             }
//         }
//         uxPriority--;
//     } while (uxPriority >= 0); // 注意：包含 0 号优先级
//
//     // =========================================================
//     // 【终极防御】：如果走到这里，说明所有用户任务都在无耻地睡觉！
//     // 强行把当前任务指向你的 PIDTaskTCB（或者你未来创建的 IdleTask），绝对不能让指针悬空！
//     // =========================================================
//     pxCurrentTCB = &IdleTaskTCB;
// }

void vTaskSwitchContext(void)
{
    int uxPriority = configMAX_PRIORITIES - 1;

    do
    {
        /* 1. 先看看这个优先级有没有任务 */
        if (listLIST_IS_EMPTY(&(pxReadyTasksLists[uxPriority])) == pdFALSE)
        {
            /* 2. 【核心修复】：使用你原本正确的宏，不仅获取任务，还要把指针向后挪动！
                  这保证了如果有两个同优先级的任务，它们能轮流执行，不会死机 */
            listGET_OWNER_OF_NEXT_ENTRY(pxCurrentTCB, &(pxReadyTasksLists[uxPriority]));
            return;
        }
        uxPriority--;
    } while (uxPriority >= 0);

    /* 终极防御 */
    pxCurrentTCB = &IdleTaskTCB;
}

#endif


//获取空闲任务的内存，任务栈，TCB
void vApplicationGetIdleTaskMemory(TCB_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &IdleTaskTCB;
    *ppxIdleTaskStackBuffer = IdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}


static void prvIdleTask(void *pvParameters)
{
    (void) pvParameters;

    for (;;)
    {

    }
}

// void vTaskDelay(const TickType_t xTicksToDelay)
// {
//     TCB_t *pxTCB = NULL;
//
//     pxTCB = pxCurrentTCB;
//     pxTCB->xTicksToDelay = xTicksToDelay;
//
//     taskYIELD();
// }

void vTaskDelay(const TickType_t xTicksToDelay)
{
    TickType_t xTimeToWake;
    TCB_t *pxTCB = pxCurrentTCB;

    if (xTicksToDelay > 0)
    {
        /* 1. 进入临界区，保护链表操作 */
        vPortEnterCritical();

        /* 2. 计算绝对醒来的时间戳 */
        xTimeToWake = xTickCount + xTicksToDelay;

        /* 3. 把这个绝对时间赋予链表项的辅助排序值 */
        pxTCB->xStateListItem.xItemValue = xTimeToWake;

        /* 4. 从就绪链表中彻底拔除 */
        uxListRemove(&(pxTCB->xStateListItem));

        /* 5. 按照升序规则，插入到延时链表中（最早醒来的在最前面） */
        vListInsert(&xDelayedTaskList, &(pxTCB->xStateListItem));

        /* 6. 退出临界区 */
        vPortExitCritical();

        /* 7. 触发 PendSV 异常，把 CPU 让给别的就绪任务 */
        taskYIELD();
    }
}


//调度器启动（教学2）
void vTaskStartScheduler(void)
{
    TCB_t *pxIdleTaskTCBBuffer = NULL;              //指向空闲任务的TCB
    StackType_t *pxIdleTaskStackBuffer = NULL;      //指向空闲任务的起始栈地址
    uint32_t ulIdleTaskStackSize;                   //

    vApplicationGetIdleTaskMemory(&pxIdleTaskTCBBuffer, &pxIdleTaskStackBuffer, &ulIdleTaskStackSize);

    xIdleTaskHandle = xTaskCreateStatic(prvIdleTask,
                                        (char *) "IDLE",
                                        (uint32_t) ulIdleTaskStackSize,
                                        (void *) NULL,
                                        (StackType_t *) pxIdleTaskStackBuffer,
                                        (TCB_t *) pxIdleTaskTCBBuffer);

    vListInsertEnd(&(pxReadyTasksLists[0]), &(((TCB_t *) pxIdleTaskTCBBuffer)->xStateListItem));

    pxCurrentTCB = &IMUTaskTCB;
    if (xPortStartScheduler() != pdFALSE)
    {
        /* 调度器启动成功，则不会返回，即不会来到这里 */
    }
}

// void xTaskIncrementTick(void)
// {
//     TCB_t *pxTCB = NULL;
//     BaseType_t i = 0;
//
//     const TickType_t xConstTickCount = xTickCount + 1;
//     xTickCount = xConstTickCount;
//
//     for (i = 0 ; i<configMAX_PRIORITIES ; i++)
//     {
//         pxTCB = (TCB_t*)listGET_OWNER_OF_HEAD_ENTRY((&pxReadyTasksLists[i]));
//         if (pxTCB->xTicksToDelay > 0)
//         {
//             pxTCB->xTicksToDelay--;
//         }
//     }
//
//     portYIELD();
// }

// void xTaskIncrementTick(void)
// {
//     TCB_t *pxTCB = NULL;
//     BaseType_t i = 0;
//
//     const TickType_t xConstTickCount = xTickCount + 1;
//     xTickCount = xConstTickCount;
//
//     /* 扫描所有优先级列表 */
//     for (i = 0 ; i < configMAX_PRIORITIES ; i++)
//     {
//         /* === 关键修复：先检查链表是不是空的！=== */
//         if (listLIST_IS_EMPTY(&pxReadyTasksLists[i]) == pdFALSE)
//         {
//             /* 只有列表不为空，才去取任务 */
//             pxTCB = (TCB_t*)listGET_OWNER_OF_HEAD_ENTRY((&pxReadyTasksLists[i]));
//
//             if (pxTCB->xTicksToDelay > 0)
//             {
//                 pxTCB->xTicksToDelay--;
//             }
//         }
//     }
//
//     portYIELD();
// }

void xTaskIncrementTick(void)
{
    TCB_t *pxTCB = NULL;
    TickType_t xItemValue;

    /* 1. 系统绝对时钟基准自增 */
    const TickType_t xConstTickCount = xTickCount + 1;
    xTickCount = xConstTickCount;

    /* 2. 高效检查延时链表 */
    /* 只要延时链表不为空，就一直检查（防止同一时刻有多个任务同时醒来） */
    while (listLIST_IS_EMPTY(&xDelayedTaskList) == pdFALSE)
    {
        /* 获取延时链表最前面的那个幸运儿 */
        pxTCB = (TCB_t *) listGET_OWNER_OF_HEAD_ENTRY(&xDelayedTaskList);

        /* 拿到它计划醒来的绝对时间 */
        xItemValue = listGET_LIST_ITEM_VALUE(&(pxTCB->xStateListItem));

        /* 判断时间到了没有？ */
        if (xTickCount >= xItemValue)
        {
            /* 时间到了！把它从延时链表里捞出来 */
            uxListRemove(&(pxTCB->xStateListItem));

            if (listLIST_ITEM_CONTAINER(&(pxTCB->xEventListItem)) != NULL)
            {
                uxListRemove(&(pxTCB->xEventListItem));// 如果这个任务还挂在事件等待链表里，把它摘掉
                pxTCB->xEventWasSet = pdFALSE; //标记不是事件唤醒，是超时唤醒
            }

            pxTCB->xTicksToDelay = 0;//清除等待时间，表示超时等待结束
            /* 完璧归赵：塞回它原本所属优先级的就绪链表 */
            vListInsertEnd(&(pxReadyTasksLists[pxTCB->uxPriority]), &(pxTCB->xStateListItem));//把任务重新放回就绪链表
        }
        else
        {
            /* 重点：如果排第一的大佬都没到点，后面的任务肯定也没到点，直接打破循环！ */
            break;
        }
    }

    /* 3. 触发一次任务调度检查 */
    portYIELD();
}



/* 参数1: 你要去哪里排队 (比如队列的“发送等待列表”)
 * 参数2: 你愿意等多久 (超时时间)
 * 作用：把任务挂起
 * 教学版本
 */
// void vTaskPlaceOnEventList(List_t * const pxEventList , const TickType_t xTicksToWait)
// {
//     //设置等待时间
//     pxCurrentTCB->xTicksToDelay = xTicksToWait;
//
//     /* 2. 【关键】把它从“就绪列表”里移除！ */
//     /* 只要不在就绪列表里，调度器就永远找不到它，它就真的“睡死”了 */
//     uxListRemove(&(pxCurrentTCB->xStateListItem));
//
//     //挂载到指定的连表上（如阻塞链表）
//     vListInsertEnd(pxEventList,&( pxCurrentTCB ->xStateListItem));
//
// }

/* 参数1: 你要去哪里排队 (比如队列的“发送等待列表”)
 * 参数2: 你愿意等多久 (超时时间)
 * 作用：把任务挂起
 */
void vTaskPlaceOnEventList(List_t * const pxEventList, const TickType_t xTicksToWait)
{
    TickType_t xTimeToWake; // 保存任务应该在未来哪个 tick 超时醒来

    pxCurrentTCB->xEventWasSet = pdFALSE; //刚开始等待事件，默认还没有被事件唤醒
    pxCurrentTCB->xTicksToDelay = xTicksToWait;//记录这次最多愿意等多少tick

    uxListRemove(&(pxCurrentTCB->xStateListItem));//把当前任务的事件节点挂到队列等待链表里

    vListInsertEnd(pxEventList, &(pxCurrentTCB->xEventListItem));//把当前任务的事件节点挂到队列等待链表里

    if (xTicksToWait != portMAX_DELAY) // 如果不是无限等待，就需要设置超时
    {
        xTimeToWake = xTickCount + xTicksToWait;// 计算绝对唤醒时间：当前 tick + 等待 tick 数
        pxCurrentTCB->xStateListItem.xItemValue = xTimeToWake;//把绝对唤醒时间写进状态节点，用于延时链表排序
        vListInsert(&xDelayedTaskList, &(pxCurrentTCB->xStateListItem));// 把状态节点插入延时链表，到时间后由 SysTick 唤醒
    }
}

/* 作用：把任务唤醒
 */
// BaseType_t xTaskRemoveFromEventList(const List_t * const pxEventList)
// {
//     TCB_t *pxUnblockedTCB;
//     BaseType_t xReturn;
//
//     //判断链表是否为空
//     if (listLIST_IS_EMPTY( pxEventList ) == pdFALSE)
//     {
//         //获取链表里的第一个任务
//         pxUnblockedTCB = (TCB_t*) listGET_OWNER_OF_HEAD_ENTRY(pxEventList);
//
//         //从等待链表里移除
//         uxListRemove( &( pxUnblockedTCB->xStateListItem ) );
//
//         //任务已经醒了，清除延时
//         pxUnblockedTCB ->xTicksToDelay = 0;
//
//         // 拔出来后，老老实实回到自己的优先级链表里去！
//         vListInsertEnd(&(pxReadyTasksLists[pxUnblockedTCB->uxPriority]), &(pxUnblockedTCB->xStateListItem));
//
//         xReturn = pdTRUE;
//     }
//     else
//     {
//         xReturn = pdFALSE;
//     }
//     return xReturn;
// }
BaseType_t xTaskRemoveFromEventList(const List_t * const pxEventList)
{
    TCB_t *pxUnblockedTCB;                               // 保存即将被唤醒的任务 TCB 指针

    BaseType_t xReturn;                                  // 保存函数返回值，表示有没有成功唤醒任务

    if (listLIST_IS_EMPTY(pxEventList) == pdFALSE)       // 如果事件等待链表不是空的，说明有任务在等事件
    {
        pxUnblockedTCB = (TCB_t *)listGET_OWNER_OF_HEAD_ENTRY(pxEventList);
        // 取出等待链表头部任务的 TCB

        uxListRemove(&(pxUnblockedTCB->xEventListItem)); // 把任务从事件等待链表里摘掉

        if (listLIST_ITEM_CONTAINER(&(pxUnblockedTCB->xStateListItem)) != NULL)
            // 如果任务的状态节点还挂在某个链表里
        {
            uxListRemove(&(pxUnblockedTCB->xStateListItem));
            // 把它从延时链表里摘掉，避免之后又被超时唤醒一次
        }

        pxUnblockedTCB->xTicksToDelay = 0;               // 清掉等待时间，表示它已经不再延时等待

        pxUnblockedTCB->xEventWasSet = pdTRUE;           // 标记：这次是事件正常唤醒，不是超时

        vListInsertEnd(&(pxReadyTasksLists[pxUnblockedTCB->uxPriority]),
                       &(pxUnblockedTCB->xStateListItem));
        // 把任务重新放回对应优先级的就绪链表

        xReturn = pdTRUE;                                // 返回成功，表示确实唤醒了一个任务
    }
    else
    {
        xReturn = pdFALSE;                               // 等待链表为空，没有任务可唤醒
    }

    return xReturn;                                      // 返回唤醒结果
}