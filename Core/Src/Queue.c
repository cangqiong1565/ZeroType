//
// Created by zero on 2026/2/8.
//

#include "../Inc/Queue.h"
#include "../Inc/Task.h"

extern TCB_t *volatile pxCurrentTCB;
extern List_t pxReadyTasksLists[configMAX_PRIORITIES];

QueueHandle_t xQueueCreateStatic(
UBaseType_t uxQueueLength,// 深度
    UBaseType_t uxItemSize,//每个数据大小
    uint8_t *pucQueueStorageBuffer,//数组存哪里（数组地址）
    Queue_t *pxQueueBuffer //队列结构体存哪里
    )
{
    Queue_t *pxNewQueue;

    /* 1. 检查参数是否合法 */
    if( ( uxQueueLength > 0 ) &&
        ( pxQueueBuffer != NULL ) &&
        ( pucQueueStorageBuffer != NULL ) &&
        ( uxItemSize > 0 ) )
    {
        /* 指向用户提供的结构体内存 */
        pxNewQueue = pxQueueBuffer;

        /* 2. 初始化环形缓冲区的指针 */
        pxNewQueue->pcHead = ( int8_t * ) pucQueueStorageBuffer; // 头指向数组首地址
        pxNewQueue->uxItemSize = uxItemSize;
        pxNewQueue->uxLength = uxQueueLength;

        /* Tail 指向数组的末尾 */
        /* 比如：Head是0x1000, 长度10, 每个4字节。Tail = 0x1000 + 40 = 0x1028 */
        pxNewQueue->pcTail = pxNewQueue->pcHead + ( uxQueueLength * uxItemSize );

        /* 初始化状态 */
        pxNewQueue->uxMessagesWaiting = 0;
        pxNewQueue->pcWriteTo = pxNewQueue->pcHead;

        /* * 注意：FreeRTOS 的读取指针通常指向“上一个读取过的位置”。
         * 所以初始化时，让它指向队列的最后一个位置。
         * 这样第一次读取时，先 ++，就刚好回到 Head，读取第 0 个数据。
         */
        pxNewQueue->pcReadFrom = pxNewQueue->pcHead + ( ( uxQueueLength - 1 ) * uxItemSize );

        /* 3. 初始化阻塞列表 (现在还没实现具体功能，但先初始化防止野指针) */
        vListInitialise( &( pxNewQueue->xTasksWaitingToSend ) );
        vListInitialise( &( pxNewQueue->xTasksWaitingToReceive ) );

        return pxNewQueue;
    }
    else
    {
        return NULL;
    }
}

//入队函数（教学）
// BaseType_t xQueueGenericSend( QueueHandle_t xQueue,const void * const pvItemToQueue)
// {
//     Queue_t * const pxQueue = ( Queue_t * ) xQueue;
//
//     taskENTER_CRITICAL();
//     {
//         if (pxQueue->uxMessagesWaiting < pxQueue->uxLength)
//         {
//             /* 2. 【先操作】：把数据拷贝到 pcWriteTo 指向的位置 */
//             /* 提示：用 memcpy，目标地址是 pxQueue->pcWriteTo */
//             // memcpy( pxQueue->pcWriteTo, pvItemToQueue, pxQueue->uxItemSize );
//             /* 为了简单演示，我们假设是逐字节拷贝，或者你自己写 memcpy */
//             int8_t *pxDest = pxQueue->pcWriteTo;
//             int8_t *pxSrc = (int8_t *)pvItemToQueue;
//             for( UBaseType_t x = 0; x < pxQueue->uxItemSize; x++ )
//             {
//                 *pxDest = *pxSrc;
//                 pxDest++;
//                 pxSrc++;
//             }
//
//             //更新 Write 指针
//             pxQueue->pcWriteTo+= pxQueue->uxItemSize;
//
//             // 4. 处理回绕：如果超出了尾巴，就回到头
//             if( pxQueue->pcWriteTo >= pxQueue->pcTail )
//             {
//                 pxQueue->pcWriteTo = pxQueue->pcHead;
//             }
//
//             //5计数器+1
//             pxQueue->uxMessagesWaiting++;
//
//             taskEXIT_CRITICAL();
//
//             return pdPASS;
//
//         }
//     }
//
//     taskEXIT_CRITICAL();
//     return pdFAIL;
//
// }


/*
 *入队函数（升级）
 * 注意：增加了 xTicksToWait 参数
 *
 */
// BaseType_t xQueueGenericSend( QueueHandle_t xQueue, const void * const pvItemToQueue, TickType_t xTicksToWait )
// {
//     Queue_t * const pxQueue = ( Queue_t * ) xQueue;
//     BaseType_t xEntryTimeSet = pdFALSE;
//
//     for( ;; ) // 死循环，直到写入成功或者超时
//     {
//         taskENTER_CRITICAL();
//         {
//             /* --- 阶段 1: 检查队列还有没有空位 --- */
//             if( pxQueue->uxMessagesWaiting < pxQueue->uxLength )
//             {
//                 /* A. 有空位！开始干活 */
//                 int8_t *pxDest = pxQueue->pcWriteTo;
//                 int8_t *pxSrc = (int8_t *)pvItemToQueue;
//                 for( UBaseType_t x = 0; x < pxQueue->uxItemSize; x++ )
//                 {
//                     *pxDest = *pxSrc;
//                     pxDest++; pxSrc++;
//                 }
//
//                 /* 移动写指针 */
//                 pxQueue->pcWriteTo += pxQueue->uxItemSize;
//                 if( pxQueue->pcWriteTo >= pxQueue->pcTail )
//                 {
//                     pxQueue->pcWriteTo = pxQueue->pcHead;
//                 }
//
//                 pxQueue->uxMessagesWaiting++;
//
//                 /* B. 【新功能】写入了数据，说明队列不空了！ */
//                 /* 如果有任务在等数据 (Receive 阻塞)，把它叫醒 */
//                 if( listLIST_IS_EMPTY( &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
//                 {
//                     if( xTaskRemoveFromEventList( &( pxQueue->xTasksWaitingToReceive ) ) == pdTRUE )
//                     {
//                         /* 如果叫醒的任务优先级比我高，我应该立刻让出 CPU */
//                         /* 暂时先不管优先级抢占，这里可以触发一次调度 */
//                         portYIELD();
//                     }
//                 }
//
//                 taskEXIT_CRITICAL();
//                 return pdPASS; // 成功返回
//             }
//             else
//             {
//                 /* --- 阶段 2: 队列满了 --- */
//
//                 if( xTicksToWait == 0 )
//                 {
//                     /* 不愿意等，直接返回失败 */
//                     taskEXIT_CRITICAL();
//                     return pdFAIL;
//                 }
//                 else
//                 {
//                     /* C. 【新功能】愿意等！把自己挂到“发送等待列表” */
//                     vTaskPlaceOnEventList( &( pxQueue->xTasksWaitingToSend ), xTicksToWait );
//
//                     /* 此时任务已经不在就绪列表了，必须立刻切走！ */
//                     /* 这个 portYIELD 会导致 CPU 去跑别的任务 */
//                     portYIELD();
//
//                     /* D. 什么时候会运行到这行代码？ */
//                     /* 只有当别人读了数据，把你叫醒(Remove)后，你才会回到这里！ */
//                     /* 醒来后，for(;;) 循环会让你再次去检查 if( waiting < length ) */
//                 }
//             }
//         }
//         taskEXIT_CRITICAL();
//     }
// }

BaseType_t xQueueGenericSend( QueueHandle_t xQueue, const void * const pvItemToQueue, TickType_t xTicksToWait )
{
    Queue_t * const pxQueue = ( Queue_t * ) xQueue;

    for( ;; ) // 死循环，直到写入成功或者彻底超时
    {
        // 1. 【加锁】：保护队列结构体不被打断
        taskENTER_CRITICAL();

        /* --- 阶段 1: 检查队列还有没有空位 --- */
        if( pxQueue->uxMessagesWaiting < pxQueue->uxLength )
        {
            /* A. 有空位！开始干活 (数据拷贝) */
            int8_t *pxDest = pxQueue->pcWriteTo;
            int8_t *pxSrc = (int8_t *)pvItemToQueue;
            for( UBaseType_t x = 0; x < pxQueue->uxItemSize; x++ )
            {
                *pxDest = *pxSrc;
                pxDest++; pxSrc++;
            }

            /* 移动写指针并处理回绕 */
            pxQueue->pcWriteTo += pxQueue->uxItemSize;
            if( pxQueue->pcWriteTo >= pxQueue->pcTail )
            {
                pxQueue->pcWriteTo = pxQueue->pcHead;
            }

            pxQueue->uxMessagesWaiting++; // 队列里多了一个数据

            /* B. 唤醒在接收端排队的任务 */
            if( listLIST_IS_EMPTY( &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
            {
                if( xTaskRemoveFromEventList( &( pxQueue->xTasksWaitingToReceive ) ) == pdTRUE )
                {
                    /* 注意：我们还在临界区里，这里不能直接 yield！
                     * 严格的 RTOS 会用一个标志位记录需要切换，等退出临界区后再切。
                     * 这里为了你的系统能跑通，咱们先简单处理，退出临界区后再切！*/
                }
            }

            // 【解锁并凯旋】：干完活了，开中断，返回成功
            taskEXIT_CRITICAL();
            return pdPASS;
        }
        else
        {
            /* --- 阶段 2: 队列满了 --- */
            if( xTicksToWait == 0 )
            {
                // 【解锁并放弃】：不愿意等，开中断，直接返回失败
                taskEXIT_CRITICAL();
                return pdFAIL;
            }
            else
            {
                /* C. 愿意等！把自己挂到“发送等待列表” */
                vTaskPlaceOnEventList( &( pxQueue->xTasksWaitingToSend ), xTicksToWait );

                /* ========================================================== */
                /* 致命漏洞修复 1：睡觉前必须开门！                        */
                /* 否则 CPU 带着关中断的状态去跑别的任务，系统当场脑死亡      */
                /* ========================================================== */
                taskEXIT_CRITICAL();

                /* D. 交出 CPU 执行权，进入休眠 */
                portYIELD();

                /* ========================================================== */
                /* 任务从这里醒来！                                        */
                /* 致命漏洞修复 2：查明唤醒原因（超时还是拿到坑位？）      */
                /* ========================================================== */
                // 我们在 vTaskPlaceOnEventList 里设置了 pxCurrentTCB->xTicksToDelay
                // 每次 SysTick 滴答，xTaskIncrementTick 都会把它减 1
                // 如果减到了 0，说明是闹钟把我叫醒的！
                if( pxCurrentTCB->xEventWasSet == pdFALSE )
                {
                    return pdFAIL;
                }

                pxCurrentTCB->xEventWasSet = pdFALSE;
                // 如果走到这里，说明 xTicksToDelay > 0！
                // 意思是：闹钟还没响，是某个好心人调用了 xQueueReceive，从队列里拿走了一个数据，
                // 然后顺手把我从 wait_list 拔出来，塞回了就绪链表！
                // 太好了！此时 for(;;) 会循环回到最上面，重新 taskENTER_CRITICAL() 去抢这个刚空出来的坑位！
            }
        }
    }
}

/*
 * 中断级入队函数 (专供 USART_IRQHandler, EXTI_IRQHandler 等使用)
 * 参数3：pxHigherPriorityTaskWoken - 这是一个标志位的指针，用来带出“是否需要切换任务”的信号
 */
BaseType_t xQueueGenericSendFromISR( QueueHandle_t xQueue, const void * const pvItemToQueue, BaseType_t *pxHigherPriorityTaskWoken )
{
    Queue_t * const pxQueue = ( Queue_t * ) xQueue;
    BaseType_t xReturn;
    uint32_t ulSavedInterruptStatus;

    /* 1. 【安全第一】：带返回值的中断屏蔽
     * 为什么不用 taskENTER_CRITICAL？因为这是在中断里！必须防中断嵌套！
     * 这调用了你 portMacro.h 里的 ulPortRaiseBASEPRI()
     */
    ulSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();

    /* 2. 只有一次机会：绝不写 for(;;) 死循环 */
    if( pxQueue->uxMessagesWaiting < pxQueue->uxLength )
    {
        /* A. 有空位！火速拷贝数据 */
        int8_t *pxDest = pxQueue->pcWriteTo;
        int8_t *pxSrc = (int8_t *)pvItemToQueue;
        for( UBaseType_t x = 0; x < pxQueue->uxItemSize; x++ )
        {
            *pxDest++ = *pxSrc++;
        }

        /* 移动写指针并处理回绕 */
        pxQueue->pcWriteTo += pxQueue->uxItemSize;
        if( pxQueue->pcWriteTo >= pxQueue->pcTail ) pxQueue->pcWriteTo = pxQueue->pcHead;

        pxQueue->uxMessagesWaiting++;

        /* B. 看看有没有任务在等数据？(比如底盘 PID 解算任务) */
        if( listLIST_IS_EMPTY( &( pxQueue->xTasksWaitingToReceive ) ) == pdFALSE )
        {
            if( xTaskRemoveFromEventList( &( pxQueue->xTasksWaitingToReceive ) ) == pdTRUE )
            {
                /* ========================================================== */
                /* 🚨 核心区别：绝不在这里调用 portYIELD()！                  */
                /* 只是在标志位上做个记号：大哥醒了，等会儿记得切过去！       */
                /* ========================================================== */
                if( pxHigherPriorityTaskWoken != NULL )
                {
                    *pxHigherPriorityTaskWoken = pdTRUE;
                }
            }
        }
        xReturn = pdPASS;
    }
    else
    {
        /* C. 队列满了！没有任何废话，不排队不睡觉，直接丢弃数据并报错 */
        xReturn = pdFAIL;
    }

    /* 3. 恢复进入函数前的中断屏蔽状态 */
    portCLEAR_INTERRUPT_MASK_FROM_ISR( ulSavedInterruptStatus );

    return xReturn;
}

/* 简单的出队逻辑：把队列里的数据拷贝到 pvBuffer */
// BaseType_t xQueueGenericReceive( QueueHandle_t xQueue, void * const pvBuffer )
// {
//     Queue_t * const pxQueue = ( Queue_t * ) xQueue;
//
//     taskENTER_CRITICAL();
//     {
//         /* 1. 判断队列是不是空的？ */
//         if( pxQueue->uxMessagesWaiting > 0 )
//         {
//             /* 2. 【先移动】：更新 Read 指针 */
//             pxQueue->pcReadFrom += pxQueue->uxItemSize;
//
//             /* 处理回绕 */
//             if( pxQueue->pcReadFrom >= pxQueue->pcTail )
//             {
//                 pxQueue->pcReadFrom = pxQueue->pcHead;
//             }
//
//             /* 3. 【后操作】：读取数据 */
//             /* 把 pcReadFrom 指向的数据，拷贝到用户的 pvBuffer 里 */
//             int8_t *pxDest = (int8_t *)pvBuffer;
//             int8_t *pxSrc = pxQueue->pcReadFrom;
//             for( UBaseType_t x = 0; x < pxQueue->uxItemSize; x++ )
//             {
//                 *pxDest = *pxSrc;
//                 pxDest++;
//                 pxSrc++;
//             }
//
//             /* 4. 计数器 -1 */
//             pxQueue->uxMessagesWaiting--;
//
//             taskEXIT_CRITICAL();
//             return pdPASS;
//         }
//     }
//     taskEXIT_CRITICAL();
//
//     return pdFAIL; // 队列空的，读取失败
// }

// BaseType_t xQueueGenericReceive( QueueHandle_t xQueue, void * const pvBuffer, TickType_t xTicksToWait )
// {
//     Queue_t * const pxQueue = ( Queue_t * ) xQueue;
//
//     for( ;; )
//     {
//         taskENTER_CRITICAL();
//         {
//             /* 1. 检查队列是不是空的 */
//             if( pxQueue->uxMessagesWaiting > 0 )
//             {
//                 /* A. 有数据！开始读取 (跟昨天一样) */
//                 pxQueue->pcReadFrom += pxQueue->uxItemSize;
//                 if( pxQueue->pcReadFrom >= pxQueue->pcTail )
//                 {
//                     pxQueue->pcReadFrom = pxQueue->pcHead;
//                 }
//
//                 int8_t *pxDest = (int8_t *)pvBuffer;
//                 int8_t *pxSrc = pxQueue->pcReadFrom;
//                 for( UBaseType_t x = 0; x < pxQueue->uxItemSize; x++ )
//                 {
//                     *pxDest = *pxSrc;
//                     pxDest++; pxSrc++;
//                 }
//
//                 pxQueue->uxMessagesWaiting--;
//
//                 /* B. 【新功能】读走了数据，说明队列有空位了！ */
//                 /* 如果有任务在等空位 (Send 阻塞)，把它叫醒 */
//                 if( listLIST_IS_EMPTY( &( pxQueue->xTasksWaitingToSend ) ) == pdFALSE )
//                 {
//                     if( xTaskRemoveFromEventList( &( pxQueue->xTasksWaitingToSend ) ) == pdTRUE )
//                     {
//                         portYIELD();
//                     }
//                 }
//
//                 taskEXIT_CRITICAL();
//                 return pdPASS;
//             }
//             else
//             {
//                 /* 2. 队列空的 */
//                 if( xTicksToWait == 0 )
//                 {
//                     taskEXIT_CRITICAL();
//                     return pdFAIL;
//                 }
//                 else
//                 {
//                     /* C. 睡觉：挂到“接收等待列表” */
//                     vTaskPlaceOnEventList( &( pxQueue->xTasksWaitingToReceive ), xTicksToWait );
//                     portYIELD();
//
//                     /* D. 醒来后，循环继续，重新检查有没有数据 */
//                 }
//             }
//         }
//         taskEXIT_CRITICAL();
//     }
// }
BaseType_t xQueueGenericReceive( QueueHandle_t xQueue, void * const pvBuffer, TickType_t xTicksToWait )
{
    Queue_t * const pxQueue = ( Queue_t * ) xQueue;

    for( ;; )
    {
        taskENTER_CRITICAL(); // 保护队列不被中断打扰

        /* 1. 检查队列是不是空的 */
        if( pxQueue->uxMessagesWaiting > 0 )
        {
            /* A. 有数据！开始读取 */
            pxQueue->pcReadFrom += pxQueue->uxItemSize;
            if( pxQueue->pcReadFrom >= pxQueue->pcTail ) pxQueue->pcReadFrom = pxQueue->pcHead;

            int8_t *pxDest = (int8_t *)pvBuffer;
            int8_t *pxSrc = pxQueue->pcReadFrom;
            for( UBaseType_t x = 0; x < pxQueue->uxItemSize; x++ ) { *pxDest++ = *pxSrc++; }

            pxQueue->uxMessagesWaiting--;

            /* B. 唤醒在“发送大厅”排队的任务 */
            if( listLIST_IS_EMPTY( &( pxQueue->xTasksWaitingToSend ) ) == pdFALSE )
            {
                if( xTaskRemoveFromEventList( &( pxQueue->xTasksWaitingToSend ) ) == pdTRUE )
                {
                    // 同样，真正的切换等退出临界区后再做
                }
            }

            taskEXIT_CRITICAL(); // 干完活解锁
            return pdPASS;
        }
        else
        {
            /* 2. 队列空的 */
            if( xTicksToWait == 0 )
            {
                taskEXIT_CRITICAL(); // 不愿等，解锁走人
                return pdFAIL;
            }
            else
            {
                /* C. 愿意等：挂到“接收等待列表” */
                vTaskPlaceOnEventList( &( pxQueue->xTasksWaitingToReceive ), xTicksToWait );

                /* ========================================================== */
                /* 核心修复：睡觉前开门，交出 CPU                        */
                /* ========================================================== */
                taskEXIT_CRITICAL();
                portYIELD();

                /* D. 醒来后，查明唤醒原因 */
                if( pxCurrentTCB->xEventWasSet == pdFALSE )//如果不是事件唤醒
                {
                    // 被 SysTick 叫醒的，彻底超时了，接收失败
                    return pdFAIL;
                }

                //消费掉这次事件标志，避免影响下一次等待
                pxCurrentTCB->xEventWasSet = pdFALSE;

                // 如果没超时，说明有人刚发了数据把我叫醒！
                // 继续 for(;;) 循环，重新上锁去取数据！
            }
        }
    }
}
