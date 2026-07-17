#ifndef CHASE_LIGHT_OS_CHASE_LIGHT_OS_CONFIG_H
#define CHASE_LIGHT_OS_CHASE_LIGHT_OS_CONFIG_H

#include <stdio.h>
#define configUSE_16_BIT_TICKS 0            //是否是32位单片机
#define configMAX_TASK_NAME_LEN 16          //最长任务名
#define configSUPPORT_STATIC_ALLOCATION 1   //是否支持
#define configMAX_PRIORITIES 5              //最长链表

#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS __NVIC_PRIO_BITS
#else
#define configPRIO_BITS 4
#endif

//中断最低优先级
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15

//系统可管理的最高中断优先级
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

//配置内核中断优先级
//Cortex-M 的优先级寄存器是 8 位 的。 但是 STM32 只用了 高 4 位，例如当优先级为15（1111）时，写入NVIC的必须是1111 0000也就是15左移4位，为240
#define configKERNEL_INTERRUPT_PRIORITY ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 -configPRIO_BITS))/* 240 */
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler
#define vPortSVCHandler SVC_Handler

#define configMAX_SYSCALL_INTERRUPT_PRIORITY ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
//左移四位，因为单片机只用最高四位

#define vAssertCalled(char,int) printf("Error:%s,%d\r\n",char,int);
//#define configASSERT(x) if((x)==0) vAssertCalled(__FILE__,__LINE__)
#define configASSERT( x ) if( ( x ) == 0 ) { portDISABLE_INTERRUPTS(); for( ;; ); }

/* 1. 定义空闲任务的栈大小 (通常在 Config.h 中，这里先手动补上) */
#define configMINIMAL_STACK_SIZE  ( ( unsigned short ) 128 )

#define configCPU_CLOCK_HZ (( uint32_t ) 480000000)
#define configTICK_RATE_HZ (( TickType_t ) 1000)


#endif //CHASE_LIGHT_OS_CHASE_LIGHT_OS_CONFIG_H