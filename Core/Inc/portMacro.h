#ifndef CHASE_LIGHT_OS_PORTMACRO_H
#define CHASE_LIGHT_OS_PORTMACRO_H

#include "stdint.h"
#include "stddef.h"
#include "Chase_Light_OS_Config.h"

/* 数据类型重定义 */
#define portCHAR char
#define portFLOAT float
#define portDOUBLE double
#define portLONG long
#define portSHORT short
#define portSTACK_TYPE uint32_t
#define portBASE_TYPE long

typedef portSTACK_TYPE StackType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;

#if( configUSE_16_BIT_TICKS == 1 )
typedef uint16_t TickType_t;
#define portMAX_DELAY ( TickType_t ) 0xffff
#else
typedef uint32_t TickType_t;
#define portMAX_DELAY ( TickType_t ) 0xffffffffUL
#endif

#define portNVIC_INT_CTRL_REG (*(( volatile uint32_t *) 0xe000ed04))

//ICSR 寄存器的 第 28 位，名字叫 PENDSVSET，这个寄存器的作用是，当其变为1时 PendSV 异常被悬起（Pending），只要没有更高优先级的中断在跑，CPU 就会立马跳进 PendSV_Handler（也就是我们写的 xPortPendSVHandler）去执行上下文切换。
#define portNVIC_PENDSVSET_BIT ( 1UL << 28UL )

//给汇编指令 DSB 和 ISB 用的参数
//15 (二进制 1111) 代表 SY (System)，也就是“全系统级同步”。
//意思是：这个屏障要管得宽一点，管住整个系统的读写顺序。
#define portSY_FULL_READ_WRITE (15)

/* GCC 版本的 portYIELD 实现 */
#define portYIELD()                                         \
{                                                           \
/* 1. 触发 PendSV，产生上下文切换 */                          \
portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;           \
                                                        \
/* 2. 数据同步屏障 (GCC 写法) */                          \
/* "memory" 告诉编译器：内存可能会变，别乱优化 */           \
__asm volatile( "dsb" ::: "memory" );                \
                                                    \
/* 3. 指令同步屏障 (GCC 写法) */                        \
__asm volatile( "isb" );                             \
}

//不带返回值的关中断函数，不能嵌套，不能在中断里使用
//BASEPRI：“中断屏蔽阈值”寄存器
static inline void vPortRaiseBASEPRI(void)
{
    uint32_t ulNewBASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY;
    __asm volatile
    (
        "msr basepri, %0 \n\t"  /* 将 ulNewBASEPRI 的值写入 BASEPRI 寄存器 */
        "dsb             \n\t"  /* 数据同步屏障 */
        "isb             \n\t"  /* 指令同步屏障 */
        :                       /* 无输出 */
        : "r" (ulNewBASEPRI)    /* 输入: %0 对应变量 ulNewBASEPRI */
        : "memory"              /* 告诉编译器内存可能会变，不要乱优化 */
    );
}
//函数作用：大于0x50的优先级的中断全被关闭，不可以嵌套不可以在中断里使用
#define portDISABLE_INTERRUPTS() vPortRaiseBASEPRI()

static inline uint32_t ulPortRaiseBASEPRI( void )
{
    uint32_t ulReturn;
    uint32_t ulNewBASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY;

    __asm volatile
    (
        "mrs %0, basepri \n\t"  /* 1. 先把当前 BASEPRI 的值读出来，存给 ulReturn (%0) */
        "msr basepri, %1 \n\t"  /* 2. 再把新值 (%1) 写入 BASEPRI，实施屏蔽 */
        "dsb             \n\t"  /* 3. 同步屏障 */
        "isb             \n\t"
        : "=r" (ulReturn)       /* 输出: %0 对应变量 ulReturn ("="表示写) */
        : "r" (ulNewBASEPRI)    /* 输入: %1 对应变量 ulNewBASEPRI */
        : "memory"
    );

    return ulReturn;            /* 返回关中断之前的状态 */
}
//带返回值的关中断函数可以嵌套，可以在中断里使用
#define portSET_INTERRUPT_MASK_FROM_ISR() ulPortRaiseBASEPRI()

//将BASEPRI的值改为上一次关中断时保存的BASEPRI值作为形参，与portDISABLE_INTERRUPTS成对使用
static inline __attribute__((always_inline)) void vPortSetBASEPRI( uint32_t ulNewBASEPRI )
{
    __asm volatile
    (
        "msr basepri, %0 \n\t"
        :
        :"r" (ulNewBASEPRI)
        : "memory"
    );
}

void vPortEnterCritical(void);
void vPortExitCritical(void);

//不带中断保护的开中断函数
#define portENABLE_INTERRUPTS() vPortSetBASEPRI( 0 )

//带中断保护的开中断函数
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x) vPortSetBASEPRI( x )

#define vPortClearBASEPRIFromISR()   vPortSetBASEPRI( 0 )

#define portENTER_CRITICAL() vPortEnterCritical()

//退出临界区
#define portEXIT_CRITICAL() vPortExitCritical()

/* ICSR 寄存器的 VECTACTIVE 字段掩码 (低9位) */
/* 0x1FF 表示二进制的 111111111，对应 Cortex-M4 的中断号字段 */
#define portVECTACTIVE_MASK ( 0x1FFUL )

#define portYIELD_FROM_ISR( xHigherPriorityTaskWoken ) \
if( ( xHigherPriorityTaskWoken ) != pdFALSE ) { portYIELD(); }

#endif