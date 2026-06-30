#ifndef CHASE_LIGHT_OS_PORT_H
#define CHASE_LIGHT_OS_PORT_H

#include "portMacro.h"
#include "Task.h"

#define portINITIAL_XPSR ( 0x01000000 )
#define portSTART_ADDRESS_MASK ( ( StackType_t ) 0xfffffffeUL )

#define portNVIC_SYSTICK_CTRL_REG (*(( volatile uint32_t * ) 0xe000e010))
#define portNVIC_SYSTICK_LOAD_REG (*((volatile uint32_t*) 0xe000e014))

#ifndef configSYSTICK_CLOCK_HZ
#define configSYSTICK_CLOCK_HZ configCPU_CLOCK_HZ

#define portNVIC_SYSTICK_CLK_BIT ( 1UL << 2UL )
#else
#define portNVIC_SYSTICK_CLK_BIT ( 0 )
#endif

#define portNVIC_SYSTICK_INT_BIT ( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT ( 1UL << 0UL )

StackType_t *pxPortInitialiseStack ( StackType_t *pxTopOfStack,TaskFunction_t pxCode,void*pvParameters);
__attribute__((naked)) void prvStartFirstTask(void);
void vPortSetupTimerInterrupt(void);
#endif //CHASE_LIGHT_OS_PORT_H