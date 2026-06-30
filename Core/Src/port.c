#include "port.h"


extern void *volatile pxCurrentTCB;
extern void HAL_IncTick(void);

extern void vTaskSwitchContext(void);

static UBaseType_t uxCriticalNesting = 0;

//错误处理，如果有错误，程序停在这
static void prvTaskExitError(void)
{
    /* 程序停止在这里 */
    for (;;);
}

//StackType_t -> uint32_t   TaskHandle_t -> void *
/* * 初始化任务栈函数
 * 作用：给寄存器的值进行一个预设（伪造现场），等待汇编指令将其载入CPU
 */

StackType_t *pxPortInitialiseStack(
    StackType_t *pxTopOfStack,
    TaskFunction_t pxCode,
    void *pvParameters
)
{
    pxTopOfStack--;
    *pxTopOfStack = portINITIAL_XPSR;

    pxTopOfStack--;
    *pxTopOfStack = ((StackType_t) pxCode) & portSTART_ADDRESS_MASK;

    pxTopOfStack--;
    *pxTopOfStack = (StackType_t) prvTaskExitError;

    pxTopOfStack -= 5;
    *pxTopOfStack = (StackType_t) pvParameters;

    /* 预留 EXC_RETURN，普通栈帧 = 0xFFFFFFFD */
    pxTopOfStack--;
    *pxTopOfStack = 0xFFFFFFFD;

    /* 预留 R4-R11 (8个普通寄存器) */
    pxTopOfStack -= 8;

    return pxTopOfStack;
}

// StackType_t *pxPortInitialiseStack(
//     StackType_t *pxTopOfStack,
//     TaskFunction_t pxCode,
//     void *pvParameters
// )
// {
//     pxTopOfStack--;
//     *pxTopOfStack = portINITIAL_XPSR;
//
//     pxTopOfStack--;
//     *pxTopOfStack = ((StackType_t) pxCode) & portSTART_ADDRESS_MASK;
//
//     pxTopOfStack--;
//     *pxTopOfStack = (StackType_t) prvTaskExitError;
//
//     pxTopOfStack -= 5;
//     *pxTopOfStack = (StackType_t) pvParameters;
//
//     /* 【核心修复 1】：为该任务预设一个干净的 EXC_RETURN (不带 FPU 帧) */
//     pxTopOfStack--;
//     *pxTopOfStack = 0xFFFFFFFD;
//
//     /* r4 - r11 (共 8 个寄存器) */
//     pxTopOfStack -= 8;
//
//     return pxTopOfStack;
// }
// StackType_t *pxPortInitialiseStack(
//     StackType_t *pxTopOfStack, // 空栈的“天花板”地址（数组末尾，最高地址）
//     TaskFunction_t pxCode, // 任务函数入口地址（函数指针）
//     void *pvParameters //万能指针包裹，任何参数都可以先变成void*传进来再拆
// )
// {
//     pxTopOfStack--;
//     *pxTopOfStack = portINITIAL_XPSR;
//
//     pxTopOfStack--;
//     *pxTopOfStack = ((StackType_t) pxCode) & portSTART_ADDRESS_MASK;
//
//     pxTopOfStack--;
//     *pxTopOfStack = (StackType_t) prvTaskExitError;
//
//     pxTopOfStack -= 5;
//     *pxTopOfStack = (StackType_t) pvParameters;
//
//     pxTopOfStack -= 8;
//
//     return pxTopOfStack;
// }

/* * 这里的 __attribute__((naked)) 是 GCC 的专用语法
 * 意思是：这是一个“裸函数”，编译器你别多管闲事，不要自动给我加压栈/出栈代码。
 * 里面所有的汇编，我自己负责！
 */
__attribute__((naked)) void prvStartFirstTask(void)
{
    /* GCC 内联汇编的标准写法：__asm volatile */
    __asm volatile (
        //ldr ->加载指令，将等号右侧的值，加载到左侧的寄存器里
        " ldr r0, = 0xE000ED08   \n" /* 1. 这里是硬件固定的门牌号，里面存着“向量表”在哪。 */
        " ldr r0, [r0]          \n" /* 2. 读取 VTOR 的值，得到向量表的起始地址 */
        " ldr r0, [r0]          \n" /* 3. 读取向量表的第一项（也就是 MSP 的初始值） */

        //msr->写入特殊寄存器指令，把R0的值（也就是MSP初始值）写入MSP，让其复位到最初始的状态;
        " msr msp, r0           \n" /* 4. 重置 MSP 主堆栈指针，把它复位到最干净的状态 */

        /*cpsie->全称Change Processor State, Enable Interrupts（改变处理器状态，使能中断）
         * i和f是其两个参数
         * i：IRQ（打开中断）
         * f：FIQ / Faults（打开异常开关）
         */
        " cpsie i               \n" /* 5. 开中断 (Primask) */
        " cpsie f               \n" /* 6. 开异常 (Faultmask) */

        //dsb->(Data Synchronization Barrier)指令同步屏障 意思是：等前面的数据操作全都彻底干完了，再继续。
        " dsb                   \n" /* 7. 数据同步屏障，防止指令乱序 */

        //isb->(Instruction Synchronization Barrier)指令同步屏障（写一次执行一次，用于已经执行了修改关键系统的操作时，要把预取的那些指令给作废，带着这些新状态重新去取）
        " isb                   \n" /* 8. 指令同步屏障 */

        //svc->Supervisor Call(超级用户调用)CPU自己产生一个异常，跳入SVC中断服务函数，在这个中断函数执行时，就会有将Task1堆栈保存的值恢复到CPU
        " svc 0                 \n" /* 9. 呼叫 SVC 中断，去启动第一个任务！ */


        " nop                   \n"
        " .align 4              \n" /* 10. 保持内存对齐 */
    );
}

//重写SVCHandler函数

__attribute__((naked)) void vPortSVCHandler(void)
{
    __asm volatile (
        " ldr r3, pxCurrentTCBConst     \n"
        " ldr r1, [r3]                  \n"
        " ldr r0, [r1]                  \n"

        /* 核心修复：把当初初始化压入的 r4-r11 以及 0xFFFFFFFD (r14) 一起弹出来 */
        " ldmia r0!, {r4-r11, r14}      \n"

        " msr psp, r0                   \n"
        " isb                           \n"
        " mov r0, #0                    \n"
        " msr basepri, r0               \n"
        /* 直接通过刚取出来的纯正魔术字跳转，不要用 orr r14, #0xd 了！ */
        " bx r14                        \n"

        " .align 4                      \n"
        "pxCurrentTCBConst: .word pxCurrentTCB \n"
    );
}

// __attribute__((naked)) void vPortSVCHandler(void)
// {
//     __asm volatile (
//         " ldr r3, pxCurrentTCBConst     \n"
//         " ldr r1, [r3]                  \n"
//         " ldr r0, [r1]                  \n"
//
//         /* 【核心修复 2】：连同刚才预留的 r14，一起弹出来 */
//         " ldmia r0!, {r4-r11, r14}      \n"
//
//         " msr psp, r0                   \n"
//         " isb                           \n"
//         " mov r0, #0                    \n"
//         " msr basepri, r0               \n"
//
//         /* 【核心修复 3】：删掉强行的 orr r14, #0xd，直接用真实弹出的 r14 跳转 */
//         " bx r14                        \n"
//
//         " .align 4                      \n"
//         "pxCurrentTCBConst: .word pxCurrentTCB \n"
//     );
// }
// __attribute__((naked)) void vPortSVCHandler(void)
// {
//     __asm volatile (
//
//         /* 1. 加载 pxCurrentTCB 的地址 */
//         " ldr r3, pxCurrentTCBConst     \n" /* 读取下方定义的常量标签地址，将 pxCurrentTCB的地址暂时存到R3里*/
//         " ldr r1, [r3]                  \n" /* r1 = pxCurrentTCB (TCB的地址)，将其存到R1*/
//
//         /* 2. 获取任务栈顶指针 */
//         " ldr r0, [r1]                  \n" /* r0 = pxCurrentTCB->pxTopOfStack */
//
//         /* 3. 恢复 R4-R11 (软件手动出栈) */
//         /* ldmia (Load Multiple Increment After)：多数据加载指令。意思是“从 R0 指向的地方开始读，读完一个 R0 自动加 4”
//          *R0!: 感叹号表示读完后，R0 的值会更新，指向读取后的新位置
//          *
//          */
//         " ldmia r0!, {r4-r11}           \n"
//
//         /* 4. 更新 PSP (进程堆栈指针) */
//         " msr psp, r0                   \n" /* 以后任务运行就用这个栈了 */
//         " isb                           \n"
//
//         /* 5. 打开中断 (把 BASEPRI 设为 0)
//          * BASEPRI:中断屏蔽寄存器，表示不会屏蔽任何中断（因为刚才我们关了中断，现在要重新打开）
//          */
//         " mov r0, #0                    \n"
//         " msr basepri, r0               \n"
//
//         /* 6. 设置返回模式 (Magic Number 0xD) */
//         /* 返回后进入线程模式 + 使用 PSP 堆栈 */
//         " orr r14, #0xd                 \n"
//
//         /* 7. 跳出异常，正式飞向任务！ */
//         " bx r14                        \n"
//
//         /* --- 常量池 (GCC 专用写法) --- */
//         " .align 4                      \n"
//         "pxCurrentTCBConst: .word pxCurrentTCB \n"
//     );
// }

// __attribute__((naked)) void xPortPendSVHandler(void)
// {
//     __asm volatile (
//         " mrs r0, psp                   \n"
//         " isb                           \n"
//         " ldr r3, pxCurrentTCBConst_PendSV \n"
//         " ldr r2, [r3]                  \n"
//
//         /* 【核心修复 4】：把当前快睡着的任务的 r14，压入它自己的栈里 */
//         " stmdb r0!, {r4-r11, r14}      \n"
//         " str r0, [r2]                  \n"
//
//         " stmdb sp!, {r3, r14}          \n"
//         " mov r0, %0                    \n"
//         " msr basepri, r0               \n"
//         " dsb                           \n"
//         " isb                           \n"
//         " bl vTaskSwitchContext         \n"
//         " mov r0, #0                    \n"
//         " msr basepri, r0               \n"
//         " ldmia sp!, {r3, r14}          \n"
//
//         " ldr r1, [r3]                  \n"
//         " ldr r0, [r1]                  \n"
//
//         /* 【核心修复 5】：把刚睡醒的任务的 r14，从它自己的栈里弹出来 */
//         " ldmia r0!, {r4-r11, r14}      \n"
//         " msr psp, r0                   \n"
//         " isb                           \n"
//         " bx r14                        \n"
//
//         " .align 4                      \n"
//         "pxCurrentTCBConst_PendSV: .word pxCurrentTCB      \n"
//         :
//         : "i"(configMAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
//     );
// }

__attribute__((naked)) void xPortPendSVHandler(void)
{
    __asm volatile (
        /* --- 第一阶段：保存旧任务现场 --- */
        " mrs r0, psp                   \n"
        " isb                           \n"
        " ldr r3, pxCurrentTCBConst_PendSV \n"
        " ldr r2, [r3]                  \n"

        /* 【FPU 终极防御：如果任务用了浮点，保存 S16-S31】 */
        /* R14 (EXC_RETURN) 的 bit 4 如果是 0，说明使用了 FPU 扩展栈帧 */
        " tst r14, #0x10                \n"
        " it eq                         \n"
        " vstmdbeq r0!, {s16-s31}       \n" /* 只有 eq (等于0) 时，才把 S16-S31 压入当前任务的 PSP 栈里 */

        /* 保存普通的 R4-R11 和 R14 魔术字 */
        " stmdb r0!, {r4-r11, r14}      \n"
        " str r0, [r2]                  \n" /* 更新 TCB 里的栈顶指针 */

        /* --- 第二阶段：调用 C 函数挑选下一个任务 --- */
        " stmdb sp!, {r3, r14}          \n"
        " mov r0, %0                    \n"
        " msr basepri, r0               \n"
        " dsb                           \n"
        " isb                           \n"
        " bl vTaskSwitchContext         \n"
        " mov r0, #0                    \n"
        " msr basepri, r0               \n"
        " ldmia sp!, {r3, r14}          \n"

        /* --- 第三阶段：恢复新任务现场 --- */
        " ldr r1, [r3]                  \n"
        " ldr r0, [r1]                  \n"

        /* 弹出普通的 R4-R11 和 R14 魔术字 */
        " ldmia r0!, {r4-r11, r14}      \n"

        /* 【FPU 终极防御：如果新任务睡前用了浮点，把它的 S16-S31 还给它】 */
        " tst r14, #0x10                \n"
        " it eq                         \n"
        " vldmiaeq r0!, {s16-s31}       \n" /* 只有 eq 时，才把栈里的浮点数据弹回硬件寄存器 */

        " msr psp, r0                   \n"
        " isb                           \n"
        " bx r14                        \n" /* 跳转恢复！ */

        /* --- 数据池 --- */
        " .align 4                      \n"
        "pxCurrentTCBConst_PendSV: .word pxCurrentTCB      \n"
        "vTaskSwitchContextConst: .word vTaskSwitchContext \n"
        : /* 无输出 */
        : "i"(configMAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
    );
}

// __attribute__((naked)) void xPortPendSVHandler(void)
// {
//     __asm volatile (
//         /* --- 第一阶段：保存旧任务现场 --- */
//         //mrs -> 将特殊寄存器的值移动到基础寄存器
//         " mrs r0, psp                   \n"
//         //isb->(Instruction Synchronization Barrier)指令同步屏障（写一次执行一次，用于已经执行了修改关键系统的操作时，要把预取的那些指令给作废，带着这些新状态重新去取）
//         " isb                           \n"
//
//         /* 【注意】这里改名了！引用下面的新名字 */
//         //加载 pxCurrentTCBConst 这个标签所在的地址到 R3，查找当前是谁在跑
//         " ldr r3, pxCurrentTCBConst_PendSV \n"
//
//         //去 R3 指向的地址读数据，存入 R2，现在R2就等于正在运行任务的TCB首地址
//         " ldr r2, [r3]                  \n"
//         //stmdb -> Store Multiple Decrement Before. 从 R0 指向的地址开始，向下（地址减小方向）压栈，把 R4 到 R11 存进去。! 表示存完后，自动更新 R0 的值
//         " stmdb r0!, {r4-r11}           \n"
//         //把 R0 的值写入到 R2 指向的内存地址,等价于 C 语言的 Task1TCB->pxTopOfStack = r0;
//         " str r0, [r2]                  \n"
//
//         /* --- 第二阶段：切换核心 --- */
//         //把 R3 和 R14 压入 MSP (主堆栈)
//         " stmdb sp!, {r3, r14}          \n"
//         //把输入参数 %0 (即优先级阈值) 放入 R0
//         " mov r0, %0                    \n"
//         " msr basepri, r0               \n"
//         " dsb                           \n"
//         " isb                           \n"
//         //bl -> Branch with Link（跳转加保存地址）
//         " bl vTaskSwitchContext         \n"
//         " mov r0, #0                    \n"
//         " msr basepri, r0               \n"
//         " ldmia sp!, {r3, r14}          \n"
//
//         /* --- 第三阶段：恢复新任务现场 --- */
//         " ldr r1, [r3]                  \n"
//         " ldr r0, [r1]                  \n"
//         //ldmia -> Load Multiple Increment After. 从 R0 指向的地址开始读，填入 R4-R11
//         " ldmia r0!, {r4-r11}           \n"
//         " msr psp, r0                   \n"
//         " isb                           \n"
//         " bx r14                        \n"
//
//         /* --- 数据池 --- */
//         " .align 4                      \n"
//
//         /* 【注意】这里是定义标签的地方，必须和上面引用的名字一致，且不能和 SVC 函数里的重名 */
//         "pxCurrentTCBConst_PendSV: .word pxCurrentTCB      \n"
//
//         "vTaskSwitchContextConst: .word vTaskSwitchContext \n"
//         : /* 无输出 */
//         : "i"(configMAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
//     );
// }

//进入临界区函数
void vPortEnterCritical(void)
{
    //关中断
    portDISABLE_INTERRUPTS();
    uxCriticalNesting++;

    if (uxCriticalNesting == 1)
    {
        configASSERT((portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK) == 0);
    }
}

//退出临界区函数
void vPortExitCritical(void)
{
    configASSERT( uxCriticalNesting );
    uxCriticalNesting--;

    if (uxCriticalNesting == 0)
    {
        portENABLE_INTERRUPTS();
    }
}

void xPortSysTickHandler(void)
{
    vPortRaiseBASEPRI();
    xTaskIncrementTick();
    vPortClearBASEPRIFromISR();
}

void vPortSetupTimerInterrupt(void)
{
    //设置重装载寄存器的值
    portNVIC_SYSTICK_LOAD_REG = (configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ) - 1UL;

    portNVIC_SYSTICK_CTRL_REG = (portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT);
}