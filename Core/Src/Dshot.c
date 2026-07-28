#include "Dshot.h"
#include "tim.h"
#include "dma.h"
#include "Task.h"
#include "Queue.h"
#define DSHOT_DMA_EVENT_DONE  1U
#define DSHOT_DMA_EVENT_ERROR 2U

//DMA完成事件队列
static Queue_t dshot_dma_done_queue;                            //队列控制块本体
static uint8_t dshot_dma_done_storage[1];                       //数据存储区
static QueueHandle_t dshot_dma_done_queue_handle = NULL;        //队列句柄

#define DSHOT_MOTOR_COUNT 4U
#define DSHOT_BURST_BUF_LEN (DSHOT_FRAME_LEN * DSHOT_MOTOR_COUNT)
#define DSHOT_BURST_BUF_BYTES (DSHOT_BURST_BUF_LEN * sizeof(uint32_t))

static uint32_t dshot_dma_buf[DSHOT_BURST_BUF_LEN]
        __attribute__((__section__(".dma_buffer"), aligned(32)));

//dma在忙标志
static volatile uint8_t dshot_busy = 0;

//CRC校验计算函数，参考BF官方文档
static uint8_t Dshot_CalcCRC(uint16_t packet)
{
    return (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
}

/*油门限幅函数，把油门限制在合理范围内
 * 大于0且小于48就等于48
 * 大于2047就等于2047
 */
static uint16_t Dshot_ClampThrottle(uint16_t throttle)
{
    if (throttle > DSHOT_THROTTLE_MAX)
    {
        throttle = DSHOT_THROTTLE_MAX;
    }

    if ((throttle > 0U) && (throttle < DSHOT_THROTTLE_MIN))
    {
        throttle = DSHOT_THROTTLE_MIN;
    }

    return throttle;
}

/*
 * 单电机帧生成函数
 * 将油门值和遥测请求(telemetry)打包
 */
static uint16_t Dshot_MakeFrame(uint16_t value,uint8_t telemetry)
{
    uint16_t packet;    //包
    uint8_t crc;        //校验

    value = Dshot_ClampThrottle(value);     //油门进来先做限幅

    value &= 0x07FFU;                       //

    telemetry = telemetry ? 1U : 0U;        //遥测要么是0要么是1,来自BF文档

    packet = (uint16_t)((value << 1) | telemetry);//将value左移一位与遥测请求拼起来

    crc = Dshot_CalcCRC(packet);            //计算CRC校验

    return (uint16_t)((packet << 4) | crc);//将CRC拼进去
}

/*
 * DShot 特殊命令帧生成函数。
 *
 * DShot 的 0..47 是命令区，例如：
 * 12 = SAVE_SETTINGS
 * 20 = SPIN_DIRECTION_NORMAL
 * 21 = SPIN_DIRECTION_REVERSED
 *
 * 这些值不能经过 Dshot_ClampThrottle()，否则 1..47 会被改成 48，
 * 电调就收不到真正的命令。
 */
static uint16_t Dshot_MakeCommandFrame(uint16_t command)
{
    uint16_t packet;
    uint8_t crc;
    uint8_t telemetry;

    /*
     * 只允许 0..47 的 DShot 命令值。
     * 如果传进来超范围，按 0 处理，也就是 motor stop。
     */
    if (command > 47U)
    {
        command = 0U;
    }

    /*
     * Betaflight 在发送非 0 DShot 命令时会 requestTelemetry。
     * 普通 motor stop 命令不请求遥测。
     */
    telemetry = (command != 0U) ? 1U : 0U;

    command &= 0x07FFU;
    packet = (uint16_t)((command << 1) | telemetry);
    crc = Dshot_CalcCRC(packet);

    return (uint16_t)((packet << 4) | crc);
}

//DMA波形打包函数
static void Dshot_PackBurstFrame(uint16_t m0,uint16_t m1,uint16_t m2,uint16_t m3)
{
    uint16_t frame[DSHOT_MOTOR_COUNT];
    //将四个油门值和遥测值生成Dshot帧
    frame[0] = Dshot_MakeFrame(m0,0U);
    frame[1] = Dshot_MakeFrame(m1,0U);
    frame[2] = Dshot_MakeFrame(m2,0U);
    frame[3] = Dshot_MakeFrame(m3,0U);

    //把数据按位拆分，转换成PWM占空比，写入发送数组
    for (uint8_t bit = 0U; bit < 16U; bit++)
    {
        //逐位提取
        uint16_t mask = (uint16_t)(1U << (15U - bit));

        //数组索引
        uint32_t base = (uint32_t)bit * DSHOT_MOTOR_COUNT;

        //每一位都单独拆分，就剩当前要写入的这位，如果为1就写入15,为0写入7
        dshot_dma_buf[base + 0U] = (frame[0] & mask) ? DSHOT_BIT_1 : DSHOT_BIT_0;
        dshot_dma_buf[base + 1U] = (frame[1] & mask) ? DSHOT_BIT_1 : DSHOT_BIT_0;
        dshot_dma_buf[base + 2U] = (frame[2] & mask) ? DSHOT_BIT_1 : DSHOT_BIT_0;
        dshot_dma_buf[base + 3U] = (frame[3] & mask) ? DSHOT_BIT_1 : DSHOT_BIT_0;
    }

    //写入两位结束位（应该是帧结束吧，一会看看）
    for (uint8_t tail = 16U; tail < DSHOT_FRAME_LEN; tail++)
    {
        uint32_t base = (uint32_t)tail * DSHOT_MOTOR_COUNT;

        dshot_dma_buf[base + 0U] = 0U;
        dshot_dma_buf[base + 1U] = 0U;
        dshot_dma_buf[base + 2U] = 0U;
        dshot_dma_buf[base + 3U] = 0U;
    }
}

/*
 * DShot 特殊命令 DMA 波形打包函数。
 *
 * 和 Dshot_PackBurstFrame() 的区别只有一个：
 * 这里使用 Dshot_MakeCommandFrame()，不会把 20/21/12 这类命令限幅成 48。
 */
static void Dshot_PackBurstCommandFrame(uint16_t c0,uint16_t c1,uint16_t c2,uint16_t c3)
{
    uint16_t frame[DSHOT_MOTOR_COUNT];

    frame[0] = Dshot_MakeCommandFrame(c0);
    frame[1] = Dshot_MakeCommandFrame(c1);
    frame[2] = Dshot_MakeCommandFrame(c2);
    frame[3] = Dshot_MakeCommandFrame(c3);

    for (uint8_t bit = 0U; bit < 16U; bit++)
    {
        uint16_t mask = (uint16_t)(1U << (15U - bit));
        uint32_t base = (uint32_t)bit * DSHOT_MOTOR_COUNT;

        dshot_dma_buf[base + 0U] = (frame[0] & mask) ? DSHOT_BIT_1 : DSHOT_BIT_0;
        dshot_dma_buf[base + 1U] = (frame[1] & mask) ? DSHOT_BIT_1 : DSHOT_BIT_0;
        dshot_dma_buf[base + 2U] = (frame[2] & mask) ? DSHOT_BIT_1 : DSHOT_BIT_0;
        dshot_dma_buf[base + 3U] = (frame[3] & mask) ? DSHOT_BIT_1 : DSHOT_BIT_0;
    }

    for (uint8_t tail = 16U; tail < DSHOT_FRAME_LEN; tail++)
    {
        uint32_t base = (uint32_t)tail * DSHOT_MOTOR_COUNT;

        dshot_dma_buf[base + 0U] = 0U;
        dshot_dma_buf[base + 1U] = 0U;
        dshot_dma_buf[base + 2U] = 0U;
        dshot_dma_buf[base + 3U] = 0U;
    }
}

/*Dshot中断事件发送函数
 * 此函数为私有函数，专门服务于
 * Dshot_DMA_CpltCallback()
 * Dshot_DMA_ErrorCallback()
 * 这两个回调，作用是把发送结果作为两个事件发出去
 */
static void Dshot_SendDmaEventFromISR(uint8_t event)
{
    //高优先级任务唤醒标志，如果有更高优先级任务被唤醒，
    //如果有，xQueueGenericSendFromISR会决定中断退出以后要不要进行一次任务切换
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    //判断DMA事件完成队列是否被创建成功，如果它是NULL说明还没有执行Dshot_Init()
    if (dshot_dma_done_queue_handle != NULL)
    {
        //句柄有效则从中断里发送一个事件
        (void)xQueueGenericSendFromISR(
            dshot_dma_done_queue_handle,//目标队列
            &event,                     //本次事件
            &xHigherPriorityTaskWoken   //如果有更高优先级任务被唤醒，这个函数会把值改成pdTRUE
        );

        //根据 xHigherPriorityTaskWoken 判断是否需要中断后立刻切换任务
        //如果 xHigherPriorityTaskWoken == pdTRUE
        //说明有更高优先级任务刚刚被这个事件唤醒
        //那就触发一次 PendSV，让 RTOS 在中断退出后切任务
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

//DMA发送完成回调函数
static void Dshot_DMA_CpltCallback(DMA_HandleTypeDef *hdma)
{
    //明确告诉编译器这个参数没有使用
    (void)hdma;

    //发送完成后失能DMA,清标志位，因为一帧 DShot 已经搬完了
    //如果不关，TIM8 后续 update 事件还可能继续尝试触发 DMA
    //容易影响下一帧启动时的状态
    __HAL_TIM_DISABLE_DMA(&htim8,TIM_DMA_UPDATE);

    //DShot 发送不忙了，允许下一帧启动
    dshot_busy = 0U;

    //通知等待 DMA 的任务：这一帧正常完成（体现了上一个函数的作用）
    Dshot_SendDmaEventFromISR(DSHOT_DMA_EVENT_DONE);
}

//Dshot旧DMA事件清空函数（讲）
static void Dshot_ClearOldDmaEvents(void)
{
    //事件变量
    //这里的 event 只是用来接收从队列里取出来的旧事件
    //取出来后不处理，直接丢弃
    uint8_t event;

    //判断 DShot DMA 事件队列是否已经创建
    //
    //如果句柄是 NULL
    //说明 Dshot_Init() 还没执行，或者队列创建失败，直接返回
    if (dshot_dma_done_queue_handle == NULL)
    {
        return;
    }

    //xQueueGenericReceive的作用是从队列里接收一个元素
    //第一个参数：dshot_dma_done_queue_handle
    //表示从 DShot DMA 完成事件队列里取
    //第二个参数：&event
    //表示把取出来的事件值存到 event 变量里
    //第三个参数：0U
    //表示不等待
    //如果队列里现在没有事件，马上返回失败
    while (xQueueGenericReceive(dshot_dma_done_queue_handle, &event, 0U) == pdPASS)
    {
        //清空旧事件
    }
}

//四电机发送函数
uint8_t Dshot_WriteAll(uint16_t m0, uint16_t m1, uint16_t m2, uint16_t m3)
{
    //标志为忙代表上一帧还没发完，直接返回
    if (dshot_busy != 0U)
    {
        return 0U;
    }

    //清空旧事件，比如上一帧完成后留下了 DONE 事件如果不清掉，后面等待本帧完成时可能误读到旧事件
    Dshot_ClearOldDmaEvents();

    //打包波形
    Dshot_PackBurstFrame(m0,m1,m2,m3);

    /*判断CPU的DCache是否开启，SCB->CCR是 Cortex-M7 的系统控制寄存器,是DCache的Enable位
    * 如果不为0说明已经开启
    * 为什么要判断？
    * 因为STM32H7 的 CPU 写 dshot_dma_buf 时，数据可能先写进 Cache
    * DMA 读取的是 SRAM，不会自动读取 CPU Cache
    * 所以如果 Cache 开着，DMA 可能读到旧数据
    */
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_CleanDCache_by_Addr((uint32_t *)dshot_dma_buf,
                                (int32_t)DSHOT_BURST_BUF_BYTES);
        //如果开着就把Cache里的数据清空回SRAM，这样DMA从SRAM读取时，才能读到刚刚打包好的Dshot波形
    }

    //进入临界区保护
    taskENTER_CRITICAL();

    //标志为忙直接退出中断区返回
    if (dshot_busy != 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }

    //如果不忙，标志为忙，表示开始发送DMA
    dshot_busy = 1U;

    //先关闭TIM8 update事件触发DMA，否则在DMA还没配置完整前，TIM8可能已经触发了一次DMA请求
    __HAL_TIM_DISABLE_DMA(&htim8, TIM_DMA_UPDATE);

    //清除TIM8的update标志位，如果之前已经产生过update事件，这个标志可能还挂着
    //启动DMA前清掉它，避免刚打开DMA请求时，立刻响应一个旧的update事件
    __HAL_TIM_CLEAR_FLAG(&htim8,TIM_FLAG_UPDATE);

    //配置TIM8的DMA Burst模式，TIM_DMABASE_CCR1 表示DMA Burst从CCR1寄存器开始写
    //TIM_DMABURSTLENGTH_4TRANSFERS 表示每次Burst连续写4个寄存器
    //总结就是，每次触发DMA时，都连续更新CCR1,CCR2,CCR3,CCR4
    htim8.Instance->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_4TRANSFERS;

    /* 启动DMA中断传输
     * 参数1：htim8.hdma[TIM_DMA_ID_UPDATE]，TIM8 UPDATE事件绑定的句柄，这个DMA通道由TIM8UPDATE事件触发
     * 参数2：(uint32_t)dshot_dma_buf,刚打包好的dma数据
     * 参数3: (uint32_t)&htim8.Instance->DMAR,DMA目标地址，不是直接写CCR1地址，而是TIM8的DMAR寄存器
     * 参数4: DMA要搬运的数据数量
     */
    if (HAL_DMA_Start_IT(htim8.hdma[TIM_DMA_ID_UPDATE],
        (uint32_t)dshot_dma_buf,
        (uint32_t)&htim8.Instance->DMAR,
        DSHOT_BURST_BUF_LEN) != HAL_OK)
    {
        //启动失败，把标志位清回0，退出中断区，返回
        dshot_busy = 0U;
        taskEXIT_CRITICAL();
        return 0U;
    }

    //把TIM8计数器清零
    //这样新一帧Dshot从一个完整的新PWM周期开始
    __HAL_TIM_SET_COUNTER(&htim8 ,0U);

    //打开TIM8 update DMA请求
    //从这一句开始，TIM8每次产生update事件
    //都会触发DMA搬运数据到TIM8->DMAR
    //也就是Dshot波形真正开始输出
    //前一个HAL_DMA_Start_IT是让DMA处于等待触发状态
    __HAL_TIM_ENABLE_DMA(&htim8, TIM_DMA_UPDATE);

    //都完成，退出临界区
    taskEXIT_CRITICAL();

    return 1U;
}

//四电机 DShot 特殊命令发送函数
uint8_t Dshot_WriteAllCommand(uint16_t c0, uint16_t c1, uint16_t c2, uint16_t c3)
{
    if (dshot_busy != 0U)
    {
        return 0U;
    }

    Dshot_ClearOldDmaEvents();

    /*
     * 注意这里打包的是特殊命令帧，不是普通油门帧。
     * c0..c3 可以是 12/20/21 这种低于 48 的命令值。
     */
    Dshot_PackBurstCommandFrame(c0,c1,c2,c3);

    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_CleanDCache_by_Addr((uint32_t *)dshot_dma_buf,
                                (int32_t)DSHOT_BURST_BUF_BYTES);
    }

    taskENTER_CRITICAL();

    if (dshot_busy != 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }

    dshot_busy = 1U;

    __HAL_TIM_DISABLE_DMA(&htim8, TIM_DMA_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim8,TIM_FLAG_UPDATE);

    htim8.Instance->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_4TRANSFERS;

    if (HAL_DMA_Start_IT(htim8.hdma[TIM_DMA_ID_UPDATE],
        (uint32_t)dshot_dma_buf,
        (uint32_t)&htim8.Instance->DMAR,
        DSHOT_BURST_BUF_LEN) != HAL_OK)
    {
        dshot_busy = 0U;
        taskEXIT_CRITICAL();
        return 0U;
    }

    __HAL_TIM_SET_COUNTER(&htim8 ,0U);
    __HAL_TIM_ENABLE_DMA(&htim8, TIM_DMA_UPDATE);

    taskEXIT_CRITICAL();

    return 1U;
}

//完成等待函数
uint8_t Dshot_WaitDmaDone(uint32_t timeout_ticks)
{
    //事件变量
    uint8_t event = 0U;
    //句柄为空就返回
    if (dshot_dma_done_queue_handle == NULL)
    {
        return 0U;
    }

    ///从Dshot DMA事件队列里等待一个事件
    if (xQueueGenericReceive(dshot_dma_done_queue_handle,
                             &event,
                             timeout_ticks) != pdPASS)
    {
        taskENTER_CRITICAL();
        __HAL_TIM_DISABLE_DMA(&htim8,TIM_DMA_UPDATE);
        taskEXIT_CRITICAL();

        (void)HAL_DMA_Abort(htim8.hdma[TIM_DMA_ID_UPDATE]);

        taskENTER_CRITICAL();
        dshot_busy = 0U;
        taskEXIT_CRITICAL();

        return 0U;
    }

    return event == DSHOT_DMA_EVENT_DONE;
}

uint8_t Dshot_Ready(void)
{
    return dshot_busy == 0U;
}

//错误回调函数
static void Dshot_DMA_ErrorCallback(DMA_HandleTypeDef *hdma)
{
    (void)hdma;

    __HAL_TIM_DISABLE_DMA(&htim8, TIM_DMA_UPDATE);

    dshot_busy = 0U;

    Dshot_SendDmaEventFromISR(DSHOT_DMA_EVENT_ERROR);
}

//初始化（讲）
void Dshot_Init(void)
{
    //创建句柄
    dshot_dma_done_queue_handle = xQueueCreateStatic(1,
        sizeof(uint8_t),
        dshot_dma_done_storage,
        &dshot_dma_done_queue);

    //把Dshot DMA完成回调函数挂到TIM8 update DMA句柄上，也就是HAL库会接入我们的回调函数
    htim8.hdma[TIM_DMA_ID_UPDATE]->XferCpltCallback = Dshot_DMA_CpltCallback;
    htim8.hdma[TIM_DMA_ID_UPDATE]->XferErrorCallback = Dshot_DMA_ErrorCallback;

    //初始化一帧0油门，防止失控
    Dshot_PackBurstFrame(0U,0U,0U,0U);

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    //忙碌标志位空闲
    dshot_busy = 0U;
}
