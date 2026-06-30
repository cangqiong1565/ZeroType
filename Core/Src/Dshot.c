#include "Dshot.h"
#include "tim.h"

/* ══════════════════════════════════════════════════════
 * 1. 每个电机的 DMA 缓冲区  [电机号][18 个 CCR 值]
 *    DMA 把它逐个灌进 TIM8->CCRx，定时器自动输出波形
 * ══════════════════════════════════════════════════════ */
static uint16_t dshot_dma_buf[4][DSHOT_FRAME_LEN];

/* DMA 忙标志：1=正在发，0=空闲可写 */
static volatile uint8_t dshot_busy[4];

/* 当前正在发送的电机号，-1 表示全空闲 */
static volatile int8_t dshot_current_motor = -1;

/* ══════════════════════════════════════════════════════
 * 2. DMA 传输完成回调（H7 HAL 用函数指针，不是 __weak）
 *    HAL_DMA_IRQHandler 检测到 TC 中断后调这个指针
 * ══════════════════════════════════════════════════════ */
static void Dshot_DMA_CpltCallback(DMA_HandleTypeDef *hdma)
{
    (void)hdma;   /* 只用全局变量，参数用不到 */
    if (dshot_current_motor >= 0) {
        dshot_busy[dshot_current_motor] = 0;
        dshot_current_motor = -1;
    }
}

/* ══════════════════════════════════════════════════════
 * 3. CRC4 计算 — XOR 折叠法（Betaflight 官方实现）
 *    输入：油门(11bit) + 遥测(1bit) 拼成的 12 位
 *    输出：4 位校验码
 *    data12 的 bit11=遥测, bit10~0=油门
 * ══════════════════════════════════════════════════════ */
static uint8_t Dshot_CalcCRC(uint16_t data12)
{
    return (uint8_t)((data12 ^ (data12 >> 4) ^ (data12 >> 8)) & 0x0F);
}

/* ══════════════════════════════════════════════════════
 * 4. 装填 DMA 缓冲区：把油门 + 遥测 + CRC 拆成 16 个 bit，
 *    每个 bit 映射为 CCR 值（0→7, 1→15），再加 2 个帧尾 0
 * ══════════════════════════════════════════════════════ */
static void Dshot_PackFrame(uint16_t throttle, uint8_t telemetry,
                            uint16_t *buf)
{
    /* ── 拼出待校验的 12 位：bit11=遥测, bit10~0=油门 ── */
    uint16_t data12 = ((uint16_t)telemetry << 11) | (throttle & 0x7FF);

    /* ── 算 CRC ── */
    uint8_t crc = Dshot_CalcCRC(data12);

    /* ── 拼 16 位帧：bit15~12=CRC, bit11=遥测, bit10~0=油门 ── */
    uint16_t frame = ((uint16_t)crc << 12) | data12;

    /* ── 高位先发：bit15→buf[0], bit14→buf[1], …, bit0→buf[15] ── */
    for (int i = 0; i < 16; i++) {
        uint8_t bit = (frame >> (15 - i)) & 0x01;
        buf[i] = bit ? DSHOT_BIT_1 : DSHOT_BIT_0;
    }

    /* ── 帧尾：2 位全低，给电调时间识别帧结束 ── */
    buf[16] = 0;
    buf[17] = 0;
}

/* ══════════════════════════════════════════════════════
 * 5. 初始化：注册回调、开 DMA、发首帧 0 让电调识别协议
 * ══════════════════════════════════════════════════════ */
void Dshot_Init(void)
{
    /* ── H7 HAL：注册回调函数指针（不是重写 __weak） ── */
    htim8.hdma[TIM_DMA_ID_UPDATE]->XferCpltCallback = Dshot_DMA_CpltCallback;

    /* ── 缓冲区清零，全部电机初始发 0（停转） ── */
    for (int m = 0; m < 4; m++) {
        dshot_busy[m] = 0;
        Dshot_PackFrame(0, 0, dshot_dma_buf[m]);
    }

    /* ── 使能 TIM8 更新事件触发 DMA ── */
    __HAL_TIM_ENABLE_DMA(&htim8, TIM_DMA_UPDATE);

    /* ── 首帧发 0，告诉电调"这是 Dshot 信号" ── */
    HAL_DMA_Start_IT(htim8.hdma[TIM_DMA_ID_UPDATE],
                     (uint32_t)dshot_dma_buf[0],
                     (uint32_t)&htim8.Instance->CCR1,
                     DSHOT_FRAME_LEN);
    dshot_busy[0] = 1;
    dshot_current_motor = 0;

    __HAL_TIM_ENABLE(&htim8);
}

/* ══════════════════════════════════════════════════════
 * 6. 发送一帧（PID_Task 里调用）
 *    非阻塞：如果该电机 DMA 忙，直接 return
 *    CCR1~CCR4 地址依次相差 4 字节
 * ══════════════════════════════════════════════════════ */
void Dshot_Write(uint8_t motor, uint16_t throttle, uint8_t telemetry)
{
    if (motor >= 4) return;
    if (dshot_busy[motor]) return;

    /* ── 限幅 ── */
    if (throttle > DSHOT_THROTTLE_MAX) throttle = DSHOT_THROTTLE_MAX;
    if (throttle > 0 && throttle < DSHOT_THROTTLE_MIN) throttle = DSHOT_THROTTLE_MIN;

    /* ── 装填 ── */
    Dshot_PackFrame(throttle, telemetry, dshot_dma_buf[motor]);

    /* ── 启动 DMA ── */
    uint32_t ccr_addr = (uint32_t)(&htim8.Instance->CCR1) + motor * sizeof(uint32_t);

    HAL_DMA_Start_IT(htim8.hdma[TIM_DMA_ID_UPDATE],
                     (uint32_t)dshot_dma_buf[motor],
                     ccr_addr,
                     DSHOT_FRAME_LEN);
    dshot_busy[motor] = 1;
    dshot_current_motor = (int8_t)motor;
}

/* ══════════════════════════════════════════════════════
 * 7. 查询 DMA 是否空闲
 * ══════════════════════════════════════════════════════ */
uint8_t Dshot_Ready(uint8_t motor)
{
    if (motor >= 4) return 0;
    return !dshot_busy[motor];
}
