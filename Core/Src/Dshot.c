#include "Dshot.h"
#include "tim.h"
#include "dma.h"


#define DSHOT_MOTOR_COUNT 4U
#define DSHOT_BURST_BUF_LEN (DSHOT_FRAME_LEN * DSHOT_MOTOR_COUNT)
#define DSHOT_BURST_BUF_BYTES (DSHOT_BURST_BUF_LEN * sizeof(uint32_t))
#define DSHOT_DMA_BUF_ADDR ((uint32_t *)0x30000400U)

static uint32_t * const dshot_dma_buf = DSHOT_DMA_BUF_ADDR;

//dma在忙标志
static volatile uint8_t dshot_busy = 0;

static uint8_t Dshot_CalcCRC(uint16_t packet)
{
    return (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
}

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

static uint16_t Dshot_MakeFrame(uint16_t value,uint8_t telemetry)
{
    uint16_t packet;
    uint8_t crc;

    value = Dshot_ClampThrottle(value);

    value &= 0x07FFU;

    telemetry = telemetry ? 1U : 0U;

    packet = (uint16_t)((value << 1) | telemetry);

    crc = Dshot_CalcCRC(packet);

    return (uint16_t)((packet << 4) | crc);
}

static void Dshot_PackBurstFrame(uint16_t m0,uint16_t m1,uint16_t m2,uint16_t m3)
{
    uint16_t frame[DSHOT_MOTOR_COUNT];

    frame[0] = Dshot_MakeFrame(m0,0U);
    frame[1] = Dshot_MakeFrame(m1,0U);
    frame[2] = Dshot_MakeFrame(m2,0U);
    frame[3] = Dshot_MakeFrame(m3,0U);

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


static void Dshot_DMA_CpltCallback(DMA_HandleTypeDef *hdma)
{
    (void)hdma;

    /*
     * 一帧 DShot 已经搬完，后续不需要 TIM8 update 再继续触发 DMA。
     * 这里不要 HAL_DMA_Abort：完成回调本身就是 DMA 正常结束路径，
     * 在回调里 abort 容易破坏下一帧启动时的 DMA 状态。
     */
    __HAL_TIM_DISABLE_DMA(&htim8, TIM_DMA_UPDATE);

    dshot_busy = 0U;
}

void Dshot_WriteAll(uint16_t m0, uint16_t m1, uint16_t m2, uint16_t m3)
{
    if (dshot_busy)
    {
        return;
    }

    Dshot_PackBurstFrame(m0,m1,m2,m3);

    SCB_CleanDCache_by_Addr((uint32_t *)dshot_dma_buf,
                            (int32_t)DSHOT_BURST_BUF_BYTES);

    dshot_busy = 1U;

    __HAL_TIM_DISABLE_DMA(&htim8, TIM_DMA_UPDATE);

    htim8.Instance->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_4TRANSFERS;

    if (HAL_DMA_Start_IT(htim8.hdma[TIM_DMA_ID_UPDATE],
        (uint32_t)dshot_dma_buf,
        (uint32_t)&htim8.Instance->DMAR,
        DSHOT_BURST_BUF_LEN) != HAL_OK)
    {
        dshot_busy = 0U;
        return;
    }

    __HAL_TIM_SET_COUNTER(&htim8 ,0U);

    __HAL_TIM_ENABLE_DMA(&htim8, TIM_DMA_UPDATE);
}

uint8_t Dshot_Ready(void)
{
    return dshot_busy == 0U;
}

void Dshot_Init(void)
{
    htim8.hdma[TIM_DMA_ID_UPDATE]->XferCpltCallback = Dshot_DMA_CpltCallback;

    Dshot_PackBurstFrame(0U,0U,0U,0U);

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    dshot_busy = 0;
}
