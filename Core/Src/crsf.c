#include "crsf.h"
#include "main.h"
#include "usart.h"
#include <string.h>

typedef enum {
    CRSF_STATE_WAIT_SYNC = 0,
    CRSF_STATE_WAIT_LEN,
    CRSF_STATE_WAIT_TYPE,
    CRSF_STATE_RECV_PAYLOAD,
    CRSF_STATE_RECV_CRC,
} crsf_state_t;

static uint8_t  rx_buf[CRSF_MAX_FRAME];
static uint8_t  rx_idx;
static uint8_t  rx_payload_len;   /* = LEN - 2 */
static crsf_state_t rx_state;

static volatile uint16_t channels[CRSF_NUM_CHANNELS];
static volatile bool     new_frame;
static volatile uint32_t last_frame_tick;

static CRSF_Stats stats;

/* ----------------------------------------------------------------
 * CRC8 — poly 0xD5, init 0x00 (CRSF/TBS/ELRS 标准)
 * ---------------------------------------------------------------- */
static uint8_t crsf_crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5)
                               : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* ----------------------------------------------------------------
 * RC 帧解包: 22 字节 payload -> 11 通道 x 11 bit (小端位打包)
 * ---------------------------------------------------------------- */
static void crsf_decode_rc(const uint8_t *p, uint16_t ch[CRSF_NUM_CHANNELS])
{
    ch[0]  = ( p[0]        | (p[1]  << 8)               ) & 0x7FF;
    ch[1]  = ((p[1]  >> 3) | (p[2]  << 5)               ) & 0x7FF;
    ch[2]  = ((p[2]  >> 6) | (p[3]  << 2) | (p[4]  << 10)) & 0x7FF;
    ch[3]  = ((p[4]  >> 1) | (p[5]  << 7)               ) & 0x7FF;

    ch[4]  = ((p[5]  >> 4) | (p[6]  << 4)               ) & 0x7FF;
    ch[5]  = ((p[6]  >> 7) | (p[7]  << 1) | (p[8]  << 9) ) & 0x7FF;
    ch[6]  = ((p[8]  >> 2) | (p[9]  << 6)               ) & 0x7FF;
    ch[7]  = ((p[9]  >> 5) | (p[10] << 3)               ) & 0x7FF;

    ch[8]  = ( p[11]       | (p[12] << 8)               ) & 0x7FF;
    ch[9]  = ((p[12] >> 3) | (p[13] << 5)               ) & 0x7FF;
    ch[10] = ((p[13] >> 6) | (p[14] << 2) | (p[15] << 10)) & 0x7FF;
    ch[11] = ((p[15] >> 1) | (p[16] << 7)                ) & 0x07FF;

    ch[12] = ((p[16] >> 4) | (p[17] << 4)                ) & 0x07FF;
    ch[13] = ((p[17] >> 7) | (p[18] << 1) | (p[19] << 9) ) & 0x07FF;
    ch[14] = ((p[19] >> 2) | (p[20] << 6)                ) & 0x07FF;
    ch[15] = ((p[20] >> 5) | (p[21] << 3)                ) & 0x07FF;
}

/* ----------------------------------------------------------------
 * 一帧 CRC 通过后的处理 (运行在 UART5 中断上下文, 优先级最高,
 * 不会被其它中断打断, 可直接写 channels 无需关中断)
 * ---------------------------------------------------------------- */
static void crsf_handle_frame(void)
{
    uint8_t type = rx_buf[0];

    if (type == CRSF_TYPE_RC_CHANNELS && rx_payload_len >= CRSF_RC_PAYLOAD_LEN) {
        uint16_t tmp[CRSF_NUM_CHANNELS];
        crsf_decode_rc(&rx_buf[1], tmp);

        for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; i++)
            channels[i] = tmp[i];

        new_frame = true;
        last_frame_tick = HAL_GetTick();
    }
    /* 其它帧类型 (link stats / GPS / battery 等) 暂不解析,
       但能走到这里说明链路通畅, 由 stats.frame_ok 计数 */
}

/* ----------------------------------------------------------------
 * 状态机: 逐字节喂入 (UART5 中断上下文)
 * ---------------------------------------------------------------- */
static void crsf_on_byte(uint8_t b)
{
    stats.rx_bytes++;

    switch (rx_state) {
    case CRSF_STATE_WAIT_SYNC:
        if (b == CRSF_SYNC)
            rx_state = CRSF_STATE_WAIT_LEN;
        break;

    case CRSF_STATE_WAIT_LEN:
        /* LEN = type(1)+payload+crc(1), 合法范围 [4, CRSF_MAX_FRAME-2] */
        if (b < 4 || b > (CRSF_MAX_FRAME - 2)) {
            stats.frame_len_err++;
            rx_state = CRSF_STATE_WAIT_SYNC;
        } else {
            rx_payload_len = (uint8_t)(b - 2);
            rx_idx = 0;
            rx_state = CRSF_STATE_WAIT_TYPE;
        }
        break;

    case CRSF_STATE_WAIT_TYPE:
        rx_buf[0] = b;                 /* TYPE */
        rx_idx = 1;
        rx_state = (rx_payload_len > 0) ? CRSF_STATE_RECV_PAYLOAD
                                        : CRSF_STATE_RECV_CRC;
        break;

    case CRSF_STATE_RECV_PAYLOAD:
        rx_buf[rx_idx++] = b;          /* payload 字节 */
        if (rx_idx > rx_payload_len)   /* payload 收完 */
            rx_state = CRSF_STATE_RECV_CRC;
        break;

    case CRSF_STATE_RECV_CRC:
        /* CRC 覆盖 TYPE+PAYLOAD = rx_buf[0 .. rx_payload_len] */
        if (b == crsf_crc8(rx_buf, (uint32_t)rx_payload_len + 1)) {
            stats.frame_ok++;
            crsf_handle_frame();
        } else {
            stats.frame_crc_err++;
        }
        rx_state = CRSF_STATE_WAIT_SYNC;
        break;
    }
}

/* ================================================================
 * 公开 API
 * ================================================================ */

void CRSF_Init(void)
{
    for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; i++)
        channels[i] = CRSF_CHANNEL_MID;   /* 失联时安全的中点值 */

    rx_state = CRSF_STATE_WAIT_SYNC;
    rx_idx = 0;
    new_frame = false;
    last_frame_tick = 0;
    memset(&stats, 0, sizeof(stats));

    /* 让 UART5 接收能抢占 TIM1(IMU):
       两者原本都是抢占优先级 0, 互不抢占; TIM1 的 ISR (SPI 读 IMU +
       Mahony 姿态解算) 可能占用数十~上百 us, 会盖住 24us/字节的
       CRSF 字节. 把 TIM1 抬到优先级 1, UART5 保持 0 即可打断它. */
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 1, 0);

    /* 启用 UART5 RX FIFO (8 字节深, ~190us 缓冲), 阈值 1/8 = 每字节
       触发. 失败则退化为单字节缓冲, 仍可工作. */
    HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_EnableFifoMode(&huart5);

    /* 清残留错误标志, 使能 RXNE 中断并启动接收 */
    huart5.Instance->ICR = 0xFFFFFFFFU;
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
}

void CRSF_UART5_IRQHandler(void)
{
    USART_TypeDef *u = huart5.Instance;

    /* 清除 ORE/FE/NE/PE 等错误标志 (H7 上 ORE 会阻塞接收) */
    if (u->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE))
        u->ICR = 0xFFFFFFFFU;

    /* 读出 FIFO 内所有可用字节并喂状态机
       (H7 CMSIS: bit5 在非 FIFO 模式=RXNE, FIFO 模式=RXFNE, 组合宏) */
    while (u->ISR & USART_ISR_RXNE_RXFNE)
        crsf_on_byte((uint8_t)(u->RDR & 0xFF));
}

bool CRSF_IsNewFrame(void)
{
    if (new_frame) {
        new_frame = false;
        return true;
    }
    return false;
}

bool CRSF_IsLinkUp(void)
{
    if (last_frame_tick == 0U)
    {
        return false;
    }
    return (HAL_GetTick() - last_frame_tick) < CRSF_LINK_TIMEOUT_MS;
}

uint16_t CRSF_GetChannel(uint8_t ch)
{
    if (ch >= CRSF_NUM_CHANNELS)
        return CRSF_CHANNEL_MID;
    return channels[ch];
}

void CRSF_GetChannels(uint16_t buf[CRSF_NUM_CHANNELS])
{
    /* 关中断拷贝, 防止被 UART5 中断打断读到半新半旧数据 */
    __disable_irq();
    for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; i++)
        buf[i] = channels[i];
    __enable_irq();
}

uint32_t CRSF_GetLastFrameTick(void)
{
    return last_frame_tick;
}

const CRSF_Stats *CRSF_GetStats(void)
{
    return &stats;
}

uint16_t CRSF_MapRawToUs(uint16_t raw)
{
    if (raw < CRSF_CHANNEL_MIN)
    {
        raw = CRSF_CHANNEL_MIN;
    }

    if (raw > CRSF_CHANNEL_MAX)
    {
        raw = CRSF_CHANNEL_MAX;
    }

    return (uint16_t)(1000U+((uint32_t)(raw - CRSF_CHANNEL_MIN) * 1000U) / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN));
}
