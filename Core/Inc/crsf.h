#ifndef CRSF_H
#define CRSF_H
#include <stdint.h>
#include <stdbool.h>

/* ================================================================
 * CRSF (Crossfire / ExpressLRS) RC 接收协议
 *
 * 物理层: 420000 baud, 8N1, 普通 (非反相) UART 电平, 适配 ELRS
 *   接收机的默认输出. 若接收机输出反相信号, 需在 UART5 的
 *   AdvancedInit 里使能 RXINV (见 crsf.c 注释).
 *
 * 帧格式: [SYNC=0xC8][LEN][TYPE][PAYLOAD...][CRC8]
 *   LEN  = TYPE(1) + payload + CRC(1), 不含 SYNC 和 LEN 本身
 *   CRC8 = poly 0xD5, init 0x00, 覆盖 TYPE+PAYLOAD
 *   RC帧 = TYPE 0x16, payload 22B = 11 通道 x 11 bit
 * ================================================================ */

#define CRSF_SYNC                 0xC8   /* 帧同步字节 */

/* 帧类型 */
#define CRSF_TYPE_RC_CHANNELS     0x16
#define CRSF_TYPE_LINK_STATISTICS 0x14

/* 通道 */
#define CRSF_NUM_CHANNELS         16     /* 11 通道 x 11 bit */
#define CRSF_RC_PAYLOAD_LEN       22     /* RC 帧负载字节数 */

/* 通道值范围 (Betaflight/ELRS 约定), 中点 992 */
#define CRSF_CHANNEL_MIN          172
#define CRSF_CHANNEL_MID          992
#define CRSF_CHANNEL_MAX          1811

#define CRSF_MAX_FRAME            64     /* 最大帧长 (含 sync/len/type/payload/crc) */

/* 链路超时: 超过此时间未收到 RC 帧视为失联 (ms) */
#define CRSF_LINK_TIMEOUT_MS      200U

/* ================================================================
 * API
 * ================================================================ */

/* 初始化并启动 UART5 接收 (RXNE 中断 + 状态机解析) */
void CRSF_Init(void);

/* UART5 中断入口 — 在 UART5_IRQHandler() 中调用, 处理完后 return */
void CRSF_UART5_IRQHandler(void);

/* 是否收到新的 RC 帧. 调用后标志自动清零 (边沿触发) */
bool CRSF_IsNewFrame(void);

/* 链路是否活跃 (CRSF_LINK_TIMEOUT_MS 内收到过 RC 帧) */
bool CRSF_IsLinkUp(void);

/* 读单个通道值 (172~1811, 中点 992). ch: 0~10, 越界返回中点 */
uint16_t CRSF_GetChannel(uint8_t ch);

/* 一次读出全部 11 个通道 (关中断拷贝, 线程安全) */
void CRSF_GetChannels(uint16_t buf[CRSF_NUM_CHANNELS]);

/* 最近一次收到 RC 帧的时刻 (HAL_GetTick, ms) */
uint32_t CRSF_GetLastFrameTick(void);

/* 调试统计 */
typedef struct {
    uint32_t rx_bytes;      /* 收到的总字节数 */
    uint32_t frame_ok;      /* CRC 校验通过的帧数 */
    uint32_t frame_crc_err; /* CRC 校验失败的帧数 */
    uint32_t frame_len_err; /* LEN 字段非法的帧数 */
} CRSF_Stats;
const CRSF_Stats *CRSF_GetStats(void);

uint16_t CRSF_MapRawToUs(uint16_t raw);
#endif /* CRSF_H */
