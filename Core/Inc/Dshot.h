#ifndef ZEROTYPE_DSHOT_H
#define ZEROTYPE_DSHOT_H

#include <stdint.h>

/* ── Dshot150 位波形（TIM8: 3MHz tick, ARR=19 → 6.67µs/bit） ── */
#define DSHOT_BIT_0       7    /* 35% 占空比 → 电调判 0 */
#define DSHOT_BIT_1      15    /* 75% 占空比 → 电调判 1 */
#define DSHOT_FRAME_LEN  18    /* 16 数据位 + 2 帧尾空闲位 */

/* ── 油门范围 ── */
#define DSHOT_THROTTLE_MIN   48
#define DSHOT_THROTTLE_MAX   2047

/* ── 初始化：开 DMA、发首帧 0 给电调建立同步 ── */
void Dshot_Init(void);

/* ── 发送一帧 ──
   motor     : 0~3 对应 TIM8 CH1~CH4 (PC6~PC9)
   throttle  : 0=停转, 48~2047=正常转速
   telemetry : 0=不询问, 1=请求电调回传数据
   ★ 同一电机上一帧没发完时直接 return，不阻塞 */
void Dshot_Write(uint8_t motor, uint16_t throttle, uint8_t telemetry);

/* ── 查询指定电机 DMA 是否空闲 ── */
uint8_t Dshot_Ready(uint8_t motor);

#endif
