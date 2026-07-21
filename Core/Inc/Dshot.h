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

void Dshot_Init(void);
uint8_t Dshot_WriteAll(uint16_t m0, uint16_t m1, uint16_t m2, uint16_t m3);
uint8_t Dshot_WaitDmaDone(uint32_t timeout_ticks);
uint8_t Dshot_Ready(void);

#endif
