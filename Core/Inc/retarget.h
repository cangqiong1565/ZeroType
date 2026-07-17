#ifndef ZEROTYPE_RETARGET_H
#define ZEROTYPE_RETARGET_H
#include "stdint.h"

/* 在 USB_Task 中周期调用，负责把日志 ring 里的数据真正发到 CDC。 */
void Betaflight_USB_Server(void);

/* 兼容旧接口：写入一条日志，自动补 CRLF，不直接阻塞 USB。 */
void safe_usb_print(const char *str);

/* printf 风格日志接口，同样只是写入日志 ring。 */
void usb_log_printf(const char *fmt, ...);

/* 返回因为 ring 满或发送失败丢弃的字节数。 */
uint32_t usb_log_dropped_count(void);

/* CDC 收到的数据先进入 RX ring，业务任务再从这里逐字节取出解析。 */
void usb_rx_push_data(const uint8_t *data, uint32_t len);
uint8_t usb_rx_get_byte(uint8_t *ch);

/* 兼容旧接口：把 Buf/Len 入队，不忙等 CDC。 */
uint8_t Safe_USB_Transmit(uint8_t *Buf, uint16_t Len);
#endif //ZEROTYPE_RETARGET_H
