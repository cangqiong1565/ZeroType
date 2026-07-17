#include "retarget.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>

#include "Task.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

#define USB_LOG_RING_SIZE 1024U
#define USB_LOG_TX_CHUNK  64U
#define USB_RX_RING_SIZE  128U

/*
 * USB 日志层的核心设计：
 *
 * 1. 其他任务只把字符串写进 usb_log_ring，不直接碰 CDC_Transmit_FS。
 *    这样高优先级任务不会因为 USB busy 被卡住。
 *
 * 2. USB_Task 周期性调用 Betaflight_USB_Server，把 ring 里的数据搬到
 *    tx_buffer，再交给 CDC_Transmit_FS。
 *
 * 3. 当前链接脚本把普通 .bss 放在 DTCM。STM32H7 的 USB 外设不能直接
 *    访问 DTCM，所以日志 ring 固定放在 D2 SRAM 起始地址 0x30000000。
 */
static uint8_t * const usb_log_ring = (uint8_t *)0x30000000U;
static volatile uint32_t usb_log_head = 0;       /* 下一个写入位置 */
static volatile uint32_t usb_log_tail = 0;       /* 下一个读出位置 */
static volatile uint32_t usb_log_dropped = 0;    /* ring 满或发送失败时丢弃的字节数 */

/*
 * USB RX ring 只保存串口命令，数据量很小。
 *
 * producer 是 CDC_Receive_FS，consumer 是 USB_Task。一个写、一个读时，
 * head/tail 的 32 位读写在 Cortex-M 上是原子的，所以这里不需要让 USB
 * 中断等待任务临界区。
 */
static uint8_t usb_rx_ring[USB_RX_RING_SIZE];
static volatile uint32_t usb_rx_head = 0;        /* CDC 回调写入位置 */
static volatile uint32_t usb_rx_tail = 0;        /* USB_Task 读出位置 */
static volatile uint32_t usb_rx_dropped = 0;     /* RX ring 满时丢弃的字节数 */

static void usb_log_push_byte(uint8_t ch)
{
    /* ring 预留一个空位，用 head+1 == tail 表示满。 */
    uint32_t next_head = (usb_log_head + 1U) % USB_LOG_RING_SIZE;

    if (next_head == usb_log_tail)
    {
        /* 满了就丢弃，日志不能反过来阻塞飞控主流程。 */
        usb_log_dropped++;
        return;
    }

    usb_log_ring[usb_log_head] = ch;
    usb_log_head = next_head;
}

static void usb_log_write_raw(const char *str)
{
    char prev = '\0';

    if (str == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();

    while (*str != '\0')
    {
        /*
         * 终端通常更喜欢 CRLF。调用者如果只写 '\n'，这里自动补 '\r'。
         * 如果调用者已经写了 "\r\n"，就不重复补。
         */
        if ((*str == '\n') && (prev != '\r'))
        {
            usb_log_push_byte('\r');
        }

        usb_log_push_byte((uint8_t)*str);
        prev = *str;
        str++;
    }

    taskEXIT_CRITICAL();
}

/*
 * 兼容旧接口：现有代码仍然可以调用 safe_usb_print。
 *
 * 这里把“一次调用”视为“一条日志”。如果字符串末尾没有换行，就自动追加
 * CRLF，避免快速连续打印时多条日志粘在同一行。
 */
void safe_usb_print(const char *str)
{
    const char *p;
    char last = '\0';

    if (str == NULL)
    {
        return;
    }

    usb_log_write_raw(str);

    for (p = str; *p != '\0'; p++)
    {
        last = *p;
    }

    if ((last != '\n') && (last != '\r'))
    {
        usb_log_write_raw("\r\n");
    }
}

void usb_log_printf(const char *fmt, ...)
{
    char buffer[192];
    va_list args;

    if (fmt == NULL)
    {
        return;
    }

    va_start(args, fmt);
    /* 先格式化到栈上小缓冲区，再按一整行写入 USB 日志 ring。 */
    (void)vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    safe_usb_print(buffer);
}

void Betaflight_USB_Server(void)
{
    /*
     * USB CDC 发送缓冲区。
     *
     * Cortex-M7 开 DCache 时，SCB_CleanDCache_by_Addr 要求地址按 cache line
     * 对齐更稳妥，所以这里 32 字节对齐。
     */
    static uint8_t tx_buffer[USB_LOG_TX_CHUNK] __attribute__((aligned(32)));
    uint32_t len = 0;
    uint32_t cache_len;
    USBD_CDC_HandleTypeDef *hcdc;

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return;
    }

    /* CDC 上一包还没发完就直接返回，不忙等。 */
    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if ((hcdc == NULL) || (hcdc->TxState != 0U))
    {
        return;
    }

    taskENTER_CRITICAL();

    /*
     * 从 ring 搬一小段到 tx_buffer。
     * 只在 USB_Task 里调用本函数，但 producer 可能是 IMU_Task 等其他任务，
     * 所以读 tail/写 head 的共享区仍然用临界区保护。
     */
    while ((usb_log_head != usb_log_tail) && (len < USB_LOG_TX_CHUNK))
    {
        tx_buffer[len] = usb_log_ring[usb_log_tail];
        usb_log_tail = (usb_log_tail + 1U) % USB_LOG_RING_SIZE;
        len++;
    }

    taskEXIT_CRITICAL();

    if (len == 0U)
    {
        return;
    }

    /* DCache clean 长度也按 32 字节向上取整。 */
    cache_len = (len + 31U) & ~31U;
    SCB_CleanDCache_by_Addr((uint32_t *)tx_buffer, (int32_t)cache_len);

    if (CDC_Transmit_FS(tx_buffer, (uint16_t)len) != USBD_OK)
    {
        usb_log_dropped += len;
    }
}

uint32_t usb_log_dropped_count(void)
{
    /* 供 USB_Task 或调试命令打印日志丢弃统计。 */
    return usb_log_dropped;
}

void usb_rx_push_data(const uint8_t *data, uint32_t len)
{
    uint32_t i;

    if (data == NULL)
    {
        return;
    }

    for (i = 0; i < len; i++)
    {
        uint32_t next_head = (usb_rx_head + 1U) % USB_RX_RING_SIZE;

        if (next_head == usb_rx_tail)
        {
            usb_rx_dropped++;
            continue;
        }

        usb_rx_ring[usb_rx_head] = data[i];
        usb_rx_head = next_head;
    }
}

uint8_t usb_rx_get_byte(uint8_t *ch)
{
    if ((ch == NULL) || (usb_rx_head == usb_rx_tail))
    {
        return 0U;
    }

    *ch = usb_rx_ring[usb_rx_tail];
    usb_rx_tail = (usb_rx_tail + 1U) % USB_RX_RING_SIZE;

    return 1U;
}

uint8_t Safe_USB_Transmit(uint8_t *Buf, uint16_t Len)
{
    uint16_t i;

    if (Buf == NULL)
    {
        return USBD_FAIL;
    }

    taskENTER_CRITICAL();

    /*
     * 兼容旧接口：不再直接阻塞等待 CDC，而是把外部给的 buffer 入队。
     * 这样调用者不会因为 USB busy 卡住。
     */
    for (i = 0; i < Len; i++)
    {
        usb_log_push_byte(Buf[i]);
    }

    taskEXIT_CRITICAL();

    return USBD_OK;
}
