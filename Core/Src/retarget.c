#include "retarget.h"
#include "usbd_cdc_if.h"
#include "Task.h"
#include "ring_buffer.h" // 引入你写好的环形缓冲区

extern USBD_HandleTypeDef hUsbDeviceFS;

// 1. 定义一个专门给 USB 异步吐数据的环形缓冲区
// 我们把这个缓冲区数组定在安全的 D2 域 SRAM1 首地址
#define USB_RING_BUFFER_SIZE 256
static uint8_t *usb_ring_mem = (uint8_t *)0x30000000;
static uint32_t usb_rb_head = 0;
static uint32_t usb_rb_tail = 0;

// 2. 供 IMU_Task 调用的非阻塞打印：只管往缓冲区塞，塞完立刻回航！
void safe_usb_print(const char *str)
{
    // 如果 USB 没好，连缓冲区都不用塞，直接撤退
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return;
    }

    int i = 0;

    while (str[i] != '\0')
    {
        uint32_t next_head = (usb_rb_head + 1) % USB_RING_BUFFER_SIZE;

        // 如果缓冲区满了，后面的数据直接丢弃，绝对不锁死任务！
        if (next_head == usb_rb_tail) {

            break;
        }

        usb_ring_mem[usb_rb_head] = str[i];
        usb_rb_head = next_head;
        i++;
    }

}

// 3. 【仿 BF MSP 核心发送器】：这个函数专门放在最低优先级的 USB_Task 里跑！
// 只有它才有资格和官方的 CDC_Transmit_FS 碰一碰
void Betaflight_USB_Server(void)
{
    // 如果电脑没准备好，直接跳过
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        return;
    }

    // 检查官方库硬件的状态，如果硬件还在忙，这一轮就不发了，立刻退出！
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc == NULL || hcdc->TxState != 0) {
        return;
    }

    // 检查我们的缓冲区里有没有高优先级任务刚才丢进来的数据？
    if (usb_rb_head != usb_rb_tail)
    {
        // 临时把要发的数据提取出来（为了迎合 DMA，每次最多发 32 字节，细水长流）
        static uint8_t local_tx_buf[32];
        int len = 0;
        while (usb_rb_head != usb_rb_tail && len < 32)
        {
            local_tx_buf[len] = usb_ring_mem[usb_rb_tail];
            usb_rb_tail = (usb_rb_tail + 1) % USB_RING_BUFFER_SIZE;
            len++;
        }

        if (len > 0)
        {
            // 同步一下 D-Cache
            SCB_CleanDCache_by_Addr((uint32_t *)local_tx_buf, 32);
            // 发送！此时硬件 100% 是 IDLE 的，一发即中
            CDC_Transmit_FS(local_tx_buf, len);
        }
    }
}

uint8_t Safe_USB_Transmit(uint8_t* Buf, uint16_t Len)
{
    uint8_t result = USBD_OK;
    uint32_t timeout = HAL_GetTick();

    // 获取当前 USB 状态
    extern USBD_HandleTypeDef hUsbDeviceFS;
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

    // 如果 USB 没插上，或者没初始化好，直接跳过，不卡主循环
    if (hcdc == NULL) return USBD_FAIL;

    // 如果忙碌，最多等 2ms (对于 480MHz 足够了)，等不到就强行退出，防死机
    while (hcdc->TxState != 0)
    {
        if (HAL_GetTick() - timeout > 2) {
            return USBD_BUSY; // 强行丢包保命
        }
    }

    // 状态空闲，发送数据
    result = CDC_Transmit_FS(Buf, Len);
    return result;
}