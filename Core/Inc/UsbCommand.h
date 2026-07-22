#ifndef ZEROTYPE_USBCOMMAND_H
#define ZEROTYPE_USBCOMMAND_H

/*
 * USB 命令层。
 *
 * 这个模块只负责：
 * - 从 USB RX ring 里取字符；
 * - 拼成一行文本命令；
 * - 解析 arm / disarm / thr / status；
 * - 调用 Motor_* 接口。
 *
 * 它不直接发送 DShot，也不直接操作 TIM/DMA。
 */
void UsbCommand_ProcessRx(void);

#endif // ZEROTYPE_USBCOMMAND_H
