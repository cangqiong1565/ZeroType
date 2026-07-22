#include "UsbCommand.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "Motor.h"
#include "retarget.h"

static bool UsbCmdEqual(const char *cmd, const char *target)
{
    /*
     * 简单的大小写不敏感字符串比较。
     *
     * 这里不用标准库 strcasecmp，是为了避免不同嵌入式工具链支持不一致。
     */
    while ((*cmd != '\0') && (*target != '\0'))
    {
        char a = *cmd;
        char b = *target;

        /*
         * 如果当前字符是大写字母，就转换成小写。
         */
        if ((a >= 'A') && (a <= 'Z'))
        {
            a = (char)(a - 'A' + 'a');
        }

        /*
         * target 理论上都写小写，但这里也转换一次，函数更通用。
         */
        if ((b >= 'A') && (b <= 'Z'))
        {
            b = (char)(b - 'A' + 'a');
        }

        if (a != b)
        {
            return false;
        }

        cmd++;
        target++;
    }

    /*
     * 两个字符串必须同时结束，才算完全相等。
     * 例如 "arm" 和 "arming" 不能算相等。
     */
    return (*cmd == '\0') && (*target == '\0');
}

static bool UsbCmdParseUint16(const char *cmd, uint16_t *value)
{
    uint32_t result = 0U;
    bool has_digit = false;

    if ((cmd == NULL) || (value == NULL))
    {
        return false;
    }

    /*
     * 跳过数字前面的空格和 tab。
     */
    while ((*cmd == ' ') || (*cmd == '\t'))
    {
        cmd++;
    }

    /*
     * 逐字符解析十进制数字。
     */
    while ((*cmd >= '0') && (*cmd <= '9'))
    {
        has_digit = true;
        result = result * 10U + (uint32_t)(*cmd - '0');

        /*
         * 这个函数返回 uint16_t，超过 65535 就算非法。
         */
        if (result > 65535U)
        {
            return false;
        }

        cmd++;
    }

    /*
     * 允许数字后面带一点空格。
     */
    while ((*cmd == ' ') || (*cmd == '\t'))
    {
        cmd++;
    }

    /*
     * 必须至少解析到一个数字，并且后面不能还有其他字符。
     */
    if ((!has_digit) || (*cmd != '\0'))
    {
        return false;
    }

    *value = (uint16_t)result;
    return true;
}

static void UsbCommand_HandleLine(const char *cmd)
{
    uint16_t value;
    MotorStatus_t status;

    if ((cmd == NULL) || (*cmd == '\0'))
    {
        return;
    }

    if (UsbCmdEqual(cmd, "help") || UsbCmdEqual(cmd, "?"))
    {
        usb_log_printf("Commands: arm, disarm, stop, thr 1000, status");
        return;
    }

    if (UsbCmdEqual(cmd, "arm"))
    {
        Motor_Arm();
        usb_log_printf("ARMED");
        return;
    }

    if (UsbCmdEqual(cmd, "stop") || UsbCmdEqual(cmd, "disarm"))
    {
        Motor_Disarm();
        usb_log_printf("DISARMED");
        return;
    }

    /*
     * thr 1000
     * thr 1100
     *
     * 这里设置的是遥控意义上的油门值，不是 DShot 值。
     */
    if ((cmd[0] == 't') && (cmd[1] == 'h') && (cmd[2] == 'r') && (cmd[3] == ' '))
    {
        if (!UsbCmdParseUint16(&cmd[4], &value))
        {
            usb_log_printf("Use: thr 1000");
            return;
        }

        Motor_SetRcThrottle(value);
        usb_log_printf("THR:%u", (unsigned)value);
        return;
    }

    if (UsbCmdEqual(cmd, "status"))
    {
        Motor_GetStatus(&status);

        usb_log_printf("STATE:%u THR:%u M:%u %u %u %u",
                       (unsigned)status.state,
                       (unsigned)status.rc_throttle_us,
                       (unsigned)status.output[0],
                       (unsigned)status.output[1],
                       (unsigned)status.output[2],
                       (unsigned)status.output[3]);
        return;
    }

    /*
     * 兼容一个短命令：
     * 直接发 "1100" 等价于 "thr 1100"。
     */
    if (UsbCmdParseUint16(cmd, &value))
    {
        Motor_SetRcThrottle(value);
        usb_log_printf("THR:%u", (unsigned)value);
        return;
    }

    usb_log_printf("Bad command: %s", cmd);
}

void UsbCommand_ProcessRx(void)
{
    static char line[24];
    static uint8_t line_len = 0U;
    uint8_t ch;

    /*
     * 一次把 RX ring 里已有的字符都取出来。
     * usb_rx_get_byte() 的生产者是 USB CDC 接收回调，消费者就是这里。
     */
    while (usb_rx_get_byte(&ch) != 0U)
    {
        /*
         * 回车或换行表示一条命令结束。
         */
        if ((ch == '\r') || (ch == '\n'))
        {
            if (line_len > 0U)
            {
                line[line_len] = '\0';
                UsbCommand_HandleLine(line);
                line_len = 0U;
            }
        }
        /*
         * 支持退格和 DEL，方便串口工具里手动输入命令时改错。
         */
        else if ((ch == '\b') || (ch == 0x7FU))
        {
            if (line_len > 0U)
            {
                line_len--;
            }
        }
        /*
         * 只接收普通可打印 ASCII 字符。
         */
        else if ((ch >= 32U) && (ch <= 126U))
        {
            if (line_len < (sizeof(line) - 1U))
            {
                line[line_len] = (char)ch;
                line_len++;
            }
            else
            {
                /*
                 * 命令太长时丢弃当前行，防止缓冲区越界。
                 */
                line_len = 0U;
                usb_log_printf("Command too long");
            }
        }
    }
}
