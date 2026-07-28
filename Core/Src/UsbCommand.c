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

    if (cmd == NULL)
    {
        return;
    }

    /*
     * 串口工具有时会在命令前带空格或 tab。
     * 先跳过前导空白，让 " dir0 n" 也能被识别成 "dir0 n"。
     */
    while ((*cmd == ' ') || (*cmd == '\t'))
    {
        cmd++;
    }

    if (*cmd == '\0')
    {
        return;
    }

    if (UsbCmdEqual(cmd, "help") || UsbCmdEqual(cmd, "?"))
    {
        usb_log_printf("Commands: stop, m0 100, dir0 n, dir0 r, cmd0 7, cmd0 7 s, status");
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
     * escdir
     *
     * Props Out 最终 ESC 方向写入命令：
     * m0 r
     * m1 r
     * m2 r
     * m3 n
     *
     * 这个命令需要 ESC 上电才能生效。
     * 执行时不要装桨。
     */
    if (UsbCmdEqual(cmd, "escdir"))
    {
        Motor_RequestEscDirectionFix();
        usb_log_printf("ESC DIR: props-out m0:r m1:r m2:r m3:n saving");
        return;
    }

    /*
     * dir0 n / dir0 r
     * dir1 n / dir1 r
     * dir2 n / dir2 r
     * dir3 n / dir3 r
     *
     * 设置单个 ESC 的绝对方向：
     * n = normal
     * r = reversed
     *
     * 注意这不是“翻转一次”，而是写入一个确定方向。
     */
    if (((cmd[0] == 'd') || (cmd[0] == 'D')) &&
        ((cmd[1] == 'i') || (cmd[1] == 'I')) &&
        ((cmd[2] == 'r') || (cmd[2] == 'R')) &&
        (cmd[3] >= '0') && (cmd[3] <= '3') &&
        (cmd[4] == ' '))
    {
        uint8_t motor_index = (uint8_t)(cmd[3] - '0');
        char dir = cmd[5];

        if ((dir >= 'A') && (dir <= 'Z'))
        {
            dir = (char)(dir - 'A' + 'a');
        }

        if ((dir != 'n') && (dir != 'r'))
        {
            usb_log_printf("Use: dir0 n or dir0 r");
            return;
        }

        Motor_RequestEscDirection(motor_index, dir == 'r');
        usb_log_printf("DIR%u:%c saving",
                       (unsigned)motor_index,
                       dir);
        return;
    }

    /*
     * cmd0 7
     * cmd0 8
     * cmd0 20
     * cmd0 21
     * cmd0 21 s
     *
     * 给单个 ESC 发送原始 DShot 特殊命令。
     * 末尾带 s 表示命令后自动发送 SAVE_SETTINGS。
     */
    if (((cmd[0] == 'c') || (cmd[0] == 'C')) &&
        ((cmd[1] == 'm') || (cmd[1] == 'M')) &&
        ((cmd[2] == 'd') || (cmd[2] == 'D')) &&
        (cmd[3] >= '0') && (cmd[3] <= '3') &&
        (cmd[4] == ' '))
    {
        uint8_t motor_index = (uint8_t)(cmd[3] - '0');
        bool save_after = false;

        if (!UsbCmdParseUint16(&cmd[5], &value))
        {
            /*
             * 支持 "cmd0 21 s" 这种形式。
             * 这里手动解析前面的数字和末尾 s。
             */
            const char *p = &cmd[5];
            uint32_t result = 0U;
            bool has_digit = false;

            while ((*p >= '0') && (*p <= '9'))
            {
                has_digit = true;
                result = result * 10U + (uint32_t)(*p - '0');
                p++;
            }

            while ((*p == ' ') || (*p == '\t'))
            {
                p++;
            }

            if (((*p == 's') || (*p == 'S')) && (*(p + 1) == '\0') && has_digit && (result <= 65535U))
            {
                value = (uint16_t)result;
                save_after = true;
            }
            else
            {
                usb_log_printf("Use: cmd0 21 or cmd0 21 s");
                return;
            }
        }

        if (value > 47U)
        {
            usb_log_printf("DShot cmd must be 0..47");
            return;
        }

        Motor_RequestEscRawCommand(motor_index, value, save_after);
        usb_log_printf("CMD%u:%u%s",
                       (unsigned)motor_index,
                       (unsigned)value,
                       save_after ? " save" : "");
        return;
    }

    /*
     * m0 100
     * m1 100
     * m2 100
     * m3 100
     *
     * 单电机编号测试命令。
     * 这里的 100 是 DShot 值，不是遥控器 1000..2000 油门值。
     * 每次命令只让一个电机转，其它三个强制输出 0。
     */
    if (((cmd[0] == 'm') || (cmd[0] == 'M')) &&
        (cmd[1] >= '0') && (cmd[1] <= '3') &&
        (cmd[2] == ' '))
    {
        uint8_t motor_index = (uint8_t)(cmd[1] - '0');

        if (!UsbCmdParseUint16(&cmd[3], &value))
        {
            usb_log_printf("Use: m0 100");
            return;
        }

        Motor_SetSingleTestOutput(motor_index, value);
        usb_log_printf("M%u:%u", (unsigned)motor_index, (unsigned)value);
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
