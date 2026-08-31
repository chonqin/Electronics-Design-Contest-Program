/**
 * @file debug.c
 * @brief 底盘 UART 调试输出
 */
#include "debug.h"
#include "bsp_uart.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define DEBUG_CHANNEL_COUNT 16U
#define DEBUG_CMD_BUF_SIZE  24U
#define DEBUG_GAIN_MAX      100.0f

static char cmd_buf[DEBUG_CMD_BUF_SIZE];
static uint8_t cmd_len;
static uint8_t cmd_drop;
static int8_t cmd_status;
static float task1_time_s;

void Debug_SetTask1Time(uint32_t ms)
{
    task1_time_s = (float)ms / 1000.0f;
}

/**
 * @brief 解析并执行一条紧凑 PID 调参命令
 * @param line 以空字符结尾的命令
 * @return 1 表示成功，-1 表示格式错误，-2 表示数值越界
 * @note 命令格式：TKP/TKI/TKD/YKP/YKI/YKD 后接操作符和非负数值。
 *       '=' 设置绝对值，'+' 在当前值上增加，'-' 在当前值上减少。
 *       示例：TKP=0.18、YKD+0.10、YKI-0.05。
 */
static int debug_parse_pid(char const *line)
{
    char *end;
    char op;
    float value;
    float next;
    Car_PidId id;
    Car_PidTerm term;
    Car_PidParams param;

    if (strlen(line) < 5U || line[1] != 'K') {
        return -1;
    }

    if (line[0] == 'T') {
        id = CAR_PID_TRACK;
    } else if (line[0] == 'Y') {
        id = CAR_PID_YAW;
    } else {
        return -1;
    }

    if (line[2] == 'P') {
        term = CAR_PID_KP;
    } else if (line[2] == 'I') {
        term = CAR_PID_KI;
    } else if (line[2] == 'D') {
        term = CAR_PID_KD;
    } else {
        return -1;
    }

    op = line[3];
    if (op != '=' && op != '+' && op != '-') {
        return -1;
    }

    value = strtof(&line[4], &end);
    if (end == &line[4] || *end != '\0') {
        return -1;
    }
    if (!isfinite(value) || value < 0.0f) {
        return -2;
    }

    if (op == '=') {
        next = value;
    } else {
        Car_GetPidParams(id, &param);
        if (term == CAR_PID_KP) {
            next = param.kp;
        } else if (term == CAR_PID_KI) {
            next = param.ki;
        } else {
            next = param.kd;
        }
        if (op == '+') {
            next += value;
        } else {
            next -= value;
        }
    }

    if (!isfinite(next) || next < 0.0f || next > DEBUG_GAIN_MAX) {
        return -2;
    }

    if (Car_SetPidParam(id, term, next) != 0U) {
        return 1;
    }

    return -1;
}

static void debug_parse_line(void)
{
    int result;

    if (cmd_drop != 0U) {
        cmd_status = -3;
        return;
    }

    cmd_buf[cmd_len] = '\0';
    if (cmd_len == 0U) {
        return;
    }

    if (strcmp(cmd_buf, "STOP") == 0) {
        Car_Stop();
        cmd_status = 1;
        return;
    }

    result = debug_parse_pid(cmd_buf);
    cmd_status = (int8_t)result;
}

void Debug_Output(void)
{
    Car_Status st;
    Car_PidParams track;
    Car_PidParams yaw;
    float frame[DEBUG_CHANNEL_COUNT + 1U];
    uint32_t tail = 0x7F800000U;

    Car_GetStatus(&st);
    Car_GetPidParams(CAR_PID_TRACK, &track);
    Car_GetPidParams(CAR_PID_YAW, &yaw);

    /* 通道顺序固定，末通道为任务 1 从确认按键开始计算的用时。 */
    frame[0] = (float)st.track_pos;
    frame[1] = (float)st.enc_l;
    frame[2] = (float)st.enc_r;
    frame[3] = (float)st.duty_l;
    frame[4] = (float)st.duty_r;
    frame[5] = st.yaw;
    frame[6] = track.kp;
    frame[7] = track.ki;
    frame[8] = track.kd;
    frame[9] = yaw.kp;
    frame[10] = yaw.ki;
    frame[11] = yaw.kd;
    frame[12] = (float)st.track_mask;
    frame[13] = (float)st.mode;
    frame[14] = (float)cmd_status;
    frame[15] = task1_time_s;
    memcpy(&frame[DEBUG_CHANNEL_COUNT], &tail, sizeof(tail));

    (void)BSP_Uart_Write((uint8_t const *)frame, sizeof(frame));
}

void Debug_CommandPoll(void)
{
    uint8_t dat;
    uint16_t budget = 128U;

    while (budget > 0U && BSP_Uart_ReadByte(&dat) != 0) {
        budget--;

        if (dat == '\r') {
            continue;
        }

        /* 上行格式：TKP=0.18、TKP+0.01、TKP-0.01 或 STOP，以换行结束。 */
        if (dat == '\n') {
            debug_parse_line();
            cmd_len = 0U;
            cmd_drop = 0U;
            continue;
        }

        if (cmd_drop != 0U) {
            continue;
        }

        if (cmd_len >= (DEBUG_CMD_BUF_SIZE - 1U)) {
            cmd_drop = 1U;
            continue;
        }

        cmd_buf[cmd_len++] = (char)dat;
    }
}
