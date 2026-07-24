/**
 * @file debug.c
 * @brief Unified UART and OLED debug output for car tests.
 */

#include "debug.h"
#include "bsp_uart.h"
#include "oled.h"
#include <stdio.h>
#include <string.h>

#define DEBUG_OLED_W       128U
#define DEBUG_OLED_ROW_LEN 16U

extern u8 OLED_GRAM[144][8];

/**
 * @brief Clear one 16px text row in OLED GRAM.
 * @param y Row start y coordinate.
 */
static void debug_clear_row(u8 y)
{
    OLED_ShowString(0, y, (u8 *)"                ", 16, 1);
}

/**
 * @brief Refresh only selected OLED pages instead of the full screen.
 * @param page First page index.
 * @param cnt Page count.
 */
static void debug_refresh_pages(u8 page, u8 cnt)
{
    u8 p;
    u8 x;

    for (p = page; (p < 8U) && (p < (u8)(page + cnt)); p++) {
        OLED_WR_Byte((u8)(0xB0U + p), OLED_CMD);
        OLED_WR_Byte(0x00, OLED_CMD);
        OLED_WR_Byte(0x10, OLED_CMD);

        for (x = 0U; x < DEBUG_OLED_W; x++) {
            OLED_WR_Byte(OLED_GRAM[x][p], OLED_DATA);
        }
    }
}

/**
 * @brief Draw one fixed-width debug row to OLED GRAM.
 * @param y Row start y coordinate.
 * @param str ASCII text, clipped by caller buffer length.
 */
static void debug_show_row(u8 y, const char *str)
{
    debug_clear_row(y);
    OLED_ShowString(0, y, (u8 *)str, 16, 1);
}

/**
 * @brief Send one wheel speed line to UART.
 * @param actual Actual encoder speed.
 * @param target Target wheel speed.
 */
static void debug_uart_speed(int actual, int target)
{
    char buf[64];
    int len;

    len = snprintf(buf, sizeof(buf),
                   "Actual Speed,Target Speed: %d, %d\r\n",
                   actual,
                   target);
    if (len > 0) {
        (void)BSP_Uart_Write((uint8_t const *)buf, (uint16_t)strlen(buf));
    }
}

void Debug_Output(const Car_Status *st)
{
    Car_PidParam pid;
    char row[DEBUG_OLED_ROW_LEN + 1U];

    if (st == NULL) {
        return;
    }

    // 当前模式对应的 PID 参数从底盘状态层统一读取。
    Car_GetPidParam(&pid);

    /* UART order is left wheel first, right wheel second. */
    debug_uart_speed(st->speed_l, st->target_l);
    debug_uart_speed(st->speed_r, st->target_r);

    (void)snprintf(row, sizeof(row), "PID:%.2g %.2g %.2g",
                   (double)pid.kp,
                   (double)pid.ki,
                   (double)pid.kd);
    debug_show_row(0, row);

    (void)snprintf(row, sizeof(row), "pos:%+06d", st->track_pos);
    debug_show_row(16, row);

    /* Car status stores left=E2 and right=E1, so display order is E1 then E2. */
    (void)snprintf(row, sizeof(row), "E1 E2:%+4d %+4d",
                   st->speed_r,
                   st->speed_l);
    debug_show_row(32, row);

    // 只刷新已写入的页面，减少整屏刷新的等待。
    debug_refresh_pages(0U, 6U);
}
