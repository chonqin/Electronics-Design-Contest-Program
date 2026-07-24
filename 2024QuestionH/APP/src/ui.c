/**
 * @file ui.c
 * @brief OLED UI helpers and task menu implementation.
 */

#include "ui.h"
#include "oled.h"
#include "bsp_key.h"
#include "board.h"
#include <stdio.h>

/** @brief Menu labels aligned with main.c task dispatch order. */
static const char *TASK_NAMES[TASK_COUNT] = {
    "LINE", "PID", "MOTOR", "IMU", "OLED", "UART", "TRACK"
};

/** @brief Number of visible rows when using the 16px font. */
#define MENU_ROWS 4

/**
 * @brief Wait until all keys are released.
 */
static void wait_release(void)
{
    while (Key_Scan() != -1) {
        delay_ms(10);
    }
}

/**
 * @brief Draw the menu cursor on one row.
 * @param y Row origin on OLED.
 * @param active Non-zero shows the cursor marker.
 */
static void show_cursor(u8 y, uint8_t active)
{
    u8 mark = ' ';

    if (active != 0U) {
        mark = '>';
    }

    OLED_ShowChar(0, y, mark, 16, 1);
}

/**
 * @brief Clear one 16px text row.
 * @param y Row origin on OLED.
 */
static void clear_text_row(u8 y)
{
    OLED_ShowString(0, y, (u8 *)"                ", 16, 1);
}

void UI_Init(void)
{
    Key_Init();
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
}

void UI_Test_OLED(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"OLED Test", 16, 1);
    OLED_ShowString(0, 20, (u8 *)"TEST String:", 16, 1);
    OLED_Refresh();
}

void UI_Test_Motor(int duty, int e1, int e2)
{
    char buf[17];

    clear_text_row(0);
    clear_text_row(16);
    clear_text_row(32);
    clear_text_row(48);

    OLED_ShowString(0, 0, (u8 *)"Motor Duty", 16, 1);
    (void)snprintf(buf, sizeof(buf), "Duty:%+5d", duty);
    OLED_ShowString(0, 16, (u8 *)buf, 16, 1);
    (void)snprintf(buf, sizeof(buf), "E1:%+7d", e1);
    OLED_ShowString(0, 32, (u8 *)buf, 16, 1);
    (void)snprintf(buf, sizeof(buf), "E2:%+7d", e2);
    OLED_ShowString(0, 48, (u8 *)buf, 16, 1);
    OLED_Refresh();
}

void UI_Test_PIDSelect(uint8_t left)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"PID Tune", 16, 1);
    OLED_ShowString(0, 16, (u8 *)"Wheel:", 16, 1);
    OLED_ShowString(56, 16, (u8 *)(left ? "Left" : "Right"), 16, 1);
    OLED_ShowString(0, 32, (u8 *)"KEY1/2 Swap", 16, 1);
    OLED_ShowString(0, 48, (u8 *)"KEY3 OK", 16, 1);
    OLED_Refresh();
}

void UI_Test_PID(const char *wheel, int actual, int target, int output)
{
    char buf[24];

    clear_text_row(0);
    clear_text_row(16);
    clear_text_row(32);
    clear_text_row(48);

    OLED_ShowString(0, 0, (u8 *)"PID Tune", 16, 1);
    (void)snprintf(buf, sizeof(buf), "Wheel:%s", wheel);
    OLED_ShowString(0, 16, (u8 *)buf, 16, 1);
    (void)snprintf(buf, sizeof(buf), "A:%+4d T:%+4d", actual, target);
    OLED_ShowString(0, 32, (u8 *)buf, 16, 1);
    (void)snprintf(buf, sizeof(buf), "O:%+5d", output);
    OLED_ShowString(0, 48, (u8 *)buf, 16, 1);
    OLED_Refresh();
}

void UI_IMU_Calibrating(void)
{
    OLED_Clear();
    OLED_ShowString(0, 24, (u8 *)"Calibrating...", 16, 1);
    OLED_Refresh();
}

/**
 * @brief Draw one signed float with fixed width.
 * @param x OLED x coordinate.
 * @param y OLED y coordinate.
 * @param val Value to render.
 */
static void show_float(u8 x, u8 y, float val)
{
    char buf[12];

    (void)snprintf(buf, sizeof(buf), "%+7.2f", (double)val);
    OLED_ShowString(x, y, (u8 *)buf, 16, 1);
}

void UI_Test_IMU(float *angles)
{
    clear_text_row(0);
    OLED_ShowString(0, 0, (u8 *)"P:", 16, 1);
    show_float(16, 0, angles[1]);

    clear_text_row(22);
    OLED_ShowString(0, 22, (u8 *)"R:", 16, 1);
    show_float(16, 22, angles[2]);

    clear_text_row(44);
    OLED_ShowString(0, 44, (u8 *)"Y:", 16, 1);
    show_float(16, 44, angles[0]);

    OLED_Refresh();
}

/**
 * @brief Handle the first-level task list.
 * @return Selected task ID.
 */
static Task_ID UI_Menu_L1(void)
{
    int8_t cur = 0;
    int8_t first = 0;
    int8_t key;

    while (1) {
        if (cur < first) {
            first = cur;
        } else if (cur >= (first + MENU_ROWS)) {
            // 选中项超出可视窗口后，推动顶部索引继续下移。
            first = cur - MENU_ROWS + 1;
        }

        OLED_Clear();
        for (int8_t row = 0; row < MENU_ROWS; row++) {
            int8_t i = first + row;

            if (i >= TASK_COUNT) {
                break;
            }

            show_cursor((u8)(row * 16), (uint8_t)(cur == i));
            OLED_ShowString(8, (u8)(row * 16), (u8 *)TASK_NAMES[i], 16, 1);
        }
        OLED_Refresh();

        do {
            key = Key_Scan();
        } while (key == -1);
        wait_release();

        if (key == KEY_1) {
            if (cur > 0) {
                cur--;
            }
        } else if (key == KEY_2) {
            if (cur < (TASK_COUNT - 1)) {
                cur++;
            }
        } else if (key == KEY_3) {
            return (Task_ID)cur;
        }
    }
}

/**
 * @brief Handle the second-level confirmation page.
 * @param task Task chosen from the first-level list.
 * @return 1 when confirmed, otherwise 0.
 */
static uint8_t UI_Menu_L2(Task_ID task)
{
    uint8_t sel = 0U;
    int8_t key;

    while (1) {
        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)TASK_NAMES[task], 16, 1);
        OLED_ShowString(0, 16, (u8 *)"  Selected?", 16, 1);

        show_cursor(32, (uint8_t)(sel == 0U));
        OLED_ShowString(8, 32, (u8 *)"[OK]", 16, 1);

        show_cursor(48, (uint8_t)(sel == 1U));
        OLED_ShowString(8, 48, (u8 *)"[Cancel]", 16, 1);
        OLED_Refresh();

        do {
            key = Key_Scan();
        } while (key == -1);
        wait_release();

        if ((key == KEY_1) || (key == KEY_2)) {
            // 确认页只有两个选项，按上下键都执行翻转即可。
            sel ^= 1U;
        } else if (key == KEY_3) {
            return (uint8_t)(sel == 0U);
        }
    }
}

Task_ID UI_Process(void)
{
    while (1) {
        Task_ID sel = UI_Menu_L1();

        if (UI_Menu_L2(sel) != 0U) {
            return sel;
        }
    }
}
