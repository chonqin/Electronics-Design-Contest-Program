/**
 * @file ui.c
 * @brief OLED UI helpers and task menu implementation.
 */

#include "ui.h"
#include "oled.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
#include "board.h"
#include <stdio.h>
#include <string.h>

/** @brief Menu labels aligned with main.c task dispatch order. */
static const char *TASK_NAMES[TASK_COUNT] = {
    "TASK1 VIDEO", "TASK2 TRACK", "TASK3 TIMER", "TASK4 TRACK",
    "TASK5 TRACK", "TASK6 TRACK", "UART", "TRACK TEST"
};

/** @brief Number of visible rows when using the 16px font. */
#define MENU_ROWS 4
/** @brief 小字体里程所在的 OLED 页。 */
#define TASK_ODO_PAGE 2U
/** @brief 24 像素计时文本所在的 OLED 起始页。 */
#define TASK_TIME_PAGE 4U
/** @brief 24 像素计时文本覆盖的 OLED 页数。 */
#define TASK_TIME_PAGES 3U

/** @brief 当前已绘制运行界面的任务编号。 */
static uint8_t run_task = 0xFFU;

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

/**
 * @brief 计算字符串在 128 像素 OLED 上的水平居中坐标
 * @param str 待显示字符串
 * @param width 单字符像素宽度
 * @return 字符串起始 x 坐标
 */
static u8 task_text_x(const char *str, uint8_t width)
{
    uint32_t px;

    px = (uint32_t)strlen(str) * width;
    if (px >= 128U) {
        return 0U;
    }

    return (u8)((128U - px) / 2U);
}

/**
 * @brief 将任务编号、里程和放大计时写入 OLED 显存
 * @param task 任务编号
 * @param ms 时间，单位为 ms
 * @param odo 当前车体中心累计编码器计数
 */
static void show_task_info(uint8_t task, uint32_t ms, int32_t odo)
{
    char task_buf[10];
    char odo_buf[22];
    char time_buf[16];
    u8 x;
    uint32_t sec = ms / 1000U;
    uint32_t rem = ms % 1000U;

    OLED_Clear();
    (void)snprintf(task_buf, sizeof(task_buf), "TASK %u", task);
    x = task_text_x(task_buf, 8U);
    OLED_ShowString(x, 0U, (u8 *)task_buf, 16, 1);
    (void)snprintf(odo_buf, sizeof(odo_buf), "ODO:%+11ld", (long)odo);
    x = task_text_x(odo_buf, 6U);
    OLED_ShowString(x, 16U, (u8 *)odo_buf, 8, 1);

    /* 24 像素字体最多容纳五位秒数，超出后保持最大可显示值。 */
    if (sec > 99999U) {
        sec = 99999U;
        rem = 999U;
    }
    (void)snprintf(time_buf, sizeof(time_buf), "%lu.%03lus",
                   (unsigned long)sec, (unsigned long)rem);
    x = task_text_x(time_buf, 12U);
    OLED_ShowString(x, 32U, (u8 *)time_buf, 24, 1);
}

/**
 * @brief 将任务编号、PB0 状态和时间写入 OLED 显存
 * @param task 任务编号
 * @param ms 当前或最终时间，单位为 ms
 * @param state 当前计时状态
 */
static void show_gate_timer(uint8_t task, uint32_t ms, UI_TimerState state)
{
    const char *status;
    char task_buf[10];
    char time_buf[16];
    u8 x;
    uint32_t sec = ms / 1000U;
    uint32_t rem = ms % 1000U;

    if (state == UI_TIMER_RUNNING) {
        status = "TIMING";
    } else if (state == UI_TIMER_STOPPED) {
        status = "STOPPED";
    } else {
        status = "WAIT PB0 HIGH";
    }

    /* 保持时间文本在 24 像素字体的可显示范围内。 */
    if (sec > 99999U) {
        sec = 99999U;
        rem = 999U;
    }

    /* 固定绘制任务标题，并清除状态和时间区域中的旧字符。 */
    (void)snprintf(task_buf, sizeof(task_buf), "TASK %u", task);
    clear_text_row(0U);
    x = task_text_x(task_buf, 8U);
    OLED_ShowString(x, 0U, (u8 *)task_buf, 16, 1);
    clear_text_row(16U);
    x = task_text_x(status, 8U);
    OLED_ShowString(x, 16U, (u8 *)status, 16, 1);
    OLED_ShowString(0U, 32U, (u8 *)"          ", 24, 1);
    (void)snprintf(time_buf, sizeof(time_buf), "%lu.%03lus",
                   (unsigned long)sec, (unsigned long)rem);
    x = task_text_x(time_buf, 12U);
    OLED_ShowString(x, 32U, (u8 *)time_buf, 24, 1);
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

void UI_Task1VideoTest(void)
{
    OLED_Clear();
    /* 点阵索引 11 至 16 依次对应“任务图传测试”。 */
    OLED_ShowChinese(0U, 0U, 11U, 16U, 1U);
    OLED_ShowChinese(16U, 0U, 12U, 16U, 1U);
    OLED_ShowString(32U, 0U, (u8 *)"1:", 16, 1);
    OLED_ShowChinese(48U, 0U, 13U, 16U, 1U);
    OLED_ShowChinese(64U, 0U, 14U, 16U, 1U);
    OLED_ShowChinese(80U, 0U, 15U, 16U, 1U);
    OLED_ShowChinese(96U, 0U, 16U, 16U, 1U);
    OLED_Refresh();
    run_task = 1U;
}

void UI_TaskRunning(uint8_t task, uint32_t ms, int32_t odo)
{
    show_task_info(task, ms, odo);
    if (run_task != task) {
        OLED_Refresh();
        run_task = task;
        return;
    }

    /* 分别刷新小字体里程和大字体时间，未使用页面保持不传输。 */
    OLED_RefreshPages(TASK_ODO_PAGE, 1U);
    OLED_RefreshPages(TASK_TIME_PAGE, TASK_TIME_PAGES);
}

void UI_TaskResult(uint8_t task, uint32_t ms, int32_t odo)
{
    run_task = 0xFFU;
    show_task_info(task, ms, odo);
    OLED_Refresh();
}

void UI_TaskGateTimer(uint8_t task, uint32_t ms, UI_TimerState state)
{
    uint8_t first = 0U;
    static UI_TimerState last_state = UI_TIMER_STOPPED;

    if (run_task != task) {
        OLED_Clear();
        first = 1U;
    }
    run_task = task;
    show_gate_timer(task, ms, state);
    if (first != 0U) {
        /* 首次进入时刷新全部页面，清除菜单底部可能残留的像素。 */
        OLED_Refresh();
        last_state = state;
        return;
    }
    if (last_state != state) {
        OLED_RefreshPages(0U, 4U);
        last_state = state;
    }
    OLED_RefreshPages(TASK_TIME_PAGE, TASK_TIME_PAGES);
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
            OLED_ShowString(8, (u8)(row * 16),
                            (u8 *)TASK_NAMES[i], 16, 1);
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
 * @param confirm_ms 输出确认按键被识别时的时间戳，单位为 ms
 * @return 1 when confirmed, otherwise 0.
 */
static uint8_t UI_Menu_L2(Task_ID task, uint32_t *confirm_ms)
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
        OLED_ShowString(8, 48, (u8 *)"[Back]", 16, 1);
        OLED_Refresh();

        do {
            key = Key_Scan();
        } while (key == -1);

        if ((key == KEY_1) || (key == KEY_2)) {
            // 确认页只有两个选项，按上下键都执行翻转即可。
            wait_release();
            sel ^= 1U;
        } else if (key == KEY_3) {
            if ((sel == 0U) && (confirm_ms != NULL)) {
                /* 在等待松键前锁存，确保计时起点对应确认按下时刻。 */
                *confirm_ms = Encoder_GetMs();
            }
            wait_release();
            return (uint8_t)(sel == 0U);
        }
    }
}

Task_ID UI_Process(uint32_t *confirm_ms)
{
    while (1) {
        Task_ID sel = UI_Menu_L1();

        if (UI_Menu_L2(sel, confirm_ms) != 0U) {
            return sel;
        }
    }
}
