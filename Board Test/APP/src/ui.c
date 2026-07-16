/**
 * @file    ui.c
 * @brief   用户界面实现：基础显示函数 + 多级菜单
 */

#include "ui.h"
#include "oled.h"
#include "bsp_key.h"
#include "board.h"
#include <stdio.h>

/* ------------------------------------------------------------------ */
/*  内部常量                                                            */
/* ------------------------------------------------------------------ */

/** 一级菜单各项名称，顺序对应 Task_ID */
static const char *TASK_NAMES[TASK_COUNT] = {
    "TASK1", "TASK2", "TASK3", "TASK4"
};

/* ------------------------------------------------------------------ */
/*  内部辅助                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief 等待所有按键松开，防止同一次按键被重复响应
 */
static void wait_release(void)
{
    while (Key_Scan() != -1)
        delay_ms(10);
}

/**
 * @brief Draw the menu cursor at a fixed row.
 * @param y      Row start coordinate.
 * @param active 1=selected, 0=not selected.
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
 * @brief Clear one 16px-high text row without writing outside the OLED width.
 * @param y Row start coordinate.
 */
static void clear_text_row(u8 y)
{
    OLED_ShowString(0, y, (u8 *)"                ", 16, 1);
}

/* ------------------------------------------------------------------ */
/*  基础显示函数                                                        */
/* ------------------------------------------------------------------ */

void UI_Init(void)
{
    Key_Init();
    OLED_Init();
    OLED_ColorTurn(0);    /* 正常显示（非反色） */
    OLED_DisplayTurn(0);  /* 正常方向 */
}

void UI_Test_OLED(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0,  (u8 *)"Encoder Test", 16, 1);
    OLED_ShowString(0, 20, (u8 *)"E1:", 16, 1);
    OLED_ShowString(0, 40, (u8 *)"E2:", 16, 1);
    OLED_Refresh();
}

void UI_Test_Encoder(uint32_t count)
{
    OLED_ShowNum(10, 0,  count, 5, 16, 1);
    OLED_Refresh();
}

void UI_IMU_Calibrating(void)
{
    OLED_Clear();
    OLED_ShowString(0, 24, (u8 *)"Calibrating...", 16, 1);
    OLED_Refresh();
}

/**
 * @brief 在 OLED 指定位置显示有符号浮点数（保留2位小数）
 * @param x, y  显示坐标
 * @param val   浮点值
 * @note  格式固定为 %+7.2f，便于覆盖上一次显示内容。
 */
static void show_float(u8 x, u8 y, float val)
{
    char buf[12];

    (void)snprintf(buf, sizeof(buf), "%+7.2f", (double)val);
    OLED_ShowString(x, y, (u8 *)buf, 16, 1);
}

/**
 * @brief 刷新 IMU 三轴姿态角到 OLED
 * @param angles  float[3]：[0]=Yaw, [1]=Pitch, [2]=Roll
 * @note  显示格式：Y:+xxx.xx / P:+xxx.xx / R:+xxx.xx
 */
void UI_Test_IMU(float *angles)
{
    /* 行间距 22px，三行恰好填满 64px 屏幕 */

    clear_text_row(0);
    OLED_ShowString(0, 0, (u8 *)"P:", 16, 1);
    show_float(16, 0, angles[1]);   /* Pitch */

    clear_text_row(22);
    OLED_ShowString(0, 22, (u8 *)"R:", 16, 1);
    show_float(16, 22, angles[2]);   /* Roll  */

    clear_text_row(44);
    OLED_ShowString(0, 44, (u8 *)"Y:", 16, 1);
    show_float(16, 44,  angles[0]);   /* Yaw   */

    OLED_Refresh();
}

/* ------------------------------------------------------------------ */
/*  一级菜单                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief 显示并处理一级菜单（TASK1-TASK4）
 * @details KEY1=上移, KEY2=下移, KEY3=确认
 * @return 用户选中的 Task_ID
 */
static Task_ID UI_Menu_L1(void)
{
    int8_t  cur = 0;   /* 当前光标位置 0-3 */
    int8_t  key;

    while (1) {
        /* --- 刷新显示 --- */
        OLED_Clear();
        for (int8_t i = 0; i < TASK_COUNT; i++) {
            /* 光标字符：选中行显示 '>'，其余显示 ' ' */
            if (cur == i) {
                show_cursor((u8)(i * 16), 1U);
            } else {
                show_cursor((u8)(i * 16), 0U);
            }
            OLED_ShowString(8, (u8)(i * 16), (u8 *)TASK_NAMES[i], 16, 1);
        }
        OLED_Refresh();

        /* --- 等待按键 --- */
        do { key = Key_Scan(); } while (key == -1);
        wait_release();

        if (key == KEY_1) {
            /* 上移，到顶则停 */
            if (cur > 0) cur--;
        } else if (key == KEY_2) {
            /* 下移，到底则停 */
            if (cur < TASK_COUNT - 1) cur++;
        } else if (key == KEY_3) {
            /* 确认选中 */
            return (Task_ID)cur;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  二级菜单                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief 显示并处理二级确认菜单
 * @param task  从一级菜单传入的选中任务
 * @details 默认光标在"OK"。KEY1/KEY2 切换，KEY3 确认。
 * @return 1=用户确认, 0=用户取消
 */
static uint8_t UI_Menu_L2(Task_ID task)
{
    uint8_t sel = 0;   /* 0=OK, 1=Cancel；默认指向 OK */
    int8_t  key;

    while (1) {
        /* --- 刷新显示 --- */
        OLED_Clear();
        OLED_ShowString(0, 0,  (u8 *)TASK_NAMES[task], 16, 1);   /* 任务名 */
        OLED_ShowString(0, 16, (u8 *)"  Selected?",     16, 1);   /* 副标题 */
        /* OK 行 */
        if (sel == 0U) {
            show_cursor(32, 1U);
        } else {
            show_cursor(32, 0U);
        }
        OLED_ShowString(8, 32, (u8 *)"[OK]",            16, 1);
        /* Cancel 行 */
        if (sel == 1U) {
            show_cursor(48, 1U);
        } else {
            show_cursor(48, 0U);
        }
        OLED_ShowString(8, 48, (u8 *)"[Cancel]",        16, 1);
        OLED_Refresh();

        /* --- 等待按键 --- */
        do { key = Key_Scan(); } while (key == -1);
        wait_release();

        if (key == KEY_1 || key == KEY_2) {
            /* 在 OK / Cancel 之间切换 */
            sel ^= 1;
        } else if (key == KEY_3) {
            if (sel == 0U) {
                return 1U;
            }
            return 0U;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  对外菜单入口                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief 运行多级菜单，阻塞至用户确认选择一个任务
 * @return 用户最终确认的 Task_ID
 */
Task_ID UI_Process(void)
{
    while (1) {
        Task_ID sel = UI_Menu_L1();
        if (UI_Menu_L2(sel)) {
            return sel;   /* 用户在二级菜单选了 OK */
        }
        /* 用户在二级菜单选了 Cancel，回到一级菜单 */
    }
}
