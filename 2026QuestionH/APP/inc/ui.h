/**
 * @file ui.h
 * @brief OLED UI helpers and menu entry point.
 */
#ifndef __UI_H
#define __UI_H

#include <stdint.h>

/**
 * @brief Task IDs returned by the menu.
 */
typedef enum {
    TASK_1 = 0,
    TASK_2 = 1,
    TASK_3 = 2,
    TASK_4 = 3,
    TASK_5 = 4,
    TASK_6 = 5,
    TASK_7 = 6,
    TASK_8 = 7,
    TASK_COUNT = 8,
    TASK_NONE = -1
} Task_ID;

/** @brief 任务 3 门控计时状态。 */
typedef enum {
    UI_TIMER_WAIT = 0,
    UI_TIMER_RUNNING,
    UI_TIMER_STOPPED
} UI_TimerState;

/**
 * @brief Initialize keys and OLED for the UI.
 */
void UI_Init(void);

/**
 * @brief Draw the static OLED test page.
 */
void UI_Test_OLED(void);

/**
 * @brief Refresh the motor test view on OLED.
 * @param duty Current motor PWM duty.
 * @param e1 Encoder E1 speed sample.
 * @param e2 Encoder E2 speed sample.
 */
void UI_Test_Motor(int duty, int e1, int e2);

/**
 * @brief Draw the PID wheel select page.
 * @param left Non-zero selects the left wheel label.
 */
void UI_Test_PIDSelect(uint8_t left);

/**
 * @brief Refresh PID tuning values on OLED.
 * @param wheel Wheel label.
 * @param actual Current speed sample.
 * @param target Target speed sample.
 * @param output Current PID output.
 */
void UI_Test_PID(const char *wheel, int actual, int target, int output);

/**
 * @brief Draw the IMU calibration waiting page.
 */
void UI_IMU_Calibrating(void);

/**
 * @brief Refresh IMU yaw, pitch, and roll on OLED.
 * @param angles Angle array in order [yaw, pitch, roll].
 */
void UI_Test_IMU(float *angles);

/** @brief 显示“任务1：图传测试”页面。 */
void UI_Task1VideoTest(void);

/**
 * @brief 实时显示任务运行时间
 * @param task 任务编号
 * @param ms 当前用时，单位为 ms
 * @param odo 自发车以来的车体中心累计编码器计数
 */
void UI_TaskRunning(uint8_t task, uint32_t ms, int32_t odo);

/**
 * @brief 显示指定任务的最终计时结果
 * @param task 任务编号
 * @param ms 最终用时，单位为 ms
 * @param odo 停车时的车体中心累计编码器计数
 */
void UI_TaskResult(uint8_t task, uint32_t ms, int32_t odo);

/**
 * @brief 显示指定任务的门控计时状态和时间
 * @param task 任务编号
 * @param ms 当前或最终时间，单位为 ms
 * @param state 当前计时状态
 */
void UI_TaskGateTimer(uint8_t task, uint32_t ms, UI_TimerState state);

/**
 * @brief Run the menu flow until one task is confirmed.
 * @param confirm_ms 输出确认键被识别时的时间戳，单位为 ms
 * @return Selected task ID.
 */
Task_ID UI_Process(uint32_t *confirm_ms);

#endif
