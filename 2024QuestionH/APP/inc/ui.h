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
    TASK_COUNT = 7,
    TASK_NONE = -1
} Task_ID;

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

/**
 * @brief Run the menu flow until one task is confirmed.
 * @return Selected task ID.
 */
Task_ID UI_Process(void);

#endif
