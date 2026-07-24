/**
 * @file test.c
 * @brief Hardware bring-up test implementations.
 */

#include "test.h"
#include "board.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
#include "bsp_motor.h"
#include "bsp_track.h"
#include "bsp_uart.h"
#include "imu.h"
#include "oled.h"
#include "ui.h"
#include <stdio.h>

#define MOTOR_DUTY_STEP     100
#define MOTOR_DUTY_MAX      MOTOR_PWM_PERIOD
#define MOTOR_DUTY_MIN      (-MOTOR_PWM_PERIOD)
#define UART_TEST_BUF_SIZE  64U

/**
 * @brief Wait until one key action is fully released.
 */
static void test_wait_key_release(void)
{
    while (Key_Scan() != -1) {
        delay_ms(10);
    }
}

/**
 * @brief Clamp one integer into a closed interval.
 * @param val Input value.
 * @param min Lower bound.
 * @param max Upper bound.
 * @return Clamped value.
 */
static int test_limit_int(int val, int min, int max)
{
    if (val > max) {
        return max;
    }

    if (val < min) {
        return min;
    }

    return val;
}

void Test_Motor(void)
{
    int8_t key;
    int duty = 0;
    int e1 = 0;
    int e2 = 0;

    UI_Init();
    Encoder_Init();
    Motor_Init();

    UI_Test_Motor(duty, e1, e2);

    while (1) {
        key = Key_Scan();
        if (key != -1) {
            if (key == KEY_1) {
                duty += MOTOR_DUTY_STEP;
            } else if (key == KEY_2) {
                duty -= MOTOR_DUTY_STEP;
            } else if (key == KEY_3) {
                duty = 0;
            }

            duty = test_limit_int(duty, MOTOR_DUTY_MIN, MOTOR_DUTY_MAX);
            test_wait_key_release();
        }

        // 双电机使用同一占空比，便于先验证正反转与编码器方向。
        Motor_SetDuty(MOTOR_A, (int16_t)duty);
        Motor_SetDuty(MOTOR_B, (int16_t)duty);

        e1 = Encoder_Read(ENCODER_E1);
        e2 = Encoder_Read(ENCODER_E2);

        UI_Test_Motor(duty, e1, e2);
        lc_printf("Motor duty:%d e1:%d e2:%d\r\n", duty, e1, e2);
        delay_ms(50);
    }
}

void Test_OLED(void)
{
    UI_Init();

    while (1) {
        DL_GPIO_togglePins(GPIO_LED_LED1_PORT, GPIO_LED_LED1_PIN);
        DL_GPIO_togglePins(GPIO_LED_LED2_PORT, GPIO_LED_LED2_PIN);
        UI_Test_OLED();
        delay_ms(100);
    }
}

void Test_IMU(void)
{
    float angles[3];

    IMU_init();

    DL_TimerA_stopCounter(TIMER_IMU_TICK_INST);
    NVIC_DisableIRQ(TIMER_IMU_TICK_INST_INT_IRQN);
    DL_GPIO_clearInterruptStatus(GPIO_IMU_INT_PORT, GPIO_IMU_INT_PA16_PIN);
    NVIC_ClearPendingIRQ(GPIO_IMU_INT_INT_IRQN);
    NVIC_EnableIRQ(GPIO_IMU_INT_INT_IRQN);

    IMU_resetTimestamp();
    UI_IMU_Calibrating();

    /* Warm up the attitude estimator before entering the live page. */
    for (uint16_t i = 0U; i < 200U; i++) {
        float dummy_angles[3];

        IMU_getYawPitchRoll(dummy_angles);
        delay_ms(10);
    }

    while (1) {
        IMU_getYawPitchRoll(angles);
        // 页面与串口都直接显示当前姿态解算结果。
        UI_Test_IMU(angles);
        lc_printf("P R Y:%.2f , %.2f , %.2f\n", angles[1], angles[2], angles[0]);
    }
}

void Test_UartReceive(void)
{
    uint8_t buf[UART_TEST_BUF_SIZE];
    uint16_t len;

    BSP_Uart_FlushRx();
    lc_printf("UART0 receive echo test start\r\n");

    while (1) {
        len = BSP_Uart_Read(buf, UART_TEST_BUF_SIZE);

        if (len > 0U) {
            /* Echo host data directly to verify the RX/TX path. */
            (void)BSP_Uart_Write(buf, len);
        }
    }
}

void Test_Track(void)
{
    uint8_t mask;
    char line[16] = "x1-x8:00000000";

    UI_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"Track Test", 16, 1);

    while (1) {
        mask = Track_ReadMask();

        // 按位展开 8 路寻迹状态，方便和探头位置一一对应。
        for (uint8_t i = 0U; i < 8U; i++) {
            line[6U + i] = '0';
            if ((mask & (uint8_t)(1U << i)) != 0U) {
                /* bit0-bit7 map to X1-X8, and 1 means black line detected. */
                line[6U + i] = '1';
            }
        }

        OLED_ShowString(0, 24, (u8 *)line, 16, 1);
        OLED_Refresh();
        lc_printf("%s\r\n", line);
        delay_ms(100);
    }
}
