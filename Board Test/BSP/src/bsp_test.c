/**
 * @file    bsp_test.c
 * @brief   BSP test entry implementations.
 */

#include "bsp_test.h"
#include "board.h"
#include "bsp_encoder.h"
#include "bsp_icm42688.h"
#include "bsp_key.h"
#include "bsp_motor.h"
#include "imu.h"
#include "ui.h"
#include <stdio.h>

static void LED1_On(void)
{
    DL_GPIO_setPins(GPIO_LED_LED1_PORT, GPIO_LED_LED1_PIN);
}

static void LED1_Off(void)
{
    DL_GPIO_clearPins(GPIO_LED_LED1_PORT, GPIO_LED_LED1_PIN);
}

static void LED2_On(void)
{
    DL_GPIO_setPins(GPIO_LED_LED2_PORT, GPIO_LED_LED2_PIN);
}

static void LED2_Off(void)
{
    DL_GPIO_clearPins(GPIO_LED_LED2_PORT, GPIO_LED_LED2_PIN);
}

static void LED_Blink(uint8_t count, uint32_t ms)
{
    for (uint8_t i = 0; i < count; i++) {
        LED1_On();
        LED2_On();
        delay_ms(ms);
        LED1_Off();
        LED2_Off();
        delay_ms(ms);
    }
}

/**
 * @brief  Drive a single motor forward and reverse once.
 * @param  motor Motor selector.
 * @return Always returns 0 and relies on manual observation.
 */
static uint8_t test_single_motor(Motor_ID motor)
{
    Motor_SetDuty(motor, 2000);
    delay_ms(2000);
    Motor_Stop(motor);
    delay_ms(500);

    Motor_SetDuty(motor, -2000);
    delay_ms(2000);
    Motor_Stop(motor);
    delay_ms(500);

    return 0;
}

void BSP_Test_Motor(void)
{
    uint8_t result_a;
    uint8_t result_b;

    Motor_Init();
    encoder_init();

    LED_Blink(1, 200);
    delay_ms(200);

    result_a = test_single_motor(MOTOR_A);
    if (result_a == 0U) {
        LED1_On();
    }
    delay_ms(1000);

    result_b = test_single_motor(MOTOR_B);
    if (result_b == 0U) {
        LED2_On();
    }
    delay_ms(1000);
}

void BSP_Test_OLED(void)
{
    UI_Init();
    encoder_init();

    while (1) {
        UI_Test_OLED();
        delay_ms(100);
    }
}

void BSP_Test_Encoder(void)
{
    uint32_t count;

    encoder_init();
    Motor_Init();
    UI_Init();

    Motor_SetDuty(MOTOR_A, 3300);

    while (1) {
        count = get_encoder_count();
        lc_printf("count : %d\r\n", count);
        UI_Test_Encoder(count);
        delay_ms(20);
    }
}

void BSP_Test_KEY(void)
{
    uint32_t key1_val;
    uint32_t key2_val;
    uint32_t key3_val;

    LED1_Off();
    LED2_Off();

    while (1) {
        key1_val = DL_GPIO_readPins(GPIO_KEY_PIN_18_PORT, GPIO_KEY_PIN_18_PIN);
        key2_val = DL_GPIO_readPins(GPIO_KEY_PIN_13_PORT, GPIO_KEY_PIN_13_PIN);
        key3_val = DL_GPIO_readPins(GPIO_KEY_PIN_17_PORT, GPIO_KEY_PIN_17_PIN);

        lc_printf("key1:%d key2:%d key3:%d\r\n", key1_val, key2_val, key3_val);

        if ((key1_val == 0U) || (key2_val == 0U) || (key3_val == 0U)) {
            LED2_On();
        } else {
            LED2_Off();
        }

        delay_ms(500);
    }
}

/** @brief IMU sample tick exposed by imu.c. */
extern volatile uint32_t nowtime;
/** @brief Latest accelerometer sample exposed by imu.c. */
extern icm42688_real_data_t accval;
/** @brief Latest gyroscope sample exposed by imu.c. */
extern icm42688_real_data_t gyroval;

void BSP_Test_IMU(void)
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

    for (uint16_t i = 0U; i < 200U; i++) {
        float dummy_angles[3];

        IMU_getYawPitchRoll(dummy_angles);
        delay_ms(10);
    }

    while (1) {
        IMU_getYawPitchRoll(angles);
        UI_Test_IMU(angles);
        lc_printf("P R Y:%.2f , %.2f , %.2f\n", angles[1], angles[2], angles[0]);
    }
}
