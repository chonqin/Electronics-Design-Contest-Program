/**
 * @file test.c
 * @brief 硬件联调测试入口实现
 */

#include "test.h"
#include "board.h"
#include "bsp_encoder.h"
#include "bsp_icm42688.h"
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

/**
 * @brief 将编码器方向枚举转换为简短显示字符
 * @param dir 编码器方向
 * @return 方向字符串
 */
static const char *encoder_dir_str(ENCODER_DIR dir)
{
    return (dir == FORWARD) ? "F" : "R";
}

/**
 * @brief 单次按键动作后等待所有按键释放
 */
static void test_wait_key_release(void)
{
    while (Key_Scan() != -1) {
        delay_ms(10);
    }
}

/**
 * @brief 将整数限制到闭区间
 * @param val 输入值
 * @param min 下限
 * @param max 上限
 * @return 限幅后的整数
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
    encoder_init();
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

        Motor_SetDuty(MOTOR_A, (int16_t)duty);
        Motor_SetDuty(MOTOR_B, (int16_t)duty);

        e1 = encoder_get_count(ENCODER_1);
        e2 = encoder_get_count(ENCODER_2);

        UI_Test_Motor(duty, e1, e2);
        lc_printf("Motor duty:%d e1:%d e2:%d\r\n", duty, e1, e2);
        delay_ms(50);
    }
}

void Test_OLED(void)
{
    UI_Init();
    encoder_init();

    while (1) {
        UI_Test_OLED();
        delay_ms(100);
    }
}

void Test_Encoder(void)
{
    int e1_count;
    int e2_count;

    encoder_init();
    Motor_Init();
    UI_Init();

    Motor_SetDuty(MOTOR_A, 3300);
    Motor_SetDuty(MOTOR_B, 3300);

    while (1) {
        e1_count = encoder_get_count(ENCODER_1);
        e2_count = encoder_get_count(ENCODER_2);
        lc_printf("E1:%d %s E2:%d %s\r\n",
                  e1_count,
                  encoder_dir_str(encoder_get_dir(ENCODER_1)),
                  e2_count,
                  encoder_dir_str(encoder_get_dir(ENCODER_2)));
        UI_Test_Encoder(e1_count, e2_count);
        delay_ms(20);
    }
}

void Test_KEY(void)
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

void Test_UartReceive(void)
{
    uint8_t buf[UART_TEST_BUF_SIZE];
    uint16_t len;

    BSP_Uart_FlushRx();
    lc_printf("UART0 receive echo test start\r\n");

    while (1) {
        len = BSP_Uart_Read(buf, UART_TEST_BUF_SIZE);

        if (len > 0U) {
            /* 将上位机发来的数据原样回发，用于验证 RX 中断和 TX 发送链路 */
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

        for (uint8_t i = 0U; i < 8U; i++) {
            line[6U + i] = '0';
            if ((mask & (uint8_t)(1U << i)) != 0U) {
                /* bit0-bit7 对应 X1-X8，1 表示检测到黑线。 */
                line[6U + i] = '1';
            }
        }

        OLED_ShowString(0, 24, (u8 *)line, 16, 1);
        OLED_Refresh();
        lc_printf("%s\r\n", line);
        delay_ms(100);
    }
}
