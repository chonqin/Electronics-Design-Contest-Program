/**
 * @file    bsp_test.c
 * @brief   BSP 测试入口实现
 */

#include "bsp_test.h"
#include "board.h"
#include "bsp_encoder.h"
#include "bsp_icm42688.h"
#include "bsp_key.h"
#include "bsp_motor.h"
#include "imu.h"
#include "pid.h"
#include "ui.h"
#include <stdio.h>

/* 左右轮保留独立 PID 参数，后续可单独微调 */
#define PID_L_KP            65.0f
#define PID_L_KI            7.5f
#define PID_L_KD            4.5f
#define PID_R_KP            65.0f
#define PID_R_KI            7.5f
#define PID_R_KD            4.5f
#define PID_TUNE_STEP       5
#define PID_TUNE_MAX        40
#define PID_TUNE_MIN        0
#define MOTOR_DUTY_STEP     100
#define MOTOR_DUTY_MAX      MOTOR_PWM_PERIOD
#define MOTOR_DUTY_MIN      (-MOTOR_PWM_PERIOD)

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
 * @brief 将编码器方向枚举转换为紧凑日志字符串
 * @param dir 编码器方向
 * @return 方向字符串
 */
static const char *encoder_dir_str(ENCODER_DIR dir)
{
    return (dir == FORWARD) ? "F" : "R";
}

/**
 * @brief 单次按键动作后等待所有按键松开
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
 * @return 限幅后的数值
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

/**
 * @brief 将编码器 ID 转成调参界面使用的轮子名称
 * @param id 编码器选择
 * @return 轮子名称
 */
static const char *pid_wheel_name(ENCODER_ID id)
{
    return (id == ENCODER_1) ? "Left" : "Right";
}

/**
 * @brief 将编码器 ID 映射为对应电机
 * @param id 编码器选择
 * @return 电机选择
 */
static Motor_ID pid_motor_of(ENCODER_ID id)
{
    return (id == ENCODER_1) ? MOTOR_A : MOTOR_B;
}

/**
 * @brief 选择要整定的轮子
 * @return 被选择的编码器 ID
 */
static ENCODER_ID pid_select_wheel(void)
{
    uint8_t left = 1U;
    int8_t key;

    while (1) {
        UI_Test_PIDSelect(left);

        do {
            key = Key_Scan();
        } while (key == -1);

        test_wait_key_release();

        if (key == KEY_1 || key == KEY_2) {
            left ^= 1U;
        } else if (key == KEY_3) {
            return left ? ENCODER_1 : ENCODER_2;
        }
    }
}

/**
 * @brief 带 OLED 编码器反馈的电机 PWM 占空比测试
 * @details KEY1 增加占空比，KEY2 减少占空比，KEY3 清零占空比
 */
void BSP_Test_Motor(void)
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

/** @brief imu.c 暴露的 IMU 采样节拍 */
extern volatile uint32_t nowtime;
/** @brief imu.c 暴露的最新加速度计采样值 */
extern icm42688_real_data_t accval;
/** @brief imu.c 暴露的最新陀螺仪采样值 */
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
/**
 * @brief 带 OLED 反馈和按键目标调节的 PID 控速测试
 */
void BSP_Test_PID(void)
{
    PID pid_l;
    PID pid_r;
    PID *pid;
    ENCODER_ID enc_id;
    Motor_ID motor_id;
    Motor_ID motor_off;
    const char *wheel;
    int set = 0;
    int actual = 0;
    int out = 0;
    int8_t key;

    UI_Init();

    enc_id = pid_select_wheel();
    motor_id = pid_motor_of(enc_id);
    motor_off = (motor_id == MOTOR_A) ? MOTOR_B : MOTOR_A;
    wheel = pid_wheel_name(enc_id);

    encoder_init();
    Motor_Init();
    PID_Init(&pid_l, PID_L_KP, PID_L_KI, PID_L_KD,
             (float)(-MOTOR_PWM_PERIOD), (float)MOTOR_PWM_PERIOD);
    PID_Init(&pid_r, PID_R_KP, PID_R_KI, PID_R_KD,
             (float)(-MOTOR_PWM_PERIOD), (float)MOTOR_PWM_PERIOD);

    pid = (enc_id == ENCODER_1) ? &pid_l : &pid_r;
    Motor_Stop(motor_off);
    Motor_Stop(motor_id);

    UI_Test_PID(wheel, actual, set, out);

    while (1) {
        key = Key_Scan();
        if (key != -1) {
            if (key == KEY_1) {
                set += PID_TUNE_STEP;
            } else if (key == KEY_2) {
                set -= PID_TUNE_STEP;
            } else if (key == KEY_3) {
                set = 0;
                PID_Reset(pid);
                Motor_Stop(motor_id);
            }

            set = test_limit_int(set, PID_TUNE_MIN, PID_TUNE_MAX);
            test_wait_key_release();
        }

        actual = encoder_get_count(enc_id);
        if (set == 0) {
            out = 0;
            Motor_Stop(motor_id);
        } else {
            out = (int)PID_CalcTarget(pid, (float)set, (float)actual);
            out = test_limit_int(out, MOTOR_DUTY_MIN, MOTOR_DUTY_MAX);
            Motor_SetDuty(motor_id, (int16_t)out);
        }

        Motor_Stop(motor_off);

        UI_Test_PID(wheel, actual, set, out);
        lc_printf("actual,target,out:%d , %d , %d\r\n",actual, set, out);
        delay_ms(50);
    }
}
