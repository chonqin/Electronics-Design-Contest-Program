/**
 * @file bsp_motor.c
 * @brief TB6612 双路直流电机驱动实现
 *
 * TIMA1 的两路 PWM 分别驱动 PWMA 和 PWMB，四个 GPIO 控制转向。
 * STBY 由硬件保持高电平。
 */
#include "bsp_motor.h"

/** @brief 记录当前方向，避免同向调速时重复切断 PWM。 */
static int8_t motor_dir[2] = {0, 0};

/**
 * @brief 设置指定电机的 PWM 比较值。
 * @param motor 电机选择。
 * @param pwm PWM 比较值，范围 0..MOTOR_PWM_PERIOD。
 */
static void motor_write_pwm(Motor_ID motor, uint16_t pwm)
{
    if (motor == MOTOR_A) {
        DL_TimerA_setCaptureCompareValue(
            PWM_MOTOR_INST, pwm, GPIO_PWM_MOTOR_C0_IDX);
    } else if (motor == MOTOR_B) {
        DL_TimerA_setCaptureCompareValue(
            PWM_MOTOR_INST, pwm, GPIO_PWM_MOTOR_C1_IDX);
    }
}

/**
 * @brief 设置指定电机的两个方向输入。
 * @param motor 电机选择。
 * @param in1 IN1 是否置高。
 * @param in2 IN2 是否置高。
 */
static void motor_write_dir(Motor_ID motor, uint8_t in1, uint8_t in2)
{
    uint32_t pin1;
    uint32_t pin2;

    if (motor == MOTOR_A) {
        pin1 = GPIO_MOTOR_AIN1_PIN;
        pin2 = GPIO_MOTOR_AIN2_PIN;
    } else if (motor == MOTOR_B) {
        pin1 = GPIO_MOTOR_BIN1_PIN;
        pin2 = GPIO_MOTOR_BIN2_PIN;
    } else {
        return;
    }

    DL_GPIO_clearPins(GPIO_MOTOR_PORT, pin1 | pin2);
    if (in1 != 0U) {
        DL_GPIO_setPins(GPIO_MOTOR_PORT, pin1);
    }
    if (in2 != 0U) {
        DL_GPIO_setPins(GPIO_MOTOR_PORT, pin2);
    }
}

void Motor_Init(void)
{
    DL_TimerA_startCounter(PWM_MOTOR_INST);
    motor_dir[MOTOR_A] = 0;
    motor_dir[MOTOR_B] = 0;
    Motor_SetDuty(MOTOR_A, 0);
    Motor_SetDuty(MOTOR_B, 0);
}

void Motor_Brake(Motor_ID motor)
{
    if ((motor != MOTOR_A) && (motor != MOTOR_B)) {
        return;
    }

    /* 先关闭 PWM，再将两个方向输入置高并恢复满占空比短刹车。 */
    motor_write_pwm(motor, 0U);
    motor_write_dir(motor, 1U, 1U);
    motor_dir[motor] = 0;
    motor_write_pwm(motor, MOTOR_PWM_PERIOD);
}

void Motor_SetDuty(Motor_ID motor, int16_t duty)
{
    int dir;
    int pwm;

    if ((motor != MOTOR_A) && (motor != MOTOR_B)) {
        return;
    }

    pwm = duty;
    if (pwm > MOTOR_PWM_PERIOD) {
        pwm = MOTOR_PWM_PERIOD;
    } else if (pwm < -MOTOR_PWM_PERIOD) {
        pwm = -MOTOR_PWM_PERIOD;
    }

    if (pwm == 0) {
        /* PWM 和两个方向输入均清零，停止驱动。 */
        motor_write_pwm(motor, 0U);
        motor_write_dir(motor, 0U, 0U);
        motor_dir[motor] = 0;
        return;
    }

    dir = 1;
    if (pwm < 0) {
        dir = -1;
        pwm = -pwm;
    }

    if (motor_dir[motor] != dir) {
        /* 换向前先关闭 PWM，避免方向脚切换时电机受到反向冲击。 */
        motor_write_pwm(motor, 0U);
        if (dir > 0) {
            motor_write_dir(motor, 1U, 0U);
        } else {
            motor_write_dir(motor, 0U, 1U);
        }
        motor_dir[motor] = (int8_t)dir;
    }

    motor_write_pwm(motor, (uint16_t)pwm);
}
