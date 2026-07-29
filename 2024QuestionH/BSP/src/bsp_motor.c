/**
 * @file bsp_motor.c
 * @brief AT8236 双路直流电机驱动实现
 *
 * 每个 AT8236 使用同一定时器的两路 PWM 分别驱动 IN1 和 IN2。
 * MOTOR_A 使用 TIMG6，MOTOR_B 使用 TIMA1。
 */
#include "bsp_motor.h"

/**
 * @brief 设置指定电机的 IN1、IN2 PWM 比较值。
 * @param motor 电机选择。
 * @param in1 IN1 比较值，范围 0..MOTOR_PWM_PERIOD。
 * @param in2 IN2 比较值，范围 0..MOTOR_PWM_PERIOD。
 */
static void motor_write(Motor_ID motor, uint16_t in1, uint16_t in2)
{
    if (motor == MOTOR_A) {
        if ((in1 > 0U) && (in2 == 0U)) {
            DL_TimerG_setCaptureCompareValue(
                PWM_MOTOR1_INST, 0U, GPIO_PWM_MOTOR1_C1_IDX);
            DL_TimerG_setCaptureCompareValue(
                PWM_MOTOR1_INST, in1, GPIO_PWM_MOTOR1_C0_IDX);
        } else {
            DL_TimerG_setCaptureCompareValue(
                PWM_MOTOR1_INST, in1, GPIO_PWM_MOTOR1_C0_IDX);
            DL_TimerG_setCaptureCompareValue(
                PWM_MOTOR1_INST, in2, GPIO_PWM_MOTOR1_C1_IDX);
        }
    } else if (motor == MOTOR_B) {
        if ((in1 > 0U) && (in2 == 0U)) {
            DL_TimerA_setCaptureCompareValue(
                PWM_MOTOR2_INST, 0U, GPIO_PWM_MOTOR2_C1_IDX);
            DL_TimerA_setCaptureCompareValue(
                PWM_MOTOR2_INST, in1, GPIO_PWM_MOTOR2_C0_IDX);
        } else {
            DL_TimerA_setCaptureCompareValue(
                PWM_MOTOR2_INST, in1, GPIO_PWM_MOTOR2_C0_IDX);
            DL_TimerA_setCaptureCompareValue(
                PWM_MOTOR2_INST, in2, GPIO_PWM_MOTOR2_C1_IDX);
        }
    }
}

void Motor_Init(void)
{
    DL_TimerG_startCounter(PWM_MOTOR1_INST);
    DL_TimerA_startCounter(PWM_MOTOR2_INST);
    Motor_SetDuty(MOTOR_A, 0);
    Motor_SetDuty(MOTOR_B, 0);
}

void Motor_Brake(Motor_ID motor)
{
    if ((motor != MOTOR_A) && (motor != MOTOR_B)) {
        return;
    }

    /* AT8236 的 IN1、IN2 同时置高，进入短刹车状态。 */
    motor_write(motor, MOTOR_PWM_PERIOD, MOTOR_PWM_PERIOD);
}

void Motor_SetDuty(Motor_ID motor, int16_t duty)
{
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

    /* 左轮与右轮镜像安装，反转 MOTOR_B 极性以统一车体运动方向。 */
    if (motor == MOTOR_B) {
        pwm = -pwm;
    }

    if (pwm > 0) {
        /* 先关闭反向通道，再开启正向 PWM，避免换向时两路短暂同时导通。 */
        motor_write(motor, (uint16_t)pwm, 0U);
    } else if (pwm < 0) {
        /* 负占空比使用 IN2 输出 PWM。 */
        motor_write(motor, 0U, (uint16_t)(-pwm));
    } else {
        /* 两个输入均为低电平，停止驱动。 */
        motor_write(motor, 0U, 0U);
    }
}
