/**
 * @file bsp_motor.c
 * @brief TB6612FNG双路直流电机驱动实现
 *
 * TB6612真值表:
 *   IN1=H, IN2=L → 正转
 *   IN1=L, IN2=H → 反转
 *   IN1=L, IN2=L → 滑行停止
 *   IN1=H, IN2=H → 刹车
 */
#include "bsp_motor.h"

void Motor_Init(void)
{
    DL_TimerA_startCounter(PWM_MOTOR_INST);
    Motor_Stop(MOTOR_A);
    Motor_Stop(MOTOR_B);
}

void Motor_SetDuty(Motor_ID motor, int16_t duty)
{
    Motor_Dir dir;
    uint16_t pwm_val;
    int val;

    if (duty > 0) {
        dir = MOTOR_DIR_FORWARD;
        val = duty;
    } else if (duty < 0) {
        dir = MOTOR_DIR_BACKWARD;
        val = -duty;
    } else {
        Motor_Stop(motor);
        return;
    }

    /* duty 绝对值作为 PWM 比较值，超过周期时按满占空比限幅。 */
    if (val > MOTOR_PWM_PERIOD) {
        val = MOTOR_PWM_PERIOD;
    }
    pwm_val = (uint16_t)val;

    /* 设置方向引脚并更新PWM占空比 */
    if (motor == MOTOR_A) {
        if (dir == MOTOR_DIR_FORWARD) {
            DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN2_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN2_PIN);
        }
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, pwm_val, DL_TIMER_CC_0_INDEX);
    } else {
        if (dir == MOTOR_DIR_FORWARD) {
            DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN2_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN2_PIN);
        }
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, pwm_val, DL_TIMER_CC_1_INDEX);
    }
}

void Motor_Stop(Motor_ID motor)
{
    if (motor == MOTOR_A) {
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN1_PIN | GPIO_MOTOR_AIN2_PIN);
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_0_INDEX);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN1_PIN | GPIO_MOTOR_BIN2_PIN);
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_1_INDEX);
    }
}

void Motor_Brake(Motor_ID motor)
{
    if (motor == MOTOR_A) {
        DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN1_PIN | GPIO_MOTOR_AIN2_PIN);
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_0_INDEX);
    } else {
        DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN1_PIN | GPIO_MOTOR_BIN2_PIN);
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_1_INDEX);
    }
}
