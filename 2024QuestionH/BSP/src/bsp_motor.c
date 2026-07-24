/**
 * @file bsp_motor.c
 * @brief TB6612FNG 双路直流电机驱动实现
 *
 * TB6612 真值表:
 *   IN1=H, IN2=L → 正转
 *   IN1=L, IN2=H → 反转
 *   IN1=L, IN2=L → 滑行停止
 */
#include "bsp_motor.h"

void Motor_Init(void)
{
    DL_TimerA_startCounter(PWM_MOTOR_INST);
    Motor_SetDuty(MOTOR_A, 0);
    Motor_SetDuty(MOTOR_B, 0);
}

void Motor_SetDuty(Motor_ID motor, int16_t duty)
{
    uint32_t pin1;
    uint32_t pin2;
    DL_TIMER_CC_INDEX cc_idx;
    int pwm;

    if (motor == MOTOR_A) {
        pin1 = GPIO_MOTOR_AIN1_PIN;
        pin2 = GPIO_MOTOR_AIN2_PIN;
        cc_idx = DL_TIMER_CC_0_INDEX;
    } else if (motor == MOTOR_B) {
        pin1 = GPIO_MOTOR_BIN1_PIN;
        pin2 = GPIO_MOTOR_BIN2_PIN;
        cc_idx = DL_TIMER_CC_1_INDEX;
    } else {
        return;
    }

    pwm = duty;
    if (pwm > MOTOR_PWM_PERIOD) {
        pwm = MOTOR_PWM_PERIOD;
    } else if (pwm < -MOTOR_PWM_PERIOD) {
        pwm = -MOTOR_PWM_PERIOD;
    }

    if (pwm > 0) {
        // 正占空比对应正转方向，PWM 值直接写入比较寄存器。
        DL_GPIO_setPins(GPIO_MOTOR_PORT, pin1);
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, pin2);
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, (uint16_t)pwm, cc_idx);
    } else if (pwm < 0) {
        // 负占空比改为反转，同时对 PWM 取绝对值输出。
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, pin1);
        DL_GPIO_setPins(GPIO_MOTOR_PORT, pin2);
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, (uint16_t)(-pwm), cc_idx);
    } else {
        /* 占空比为 0 时释放双方向脚，保持滑行停止。 */
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, pin1 | pin2);
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, 0U, cc_idx);
    }
}
