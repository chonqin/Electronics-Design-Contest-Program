/**
 * @file bsp_motor.h
 * @brief TB6612FNG 双路直流电机驱动接口
 */
#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "ti_msp_dl_config.h"

typedef enum {
    MOTOR_A = 0,
    MOTOR_B = 1
} Motor_ID;

/**
 * @brief PWM 周期值，对应 SysConfig 中 TIMA1 的 period 配置
 */
#define MOTOR_PWM_PERIOD    4000

/**
 * @brief 初始化电机 PWM，并将双电机输出清零
 */
void Motor_Init(void);

/**
 * @brief 设置单个电机占空比
 * @param motor 电机选择，取值为 MOTOR_A 或 MOTOR_B
 * @param duty 占空比命令，范围 [-MOTOR_PWM_PERIOD, MOTOR_PWM_PERIOD]
 * @return 无返回值
 *
 * @details
 * duty > 0 时电机正转，duty < 0 时电机反转，duty = 0 时电机停止。
 */
void Motor_SetDuty(Motor_ID motor, int16_t duty);

#endif
