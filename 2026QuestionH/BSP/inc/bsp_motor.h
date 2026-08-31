/**
 * @file bsp_motor.h
 * @brief TB6612 双路直流电机驱动接口
 */
#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

#include "ti_msp_dl_config.h"

typedef enum {
    MOTOR_A = 0, /**< 电机1：PA15/PWMA、PB10/AIN1、PB13/AIN2，对应编码器 E1。 */
    MOTOR_B = 1  /**< 电机2：PA24/PWMB、PB15/BIN1、PB16/BIN2，对应编码器 E2。 */
} Motor_ID;

/**
 * @brief PWM 周期值，对应 SysConfig 中 TIMA1 的 period 配置。
 */
#define MOTOR_PWM_PERIOD    4000

/**
 * @brief 初始化电机 PWM，并将双电机输出清零
 */
void Motor_Init(void);

/**
 * @brief 对单个电机执行短刹车。
 * @param motor 电机选择，取值为 MOTOR_A 或 MOTOR_B。
 */
void Motor_Brake(Motor_ID motor);

/**
 * @brief 设置单个电机占空比
 * @param motor 电机选择，取值为 MOTOR_A 或 MOTOR_B
 * @param duty 占空比命令，范围 [-MOTOR_PWM_PERIOD, MOTOR_PWM_PERIOD]
 * @return 无返回值
 *
 * @details
 * 两路电机不做额外极性补偿，duty 正负直接决定各自的方向输入组合。duty = 0 时
 * PWM 清零且两个方向输入均置低。STBY 由硬件保持高电平。
 */
void Motor_SetDuty(Motor_ID motor, int16_t duty);

#endif
