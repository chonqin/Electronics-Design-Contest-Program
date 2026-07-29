/**
 * @file bsp_motor.h
 * @brief AT8236 双路直流电机驱动接口
 */
#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

#include "ti_msp_dl_config.h"

typedef enum {
    MOTOR_A = 0, /**< 电机1：PB10/IN1、PB11/IN2，对应编码器 E1。 */
    MOTOR_B = 1  /**< 电机2：PA15/IN1、PA24/IN2，对应编码器 E2。 */
} Motor_ID;

/**
 * @brief PWM 周期值，对应 SysConfig 中 TIMG6 和 TIMA1 的 period 配置。
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
 * 两路电机以小车物理运动方向为准；由于左右电机镜像安装，MOTOR_B 的输出极性
 * 在底层自动反转。相同符号的 duty 会使左右轮朝相同的车体方向转动，duty = 0
 * 时两个输入均置低。
 */
void Motor_SetDuty(Motor_ID motor, int16_t duty);

#endif
