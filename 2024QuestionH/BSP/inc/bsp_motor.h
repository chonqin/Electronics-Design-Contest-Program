/**
 * @file bsp_motor.h
 * @brief TB6612FNG双路直流电机驱动接口
 */
#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "ti_msp_dl_config.h"

typedef enum {
    MOTOR_A = 0,
    MOTOR_B = 1
} Motor_ID;

typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_BACKWARD = 1,
    MOTOR_DIR_STOP = 2,
    MOTOR_DIR_BRAKE = 3
} Motor_Dir;

/* PWM周期值，对应sysconfig中TIMA1的period配置 */
#define MOTOR_PWM_PERIOD    4000

/**
 * @brief 初始化电机，启动PWM定时器并设置为停止状态
 */
void Motor_Init(void);

/**
 * @brief 设置电机速度与方向
 * @param motor 电机选择 MOTOR_A / MOTOR_B
 * @param duty PWM 占空比命令，范围 [-4000, 4000]，正值正转，负值反转，0停止
 */
void Motor_SetDuty(Motor_ID motor, int16_t duty);

/**
 * @brief 电机滑行停止（IN1=0, IN2=0）
 */
void Motor_Stop(Motor_ID motor);

/**
 * @brief 电机刹车（IN1=1, IN2=1）
 */
void Motor_Brake(Motor_ID motor);

#endif
