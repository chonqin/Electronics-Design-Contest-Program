/**
 * @file    imu.h
 * @brief   IMU姿态解算头文件(AHRS算法)
 * @note    基于四元数的六轴融合姿态解算
 */

#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>
#include <math.h>

/**
 * @brief 三轴浮点数据结构体
 */
typedef struct {
    float x;
    float y;
    float z;
} xyz_f_t;

/* 外部变量声明 */
extern xyz_f_t north, west;
extern volatile float yaw[5];

/**
 * @brief  IMU初始化
 * @note   初始化ICM42688和四元数
 */
void IMU_init(void);

/**
 * @brief  IMU数据采样(在定时中断中调用)
 * @note   仅读取SPI数据,不做解算
 */
void IMU_sample(void);

/**
 * @brief  重置IMU时间戳
 * @note   在定时器启动、所有外设初始化完成后调用一次，
 *         防止初始化耗时导致首次解算 dt 过大引发震荡
 */
void IMU_resetTimestamp(void);

/**
 * @brief  更新姿态并获取欧拉角
 * @param  angles 存放姿态角的数组
 *         angles[0]: yaw(航向角, 单位:度)
 *         angles[1]: pitch(俯仰角, 单位:度)
 *         angles[2]: roll(滚转角, 单位:度)
 */
void IMU_getYawPitchRoll(float* angles);

#endif /* __IMU_H */
