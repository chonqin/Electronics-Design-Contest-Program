/**
 * @file    imu.h
 * @brief   IMU 姿态融合接口
 */

#ifndef __IMU_H
#define __IMU_H

#include <math.h>
#include <stdint.h>

/**
 * @brief 三轴浮点向量
 */
typedef struct {
    float x;
    float y;
    float z;
} xyz_f_t;

extern xyz_f_t north, west;
extern volatile float yaw[5];

/**
 * @brief 初始化 ICM42688P 和姿态融合状态
 */
void IMU_init(void);

/**
 * @brief 采样 IMU 数据并更新姿态
 * @note  该函数由 ICM42688P 数据就绪中断触发
 */
void IMU_sample(void);

/**
 * @brief 处理 PA16 数据就绪 GPIO 中断
 */
void IMU_dataReadyIrqHandler(void);

/**
 * @brief 重置 IMU 采样时间基准
 */
void IMU_resetTimestamp(void);

/**
 * @brief 获取最新的 yaw、pitch 和 roll 角度
 * @param angles 输出数组，索引 0/1/2 分别对应 yaw/pitch/roll，单位为度
 */
void IMU_getYawPitchRoll(float *angles);

#endif /* __IMU_H */
