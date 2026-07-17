/**
 * @file car.h
 * @brief 小车运动控制层接口
 */
#ifndef _CAR_H_
#define _CAR_H_

/**
 * @brief 初始化小车电机、编码器和轮速 PID
 */
void Car_Init(void);

/**
 * @brief 停止双轮并清空速度控制运行状态
 */
void Car_Stop(void);

/**
 * @brief 设置左右轮目标速度
 * @param left 左轮目标速度，单位为单次控制周期编码器计数
 * @param right 右轮目标速度，单位为单次控制周期编码器计数
 */
void Car_SetWheelSpeed(int left, int right);

/**
 * @brief 设置小车线速度和转向速度
 * @param linear 前进速度，单位为单次控制周期编码器计数
 * @param turn 转向差速，单位为单次控制周期编码器计数
 */
void Car_SetSpeed(int linear, int turn);

/**
 * @brief 更新轮速反馈并将 PID 输出作用到电机
 */
void Car_Update(void);

/**
 * @brief 获取左轮目标速度
 * @return 左轮目标速度
 */
int Car_GetTargetLeft(void);

/**
 * @brief 获取右轮目标速度
 * @return 右轮目标速度
 */
int Car_GetTargetRight(void);

/**
 * @brief 获取左轮最新实测速度
 * @return 左轮速度，单位为单次控制周期编码器计数
 */
int Car_GetSpeedLeft(void);

/**
 * @brief 获取右轮最新实测速度
 * @return 右轮速度，单位为单次控制周期编码器计数
 */
int Car_GetSpeedRight(void);

/**
 * @brief 获取左电机最新 PWM 占空比输出
 * @return 左电机占空比
 */
int Car_GetDutyLeft(void);

/**
 * @brief 获取右电机最新 PWM 占空比输出
 * @return 右电机占空比
 */
int Car_GetDutyRight(void);

#endif
