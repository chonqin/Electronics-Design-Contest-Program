/**
 * @file car.h
 * @brief 底盘 duty 控制接口
 */
#ifndef _CAR_H_
#define _CAR_H_

#include <stdint.h>

/** @brief 底盘控制模式 */
typedef enum {
    CAR_MODE_STOP = 0,
    CAR_MODE_TRACK,
    CAR_MODE_TURN,
    CAR_MODE_HEADING
} Car_Mode;

/** @brief 可在线调节的 PID 控制器 */
typedef enum {
    CAR_PID_TRACK = 0,
    CAR_PID_YAW
} Car_PidId;

/** @brief 可在线调节的 PID 参数项 */
typedef enum {
    CAR_PID_KP = 0,
    CAR_PID_KI,
    CAR_PID_KD
} Car_PidTerm;

/** @brief PID 参数快照 */
typedef struct {
    float kp;
    float ki;
    float kd;
} Car_PidParams;

/** @brief 任务与调试使用的底盘状态 */
typedef struct {
    Car_Mode mode;
    uint8_t done;
    uint8_t track_mask; /**< bit0-bit7 对应 X1-X8，1 表示未检测到黑线 */
    int track_pos;
    int enc_l;
    int enc_r;
    int duty_l;
    int duty_r;
    float yaw;
} Car_Status;

/** @brief 初始化电机、编码器和控制器 */
void Car_Init(void);

/** @brief 停止底盘并清空控制状态 */
void Car_Stop(void);

/**
 * @brief 设置循迹基础 PWM duty
 * @param duty 循迹基础 PWM duty
 */
void Car_SetTrack(int duty);

/**
 * @brief 设置循迹丢线后是否按最后位置自动寻线
 * @param enable 1 表示启用自动寻线，0 表示丢线后立即制动
 */
void Car_SetTrackLostSearch(uint8_t enable);

/**
 * @brief 设置相对转向角
 * @param deg 相对当前航向的目标角度，单位为度
 */
void Car_SetTurnAngle(float deg);

/**
 * @brief 设置航向保持参数
 * @param duty 基础 PWM duty
 * @param yaw 目标航向角，单位为度
 */
void Car_SetHeading(int duty, float yaw);

/** @brief 执行一次 20 ms 底盘控制更新 */
void Car_Update(void);

/**
 * @brief 读取底盘状态
 * @param st 输出状态指针
 */
void Car_GetStatus(Car_Status *st);

/**
 * @brief 读取指定 PID 的参数
 * @param id PID 控制器标识
 * @param param 输出参数快照
 */
void Car_GetPidParams(Car_PidId id, Car_PidParams *param);

/**
 * @brief 修改指定 PID 的一个参数项
 * @param id PID 控制器标识
 * @param term 参数项
 * @param value 新参数值
 * @return 1 表示成功，0 表示参数无效
 */
uint8_t Car_SetPidParam(Car_PidId id, Car_PidTerm term, float value);

#endif
