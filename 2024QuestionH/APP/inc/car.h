/**
 * @file car.h
 * @brief 通用底盘控制层接口
 */
#ifndef _CAR_H_
#define _CAR_H_

#include <stdint.h>

/**
 * @brief 底盘控制模式
 */
typedef enum {
    CAR_MODE_STOP = 0,
    CAR_MODE_SPEED,
    CAR_MODE_TRACK,
    CAR_MODE_TURN,
    CAR_MODE_HEADING
} Car_Mode;

/**
 * @brief 底盘调试状态
 */
typedef struct {
    Car_Mode mode;
    uint8_t done;
    uint8_t track_mask;
    int track_pos;
    int base;
    int turn;
    int target_l;
    int target_r;
    int speed_l;
    int speed_r;
    int duty_l;
    int duty_r;
    float yaw;
    float yaw_tar;
} Car_Status;

/**
 * @brief PID parameter snapshot for debug display.
 */
typedef struct {
    float kp;
    float ki;
    float kd;
} Car_PidParam;

/**
 * @brief 初始化底盘控制层
 */
void Car_Init(void);

/**
 * @brief 停止底盘并清空控制状态
 */
void Car_Stop(void);

/**
 * @brief 设置底盘控制模式
 * @param mode 控制模式
 */
void Car_SetMode(Car_Mode mode);

/**
 * @brief 设置左右轮目标速度并切换到速度模式
 * @param left 左轮目标速度
 * @param right 右轮目标速度
 */
void Car_SetWheelSpeed(int left, int right);

/**
 * @brief 设置底盘基础速度和差速并切换到速度模式
 * @param base 基础速度
 * @param turn 差速转向量
 */
void Car_SetSpeed(int base, int turn);

/**
 * @brief 设置循迹基础速度并切换到循迹模式
 * @param base 循迹基础速度
 */
void Car_SetTrack(int base);

/**
 * @brief 设置相对转向角并切换到定角转向模式
 * @param deg 相对当前航向的目标角度，单位为度
 */
void Car_SetTurnAngle(float deg);

/**
 * @brief 设置直行基础速度和目标航向角并切换到航向保持模式
 * @param base 直行基础速度
 * @param yaw 目标航向角，单位为度
 */
void Car_SetHeading(int base, float yaw);

/**
 * @brief 按当前模式更新底盘控制输出
 *
 * @details
 * 该函数应由上层以固定控制周期调用。
 */
void Car_Update(void);

/**
 * @brief 查询当前模式是否完成
 * @return 1 表示当前动作完成，0 表示未完成
 */
uint8_t Car_IsDone(void);

/**
 * @brief 读取当前底盘调试状态
 * @param st 输出状态指针
 */
void Car_GetStatus(Car_Status *st);

/**
 * @brief Read PID parameters used by current car mode.
 * @param param Output PID parameter snapshot.
 */
void Car_GetPidParam(Car_PidParam *param);

#endif
