/**
 * @file pid.h
 * @brief 增量式 PID 控制器接口
 */
#ifndef _PID_H_
#define _PID_H_

typedef struct {
    float kp;
    float ki;
    float kd;
    float target;
    float err;
    float last_err;
    float prev_err;
    float sum;
    float out;
    float out_min;
    float out_max;
} PID;

/**
 * @brief 初始化 PID 控制器
 * @param pid PID 控制器实例
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param out_min 输出下限
 * @param out_max 输出上限
 */
void PID_Init(PID *pid, float kp, float ki, float kd, float out_min, float out_max);

/**
 * @brief 重置 PID 运行状态，但不修改参数和限幅
 * @param pid PID 控制器实例
 */
void PID_Reset(PID *pid);

/**
 * @brief 更新 PID 参数
 * @param pid PID 控制器实例
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PID_SetParam(PID *pid, float kp, float ki, float kd);

/**
 * @brief 更新 PID 输出限幅
 * @param pid PID 控制器实例
 * @param out_min 输出下限
 * @param out_max 输出上限
 */
void PID_SetLimit(PID *pid, float out_min, float out_max);

/**
 * @brief 更新 PID 目标值
 * @param pid PID 控制器实例
 * @param target 目标值
 */
void PID_SetTarget(PID *pid, float target);

/**
 * @brief 根据当前目标计算增量式 PID 输出
 * @param pid PID 控制器实例
 * @param fb 反馈值
 * @return 限幅后的输出增量
 */
float PID_CalcInc(PID *pid, float fb);

/**
 * @brief 更新目标后计算增量式 PID 输出
 * @param pid PID 控制器实例
 * @param target 目标值
 * @param fb 反馈值
 * @return 限幅后的输出增量
 */
float PID_CalcIncTarget(PID *pid, float target, float fb);

#endif
