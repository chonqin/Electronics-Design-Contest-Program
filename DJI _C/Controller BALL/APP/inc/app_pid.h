/**
 * @file app_pid.h
 * @brief 通用 PID 控制器基础接口。
 */

#ifndef APP_PID_H
#define APP_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief PID 运算结果。 */
typedef enum
{
    APP_PID_OK = 0,
    APP_PID_ERR_ARG,
    APP_PID_ERR_RANGE,
    APP_PID_ERR_DT,
    APP_PID_ERR_NUM
} app_pid_err_t;

/** @brief PID 控制器运行上下文。 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float out_min;
    float out_max;
    float i_min;
    float i_max;
    float integral;
    float prev_measure;
    uint8_t initialized;
} app_pid_t;

/**
 * @brief 初始化 PID 控制器。
 * @param pid 待初始化的 PID 上下文。
 * @param kp 比例系数。
 * @param ki 积分系数。
 * @param kd 微分系数。
 * @param out_min 控制输出下限。
 * @param out_max 控制输出上限。
 * @param i_min 积分状态下限，必须不大于 0。
 * @param i_max 积分状态上限，必须不小于 0。
 * @return 成功返回 APP_PID_OK，否则返回参数、范围或数值错误。
 */
app_pid_err_t APP_PID_Init(app_pid_t *pid, float kp, float ki, float kd,
                           float out_min, float out_max,
                           float i_min, float i_max);

/**
 * @brief 设置 PID 比例、积分和微分系数。
 * @param pid 已初始化的 PID 上下文。
 * @param kp 比例系数。
 * @param ki 积分系数。
 * @param kd 微分系数。
 * @return 成功返回 APP_PID_OK，否则返回参数或数值错误。
 */
app_pid_err_t APP_PID_SetGains(app_pid_t *pid, float kp, float ki, float kd);

/**
 * @brief 设置控制输出限幅。
 * @param pid 已初始化的 PID 上下文。
 * @param out_min 控制输出下限。
 * @param out_max 控制输出上限。
 * @return 成功返回 APP_PID_OK，否则返回参数或范围错误。
 */
app_pid_err_t APP_PID_SetOutputLimit(app_pid_t *pid,
                                     float out_min, float out_max);

/**
 * @brief 设置积分状态限幅。
 * @param pid 已初始化的 PID 上下文。
 * @param i_min 积分状态下限，必须不大于 0。
 * @param i_max 积分状态上限，必须不小于 0。
 * @return 成功返回 APP_PID_OK，否则返回参数或范围错误。
 */
app_pid_err_t APP_PID_SetIntegralLimit(app_pid_t *pid,
                                       float i_min, float i_max);

/**
 * @brief 清除积分和微分历史状态。
 * @param pid 已初始化的 PID 上下文。
 * @return 成功返回 APP_PID_OK，否则返回 APP_PID_ERR_ARG。
 */
app_pid_err_t APP_PID_Reset(app_pid_t *pid);

/**
 * @brief 执行一次 PID 运算。
 * @param pid 已初始化的 PID 上下文。
 * @param target 目标值。
 * @param measure 当前测量值。
 * @param dt 本次运算与上次运算之间的实际时间，单位为秒，必须大于 0。
 * @param output 输出控制量。
 * @return 成功返回 APP_PID_OK，否则返回参数、时间或数值错误。
 * @note 微分项使用测量值变化计算，形式为 -kd*d(measure)/dt。
 */
app_pid_err_t APP_PID_Update(app_pid_t *pid, float target, float measure,
                             float dt, float *output);

/**
 * @brief Execute one PID update with an externally measured derivative.
 * @param pid Initialized PID context.
 * @param target Target value.
 * @param measure Current measured value.
 * @param derivative External measurement derivative per second.
 * @param dt Elapsed time in seconds, which must be greater than zero.
 * @param output Calculated control output.
 * @return APP_PID_OK on success, otherwise an argument, time, or numeric error.
 * @note The derivative output uses -kd * derivative.
 */
app_pid_err_t APP_PID_UpdateWithDerivative(app_pid_t *pid, float target,
                                           float measure, float derivative,
                                           float dt, float *output);

#ifdef __cplusplus
}
#endif

#endif /* APP_PID_H */
