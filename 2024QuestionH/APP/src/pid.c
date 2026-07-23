/**
 * @file pid.c
 * @brief 通用 PID 控制器实现
 */
#include "pid.h"

/**
 * @brief 将数值限制到闭区间
 * @param val 输入值
 * @param min 下限
 * @param max 上限
 * @return 限幅后的数值
 */
static float PID_Limit(float val, float min, float max)
{
    if (val > max) {
        return max;
    }

    if (val < min) {
        return min;
    }

    return val;
}

/**
 * @brief 限制积分项，避免位置式 PID 长时间饱和
 * @param pid PID 控制器实例
 */
static void PID_LimitSum(PID *pid)
{
    float min;
    float max;
    float tmp;

    if (pid->ki == 0.0f) {
        pid->sum = 0.0f;
        return;
    }

    min = pid->out_min / pid->ki;
    max = pid->out_max / pid->ki;

    if (min > max) {
        tmp = min;
        min = max;
        max = tmp;
    }

    pid->sum = PID_Limit(pid->sum, min, max);
}

void PID_Init(PID *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    float tmp;

    if (pid == 0) {
        return;
    }

    if (out_min > out_max) {
        tmp = out_min;
        out_min = out_max;
        out_max = tmp;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
    PID_Reset(pid);
}

void PID_Reset(PID *pid)
{
    if (pid == 0) {
        return;
    }

    pid->err = 0.0f;
    pid->last_err = 0.0f;
    pid->prev_err = 0.0f;
    pid->sum = 0.0f;
    pid->out = 0.0f;
}

void PID_SetParam(PID *pid, float kp, float ki, float kd)
{
    if (pid == 0) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetLimit(PID *pid, float out_min, float out_max)
{
    float tmp;

    if (pid == 0) {
        return;
    }

    if (out_min > out_max) {
        tmp = out_min;
        out_min = out_max;
        out_max = tmp;
    }

    pid->out_min = out_min;
    pid->out_max = out_max;
    PID_LimitSum(pid);
    pid->out = PID_Limit(pid->out, out_min, out_max);
}

void PID_SetTarget(PID *pid, float target)
{
    if (pid == 0) {
        return;
    }

    pid->target = target;
}

float PID_Calc(PID *pid, float actual)
{
    float out;

    if (pid == 0) {
        return 0.0f;
    }

    pid->err = pid->target - actual;
    pid->sum += pid->err;
    PID_LimitSum(pid);

    /* 位置式 PID 输出执行量，可直接作为 duty、角速度等控制量使用。 */
    out = pid->kp * pid->err;
    out += pid->ki * pid->sum;
    out += pid->kd * (pid->err - pid->last_err);
    pid->out = PID_Limit(out, pid->out_min, pid->out_max);

    pid->prev_err = pid->last_err;
    pid->last_err = pid->err;

    return pid->out;
}

float PID_CalcTarget(PID *pid, float target, float actual)
{
    PID_SetTarget(pid, target);

    return PID_Calc(pid, actual);
}

float PID_CalcInc(PID *pid, float actual)
{
    float inc;

    if (pid == 0) {
        return 0.0f;
    }

    pid->err = pid->target - actual;

    inc = pid->kp * (pid->err - pid->last_err);
    inc += pid->ki * pid->err;
    inc += pid->kd * (pid->err - 2.0f * pid->last_err + pid->prev_err);
    pid->out = PID_Limit(inc, pid->out_min, pid->out_max);

    pid->prev_err = pid->last_err;
    pid->last_err = pid->err;

    return pid->out;
}

float PID_CalcIncTarget(PID *pid, float target, float actual)
{
    PID_SetTarget(pid, target);

    return PID_CalcInc(pid, actual);
}
