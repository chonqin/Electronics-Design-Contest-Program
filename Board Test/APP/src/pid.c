/**
 * @file pid.c
 * @brief 基础位置式 PID 控制器实现
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
    pid->sum = PID_Limit(pid->sum, out_min, out_max);
    pid->out = PID_Limit(pid->out, out_min, out_max);
}

void PID_SetTarget(PID *pid, float target)
{
    if (pid == 0) {
        return;
    }

    pid->target = target;
}

float PID_Calc(PID *pid, float fb)
{
    float p;
    float d;

    if (pid == 0) {
        return 0.0f;
    }

    pid->err = pid->target - fb;
    pid->sum += pid->ki * pid->err;
    pid->sum = PID_Limit(pid->sum, pid->out_min, pid->out_max);

    p = pid->kp * pid->err;
    d = pid->kd * (pid->err - pid->last_err);
    pid->out = PID_Limit(p + pid->sum + d, pid->out_min, pid->out_max);
    pid->last_err = pid->err;

    return pid->out;
}

float PID_CalcTarget(PID *pid, float target, float fb)
{
    PID_SetTarget(pid, target);

    return PID_Calc(pid, fb);
}
