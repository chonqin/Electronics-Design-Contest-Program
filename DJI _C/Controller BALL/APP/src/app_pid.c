/**
 * @file app_pid.c
 * @brief 通用 PID 控制器基础实现。
 */

#include "app_pid.h"

#include <float.h>
#include <stddef.h>

static uint8_t APP_PID_IsFinite(float value)
{
    return (uint8_t)((value == value) &&
                     (value <= FLT_MAX) && (value >= -FLT_MAX));
}

static float APP_PID_Clamp(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }

    return value;
}

static uint8_t APP_PID_LimitValid(float min, float max)
{
    return (uint8_t)(APP_PID_IsFinite(min) && APP_PID_IsFinite(max) &&
                     (min <= max));
}

static uint8_t APP_PID_IntegralLimitValid(float min, float max)
{
    return (uint8_t)(APP_PID_LimitValid(min, max) &&
                     (min <= 0.0f) && (max >= 0.0f));
}

app_pid_err_t APP_PID_Init(app_pid_t *pid, float kp, float ki, float kd,
                           float out_min, float out_max,
                           float i_min, float i_max)
{
    if (pid == NULL)
    {
        return APP_PID_ERR_ARG;
    }
    if (!APP_PID_IsFinite(kp) || !APP_PID_IsFinite(ki) ||
        !APP_PID_IsFinite(kd))
    {
        return APP_PID_ERR_NUM;
    }
    if (!APP_PID_LimitValid(out_min, out_max) ||
        !APP_PID_IntegralLimitValid(i_min, i_max))
    {
        return APP_PID_ERR_RANGE;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->i_min = i_min;
    pid->i_max = i_max;
    pid->integral = 0.0f;
    pid->prev_measure = 0.0f;
    pid->initialized = 0U;

    return APP_PID_OK;
}

app_pid_err_t APP_PID_SetGains(app_pid_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL)
    {
        return APP_PID_ERR_ARG;
    }
    if (!APP_PID_IsFinite(kp) || !APP_PID_IsFinite(ki) ||
        !APP_PID_IsFinite(kd))
    {
        return APP_PID_ERR_NUM;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    return APP_PID_OK;
}

app_pid_err_t APP_PID_SetOutputLimit(app_pid_t *pid,
                                     float out_min, float out_max)
{
    if (pid == NULL)
    {
        return APP_PID_ERR_ARG;
    }
    if (!APP_PID_LimitValid(out_min, out_max))
    {
        return APP_PID_ERR_RANGE;
    }

    pid->out_min = out_min;
    pid->out_max = out_max;

    return APP_PID_OK;
}

app_pid_err_t APP_PID_SetIntegralLimit(app_pid_t *pid,
                                       float i_min, float i_max)
{
    if (pid == NULL)
    {
        return APP_PID_ERR_ARG;
    }
    if (!APP_PID_IntegralLimitValid(i_min, i_max))
    {
        return APP_PID_ERR_RANGE;
    }

    pid->i_min = i_min;
    pid->i_max = i_max;
    pid->integral = APP_PID_Clamp(pid->integral, i_min, i_max);

    return APP_PID_OK;
}

app_pid_err_t APP_PID_Reset(app_pid_t *pid)
{
    if (pid == NULL)
    {
        return APP_PID_ERR_ARG;
    }

    pid->integral = 0.0f;
    pid->prev_measure = 0.0f;
    pid->initialized = 0U;

    return APP_PID_OK;
}

/**
 * @brief Apply one PID update with a prepared measurement derivative.
 * @param pid Initialized PID context.
 * @param target Target value.
 * @param measure Current measured value.
 * @param derivative Measurement derivative per second.
 * @param dt Elapsed time in seconds.
 * @param output Calculated control output.
 * @return APP_PID_OK on success, otherwise an argument, time, or numeric error.
 */
static app_pid_err_t APP_PID_Apply(app_pid_t *pid, float target,
                                   float measure, float derivative,
                                   float dt, float *output)
{
    float error;
    float next_integral;
    float raw_output;

    if ((pid == NULL) || (output == NULL))
    {
        return APP_PID_ERR_ARG;
    }
    if (!APP_PID_IsFinite(target) || !APP_PID_IsFinite(measure) ||
        !APP_PID_IsFinite(derivative))
    {
        return APP_PID_ERR_NUM;
    }
    if (!APP_PID_IsFinite(dt) || (dt <= 0.0f))
    {
        return APP_PID_ERR_DT;
    }

    error = target - measure;
    next_integral = APP_PID_Clamp(pid->integral + error * dt,
                                  pid->i_min, pid->i_max);
    raw_output = (pid->kp * error) + (pid->ki * next_integral) -
                 (pid->kd * derivative);
    if (!APP_PID_IsFinite(raw_output))
    {
        return APP_PID_ERR_NUM;
    }

    /* 输出饱和且误差仍推动输出向饱和方向变化时，保持积分状态。 */
    if (((raw_output > pid->out_max) && (error > 0.0f)) ||
        ((raw_output < pid->out_min) && (error < 0.0f)))
    {
        next_integral = pid->integral;
        raw_output = (pid->kp * error) + (pid->ki * next_integral) -
                     (pid->kd * derivative);
    }

    if (!APP_PID_IsFinite(raw_output))
    {
        return APP_PID_ERR_NUM;
    }

    pid->integral = next_integral;
    pid->prev_measure = measure;
    pid->initialized = 1U;
    *output = APP_PID_Clamp(raw_output, pid->out_min, pid->out_max);

    return APP_PID_OK;
}

app_pid_err_t APP_PID_Update(app_pid_t *pid, float target, float measure,
                             float dt, float *output)
{
    float derivative = 0.0f;

    if (pid == NULL)
    {
        return APP_PID_ERR_ARG;
    }
    if (pid->initialized != 0U)
    {
        if (!APP_PID_IsFinite(dt) || (dt <= 0.0f))
        {
            return APP_PID_ERR_DT;
        }
        derivative = (measure - pid->prev_measure) / dt;
    }

    return APP_PID_Apply(pid, target, measure, derivative, dt, output);
}

app_pid_err_t APP_PID_UpdateWithDerivative(app_pid_t *pid, float target,
                                           float measure, float derivative,
                                           float dt, float *output)
{
    return APP_PID_Apply(pid, target, measure, derivative, dt, output);
}
