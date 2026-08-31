/**
 * @file task2.c
 * @brief TASK2 vision reception, parsing, and center-position control.
 */

#include "task2.h"

#include <stddef.h>

app_control_err_t TASK2_Init(task2_t *task, app_control_t *control)
{
    if ((task == NULL) || (control == NULL))
    {
        return APP_CONTROL_ERR_ARG;
    }

    task->control = control;
    return APP_CONTROL_OK;
}

app_control_err_t TASK2_Start(task2_t *task)
{
    if ((task == NULL) || (task->control == NULL))
    {
        return APP_CONTROL_ERR_ARG;
    }

    return APP_Control_Start(task->control);
}

app_control_err_t TASK2_Stop(task2_t *task)
{
    if ((task == NULL) || (task->control == NULL))
    {
        return APP_CONTROL_ERR_ARG;
    }

    return APP_Control_Stop(task->control);
}

void TASK2_RxByte(task2_t *task, uint8_t byte)
{
    if ((task == NULL) || (task->control == NULL))
    {
        return;
    }

    /** @brief Keep interrupt work limited to byte queuing; parse in the main loop. */
    APP_Control_RxByte(task->control, byte);
}

app_control_err_t TASK2_ProcessRx(task2_t *task, uint32_t now_ms)
{
    if ((task == NULL) || (task->control == NULL))
    {
        return APP_CONTROL_ERR_ARG;
    }

    return APP_Control_ProcessRx(task->control, now_ms);
}

app_control_err_t TASK2_Run(task2_t *task, uint32_t now_ms)
{
    if ((task == NULL) || (task->control == NULL))
    {
        return APP_CONTROL_ERR_ARG;
    }

    /** @brief The TASK2 tick performs PID control and sends the next motor angle. */
    return APP_Control_Tick(task->control, now_ms);
}

uint8_t TASK2_IsStopped(const task2_t *task)
{
    if ((task == NULL) || (task->control == NULL))
    {
        return 0U;
    }

    return (uint8_t)((task->control->state == APP_CONTROL_DISABLED) &&
                     (task->control->motor_op == APP_CONTROL_MOTOR_NONE));
}
