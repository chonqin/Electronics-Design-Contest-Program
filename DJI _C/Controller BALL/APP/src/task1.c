/**
 * @file task1.c
 * @brief TASK1 vision reception, parsing, and center-position control.
 */

#include "task1.h"

#include <stddef.h>

app_control_err_t TASK1_Init(task1_t *task, app_control_t *control)
{
    if ((task == NULL) || (control == NULL))
    {
        return APP_CONTROL_ERR_ARG;
    }

    task->control = control;
    return APP_CONTROL_OK;
}

void TASK1_SetMotorEnabled(task1_t *task, uint8_t enabled)
{
    if ((task == NULL) || (task->control == NULL))
    {
        return;
    }

    APP_Control_AdoptMotorState(task->control, enabled);
}

app_control_err_t TASK1_Start(task1_t *task)
{
    if ((task == NULL) || (task->control == NULL))
    {
        return APP_CONTROL_ERR_ARG;
    }

    return APP_Control_Start(task->control);
}

app_control_err_t TASK1_Stop(task1_t *task)
{
    if ((task == NULL) || (task->control == NULL))
    {
        return APP_CONTROL_ERR_ARG;
    }

    return APP_Control_Stop(task->control);
}

void TASK1_RxByte(task1_t *task, uint8_t byte)
{
    if ((task == NULL) || (task->control == NULL))
    {
        return;
    }

    /** @brief Keep interrupt work limited to byte queuing; parse in the main loop. */
    APP_Control_RxByte(task->control, byte);
}

app_control_err_t TASK1_ProcessRx(task1_t *task, uint32_t now_ms)
{
    if ((task == NULL) || (task->control == NULL))
    {
        return APP_CONTROL_ERR_ARG;
    }

    return APP_Control_ProcessRx(task->control, now_ms);
}

app_control_err_t TASK1_Run(task1_t *task, uint32_t now_ms)
{
    if ((task == NULL) || (task->control == NULL))
    {
        return APP_CONTROL_ERR_ARG;
    }

    /** @brief The TASK1 tick uses the same PID and motor transaction path as TASK2. */
    return APP_Control_Tick(task->control, now_ms);
}

uint8_t TASK1_IsStopped(const task1_t *task)
{
    if ((task == NULL) || (task->control == NULL))
    {
        return 0U;
    }

    return (uint8_t)((task->control->state == APP_CONTROL_DISABLED) &&
                     (task->control->motor_op == APP_CONTROL_MOTOR_NONE));
}
