/**
 * @file app_control.c
 * @brief 视觉到电机的平衡控制框架实现。
 */

#include "app_control.h"

#include <float.h>
#include <stddef.h>

#define APP_CONTROL_TWO_PI 6.28318530717958647692f

static uint8_t APP_Control_IsFinite(float value)
{
    return (uint8_t)((value == value) &&
                     (value <= FLT_MAX) && (value >= -FLT_MAX));
}

static float APP_Control_Clamp(float value, float min, float max)
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

static float APP_Control_ToMotorAngle(float angle)
{
    if (angle < 0.0f)
    {
        return APP_CONTROL_TWO_PI + angle;
    }

    return angle;
}

static uint16_t APP_Control_NextIndex(uint16_t index)
{
    ++index;
    if (index >= APP_CONTROL_RX_RING_SIZE)
    {
        index = 0U;
    }

    return index;
}

static void APP_Control_ResetHistory(app_control_t *ctl)
{
    (void)APP_PID_Reset(ctl->pid);
    ctl->frame_seen = 0U;
    ctl->vision_valid = 0U;
    ctl->last_ctrl_ms = 0U;
}

/**
 * @brief Discard buffered and partial vision frames before a new task run.
 * @param ctl Visual balance controller context.
 */
static void APP_Control_ResetRx(app_control_t *ctl)
{
    ctl->rx_tail = ctl->rx_head;
    ctl->rx_overflow = 0U;
    ctl->frame_len = 0U;
    ctl->timing_end = 0U;
}

static app_control_err_t APP_Control_Fault(app_control_t *ctl,
                                           app_control_err_t error)
{
    if (ctl->motor_op != APP_CONTROL_MOTOR_NONE)
    {
        (void)BSP_MOTOR_Cancel(ctl->motor);
        ctl->motor_op = APP_CONTROL_MOTOR_NONE;
    }

    ctl->state = APP_CONTROL_FAULT;
    ctl->stop_requested = 0U;
    APP_Control_ResetHistory(ctl);
    if (ctl->motor_enabled != 0U)
    {
        ctl->motor_err = BSP_MOTOR_Disable(ctl->motor);
        if (ctl->motor_err == BSP_MOTOR_OK)
        {
            ctl->motor_op = APP_CONTROL_MOTOR_DISABLE;
        }
        else
        {
            ctl->motor_enabled = 0U;
        }
    }

    return error;
}

static app_control_err_t APP_Control_ProcessMotor(app_control_t *ctl,
                                                   uint32_t now_ms,
                                                   uint8_t *busy)
{
    app_control_motor_op_t op;

    *busy = 0U;
    if (ctl->motor_op == APP_CONTROL_MOTOR_NONE)
    {
        return APP_CONTROL_OK;
    }

    ctl->motor_err = BSP_MOTOR_Process(ctl->motor, now_ms, &ctl->motor_fb);
    if (ctl->motor_err == BSP_MOTOR_BUSY)
    {
        *busy = 1U;
        return APP_CONTROL_OK;
    }

    op = ctl->motor_op;
    ctl->motor_op = APP_CONTROL_MOTOR_NONE;
    if (ctl->motor_err != BSP_MOTOR_OK)
    {
        if (op == APP_CONTROL_MOTOR_DISABLE)
        {
            ctl->motor_enabled = 0U;
            ctl->state = APP_CONTROL_FAULT;
            APP_Control_ResetHistory(ctl);
            return APP_CONTROL_ERR_MOTOR;
        }
        if (op == APP_CONTROL_MOTOR_ENABLE)
        {
            /* 使能报文可能已被电机执行，通信失败时仍应尝试失能。 */
            ctl->motor_enabled = 1U;
        }
        return APP_Control_Fault(ctl, APP_CONTROL_ERR_MOTOR);
    }

    if (op == APP_CONTROL_MOTOR_ENABLE)
    {
        if (ctl->motor_fb.enabled == 0U)
        {
            ctl->motor_enabled = 1U;
            ctl->motor_err = BSP_MOTOR_ERR_STATE;
            return APP_Control_Fault(ctl, APP_CONTROL_ERR_MOTOR);
        }

        ctl->motor_enabled = 1U;
        if (ctl->state == APP_CONTROL_STARTUP)
        {
            /* 电机确认使能后再移动到用户已标定的绝对零点。 */
            ctl->motor_err = BSP_MOTOR_SetAngle(ctl->motor, 0.0f);
            if (ctl->motor_err != BSP_MOTOR_OK)
            {
                return APP_Control_Fault(ctl, APP_CONTROL_ERR_MOTOR);
            }
            ctl->motor_op = APP_CONTROL_MOTOR_ANGLE;
            *busy = 1U;
            return APP_CONTROL_OK;
        }

        ctl->state = APP_CONTROL_READY;
        APP_Control_ResetHistory(ctl);
    }
    else if (op == APP_CONTROL_MOTOR_DISABLE)
    {
        ctl->motor_enabled = 0U;
        if (ctl->state != APP_CONTROL_FAULT)
        {
            ctl->state = APP_CONTROL_DISABLED;
        }
        APP_Control_ResetHistory(ctl);
    }
    else if (op == APP_CONTROL_MOTOR_ANGLE)
    {
        if (ctl->motor_fb.enabled == 0U)
        {
            ctl->motor_enabled = 0U;
            ctl->motor_err = BSP_MOTOR_ERR_STATE;
            return APP_Control_Fault(ctl, APP_CONTROL_ERR_MOTOR);
        }
        if (ctl->state == APP_CONTROL_STARTUP)
        {
            ctl->state = APP_CONTROL_READY;
            APP_Control_ResetHistory(ctl);
        }
    }

    return APP_CONTROL_OK;
}

app_control_err_t APP_Control_Init(app_control_t *ctl, bsp_motor_t *motor,
                                   app_pid_t *pid, float target_pos,
                                   float motor_dir, float angle_max,
                                   uint32_t ctrl_period_ms,
                                   uint32_t vision_timeout_ms)
{
    if ((ctl == NULL) || (motor == NULL) || (pid == NULL))
    {
        return APP_CONTROL_ERR_ARG;
    }
    if (!APP_Control_IsFinite(target_pos) ||
        ((motor_dir != 1.0f) && (motor_dir != -1.0f)) ||
        !APP_Control_IsFinite(angle_max) ||
        (angle_max <= 0.0f) ||
        (angle_max > (APP_CONTROL_TWO_PI * 0.5f)) ||
        (ctrl_period_ms == 0U) || (vision_timeout_ms == 0U))
    {
        return APP_CONTROL_ERR_RANGE;
    }

    ctl->motor = motor;
    ctl->pid = pid;
    ctl->motor_fb.state = 0U;
    ctl->motor_fb.enabled = 0U;
    ctl->motor_fb.current = 0.0f;
    ctl->motor_fb.speed = 0.0f;
    ctl->motor_fb.angle = 0.0f;
    ctl->motor_err = BSP_MOTOR_OK;
    ctl->state = APP_CONTROL_DISABLED;
    ctl->motor_op = APP_CONTROL_MOTOR_NONE;
    ctl->target_pos = target_pos;
    ctl->motor_dir = motor_dir;
    ctl->angle_max = angle_max;
    ctl->position_pos = 0.0f;
    ctl->speed_pos = 0.0f;
    ctl->speed_d_scale = 0.0f;
    ctl->ctrl_period_ms = ctrl_period_ms;
    ctl->vision_timeout_ms = vision_timeout_ms;
    ctl->last_vision_ms = 0U;
    ctl->last_ctrl_ms = 0U;
    ctl->frame_seq = 0U;
    ctl->frame_seen = 0U;
    ctl->vision_valid = 0U;
    ctl->motor_enabled = 0U;
    ctl->stop_requested = 0U;
    ctl->timing_end = 0U;
    ctl->rx_overflow = 0U;
    ctl->rx_head = 0U;
    ctl->rx_tail = 0U;
    ctl->frame_len = 0U;

    if (APP_PID_Reset(pid) != APP_PID_OK)
    {
        return APP_CONTROL_ERR_PID;
    }

    return APP_CONTROL_OK;
}

app_control_err_t APP_Control_Start(app_control_t *ctl)
{
    if ((ctl == NULL) || (ctl->motor == NULL) || (ctl->pid == NULL))
    {
        return APP_CONTROL_ERR_ARG;
    }
    if (ctl->motor_op != APP_CONTROL_MOTOR_NONE)
    {
        return APP_CONTROL_BUSY;
    }
    if (ctl->state == APP_CONTROL_FAULT)
    {
        return APP_CONTROL_ERR_STATE;
    }

    /* The boot zeroing sequence may have left the shared motor enabled. */
    if (ctl->motor_enabled != 0U)
    {
        ctl->state = APP_CONTROL_READY;
        ctl->stop_requested = 0U;
        APP_Control_ResetRx(ctl);
        APP_Control_ResetHistory(ctl);
        return APP_CONTROL_OK;
    }
    if (ctl->state != APP_CONTROL_DISABLED)
    {
        return APP_CONTROL_ERR_STATE;
    }

    ctl->motor_err = BSP_MOTOR_Enable(ctl->motor);
    if (ctl->motor_err != BSP_MOTOR_OK)
    {
        return APP_Control_Fault(ctl, APP_CONTROL_ERR_MOTOR);
    }

    ctl->state = APP_CONTROL_STARTUP;
    ctl->stop_requested = 0U;
    ctl->motor_op = APP_CONTROL_MOTOR_ENABLE;
    APP_Control_ResetRx(ctl);
    return APP_CONTROL_OK;
}

void APP_Control_AdoptMotorState(app_control_t *ctl, uint8_t enabled)
{
    if (ctl == NULL)
    {
        return;
    }

    if (enabled != 0U)
    {
        ctl->motor_enabled = 1U;
    }
    else
    {
        ctl->motor_enabled = 0U;
    }
    ctl->stop_requested = 0U;
    if (ctl->motor_op != APP_CONTROL_MOTOR_NONE)
    {
        return;
    }
    if (ctl->state == APP_CONTROL_FAULT)
    {
        return;
    }

    if (ctl->motor_enabled != 0U)
    {
        ctl->state = APP_CONTROL_READY;
    }
    else
    {
        ctl->state = APP_CONTROL_DISABLED;
    }
    APP_Control_ResetHistory(ctl);
}

void APP_Control_SetSpeedDScale(app_control_t *ctl, float scale)
{
    if (ctl == NULL)
    {
        return;
    }

    ctl->speed_d_scale = scale;
}

app_control_err_t APP_Control_Stop(app_control_t *ctl)
{
    if ((ctl == NULL) || (ctl->motor == NULL) || (ctl->pid == NULL))
    {
        return APP_CONTROL_ERR_ARG;
    }
    if (ctl->state == APP_CONTROL_FAULT)
    {
        return APP_CONTROL_ERR_STATE;
    }

    ctl->stop_requested = 1U;
    return APP_CONTROL_OK;
}

void APP_Control_RxByte(app_control_t *ctl, uint8_t byte)
{
    uint16_t next;

    if (ctl == NULL)
    {
        return;
    }

    next = APP_Control_NextIndex(ctl->rx_head);
    if (next == ctl->rx_tail)
    {
        ctl->rx_overflow = 1U;
        return;
    }

    ctl->rx_ring[ctl->rx_head] = byte;
    ctl->rx_head = next;
}

uint8_t APP_Control_TakeTimingEnd(app_control_t *ctl)
{
    uint8_t ended;

    if (ctl == NULL)
    {
        return 0U;
    }

    ended = ctl->timing_end;
    ctl->timing_end = 0U;
    return ended;
}

app_control_err_t APP_Control_ProcessRx(app_control_t *ctl,
                                        uint32_t now_ms)
{
    uint8_t byte;
    app_vision_data_t data;

    if (ctl == NULL)
    {
        return APP_CONTROL_ERR_ARG;
    }

    while (ctl->rx_tail != ctl->rx_head)
    {
        byte = ctl->rx_ring[ctl->rx_tail];
        ctl->rx_tail = APP_Control_NextIndex(ctl->rx_tail);

        if (ctl->rx_overflow != 0U)
        {
            /* 丢帧后等待下一个 AA 起始标记，避免把残缺数据提交给解析器。 */
            if (byte == (uint8_t)'\n')
            {
                ctl->frame_len = 0U;
                ctl->rx_overflow = 0U;
            }
            else if (byte == (uint8_t)'A')
            {
                ctl->frame[0] = byte;
                ctl->frame_len = 1U;
                ctl->rx_overflow = 0U;
            }
            continue;
        }

        if (byte == (uint8_t)'\r')
        {
            continue;
        }

        if (byte == (uint8_t)'\n')
        {
            if (ctl->frame_len > 0U)
            {
                ctl->frame[ctl->frame_len] = '\0';
                if (APP_Vision_Parse(ctl->frame, ctl->frame_len, &data) ==
                    APP_VISION_OK)
                {
                    if ((data.position_raw == APP_VISION_TIMING_END_VALUE) &&
                        (data.speed_raw == APP_VISION_TIMING_END_VALUE))
                    {
                        /** @brief Both fields identify the external timing-end marker. */
                        ctl->timing_end = 1U;
                    }
                    else
                    {
                        ctl->position_pos = (float)data.position_raw;
                        /** @brief Retain speed for observation; the position loop does not use it. */
                        ctl->speed_pos = (float)data.speed_raw;
                        ctl->frame_seq = data.frame_seq;
                        ctl->vision_valid = data.valid;
                        ctl->frame_seen = 1U;
                        ctl->last_vision_ms = now_ms;
                    }
                }
            }
            ctl->frame_len = 0U;
            ctl->rx_overflow = 0U;
            continue;
        }

        if (byte == (uint8_t)'A')
        {
            if ((ctl->frame_len == 1U) &&
                (ctl->frame[0] == (uint8_t)'A'))
            {
                /* 第二个 A 组成新协议的帧头。 */
                ctl->frame[ctl->frame_len] = byte;
                ++ctl->frame_len;
            }
            else
            {
                /* 新起始标记可直接丢弃未完成的旧候选帧。 */
                ctl->frame[0] = byte;
                ctl->frame_len = 1U;
            }
            continue;
        }

        if (ctl->frame_len == 0U)
        {
            continue;
        }

        if (ctl->frame_len >= (uint16_t)(APP_CONTROL_FRAME_SIZE - 1U))
        {
            ctl->rx_overflow = 1U;
            continue;
        }

        ctl->frame[ctl->frame_len] = byte;
        ++ctl->frame_len;
    }

    return APP_CONTROL_OK;
}

app_control_err_t APP_Control_SetEnabled(app_control_t *ctl, uint8_t enable)
{
    if ((ctl == NULL) || (ctl->motor == NULL) || (ctl->pid == NULL))
    {
        return APP_CONTROL_ERR_ARG;
    }
    if (ctl->motor_op != APP_CONTROL_MOTOR_NONE)
    {
        return APP_CONTROL_BUSY;
    }

    if (enable != 0U)
    {
        if (ctl->state == APP_CONTROL_FAULT)
        {
            return APP_CONTROL_ERR_STATE;
        }
        if (ctl->motor_enabled != 0U)
        {
            return APP_CONTROL_OK;
        }

        ctl->motor_err = BSP_MOTOR_Enable(ctl->motor);
        if (ctl->motor_err != BSP_MOTOR_OK)
        {
            return APP_Control_Fault(ctl, APP_CONTROL_ERR_MOTOR);
        }

        ctl->motor_op = APP_CONTROL_MOTOR_ENABLE;
        return APP_CONTROL_OK;
    }

    if (ctl->motor_enabled != 0U)
    {
        ctl->motor_err = BSP_MOTOR_Disable(ctl->motor);
        if (ctl->motor_err != BSP_MOTOR_OK)
        {
            return APP_Control_Fault(ctl, APP_CONTROL_ERR_MOTOR);
        }

        ctl->motor_op = APP_CONTROL_MOTOR_DISABLE;
        return APP_CONTROL_OK;
    }

    ctl->state = APP_CONTROL_DISABLED;
    APP_Control_ResetHistory(ctl);
    return APP_CONTROL_OK;
}

app_control_err_t APP_Control_Tick(app_control_t *ctl, uint32_t now_ms)
{
    uint32_t age;
    uint32_t elapsed;
    float dt;
    float angle;
    float derivative;
    app_pid_err_t pid_err;
    app_control_err_t control_err;
    uint8_t busy;

    if ((ctl == NULL) || (ctl->motor == NULL) || (ctl->pid == NULL))
    {
        return APP_CONTROL_ERR_ARG;
    }
    control_err = APP_Control_ProcessMotor(ctl, now_ms, &busy);
    if (control_err != APP_CONTROL_OK)
    {
        return control_err;
    }
    if (busy != 0U)
    {
        return APP_CONTROL_OK;
    }
    if (ctl->state == APP_CONTROL_FAULT)
    {
        return APP_CONTROL_ERR_STATE;
    }
    if (ctl->stop_requested != 0U)
    {
        if (ctl->motor_enabled != 0U)
        {
            ctl->motor_err = BSP_MOTOR_Disable(ctl->motor);
            if (ctl->motor_err != BSP_MOTOR_OK)
            {
                return APP_Control_Fault(ctl, APP_CONTROL_ERR_MOTOR);
            }
            ctl->motor_op = APP_CONTROL_MOTOR_DISABLE;
            return APP_CONTROL_OK;
        }

        ctl->state = APP_CONTROL_DISABLED;
        ctl->stop_requested = 0U;
        APP_Control_ResetHistory(ctl);
        return APP_CONTROL_OK;
    }
    if (ctl->motor_enabled == 0U)
    {
        return APP_CONTROL_OK;
    }

    age = now_ms - ctl->last_vision_ms;
    if ((ctl->frame_seen == 0U) ||
        (age > ctl->vision_timeout_ms) || (ctl->vision_valid == 0U))
    {
        ctl->state = APP_CONTROL_READY;
        APP_Control_ResetHistory(ctl);
        return APP_CONTROL_OK;
    }

    if (ctl->state == APP_CONTROL_READY)
    {
        ctl->state = APP_CONTROL_BALANCE;
    }
    if (ctl->last_ctrl_ms == 0U)
    {
        ctl->last_ctrl_ms = now_ms;
        return APP_CONTROL_OK;
    }

    elapsed = now_ms - ctl->last_ctrl_ms;
    if (elapsed < ctl->ctrl_period_ms)
    {
        return APP_CONTROL_OK;
    }

    ctl->last_ctrl_ms = now_ms;
    dt = (float)elapsed / 1000.0f;
    if (ctl->speed_d_scale != 0.0f)
    {
        derivative = 0.0f;
        if ((ctl->target_pos - ctl->position_pos) > 0.0f)
        {
            /** @brief TASK1 uses vision speed only on the positive-error side. */
            derivative = ctl->speed_pos * ctl->speed_d_scale;
        }
        pid_err = APP_PID_UpdateWithDerivative(ctl->pid, ctl->target_pos,
                                               ctl->position_pos, derivative,
                                               dt, &angle);
    }
    else
    {
        pid_err = APP_PID_Update(ctl->pid, ctl->target_pos,
                                 ctl->position_pos, dt, &angle);
    }
    if (pid_err != APP_PID_OK)
    {
        return APP_Control_Fault(ctl, APP_CONTROL_ERR_PID);
    }

    /* 先按机械范围限制有符号目标，再映射为电机绝对角度协议值。 */
    angle = APP_Control_Clamp(angle * ctl->motor_dir,
                              -ctl->angle_max, ctl->angle_max);
    angle = APP_Control_ToMotorAngle(angle);

    ctl->motor_err = BSP_MOTOR_SetAngle(ctl->motor, angle);
    if (ctl->motor_err != BSP_MOTOR_OK)
    {
        return APP_Control_Fault(ctl, APP_CONTROL_ERR_MOTOR);
    }
    ctl->motor_op = APP_CONTROL_MOTOR_ANGLE;

    return APP_CONTROL_OK;
}
