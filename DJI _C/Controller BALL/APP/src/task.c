/**
 * @file task.c
 * @brief Task selector and lifecycle dispatcher implementation.
 */

#include "task.h"

#include <stddef.h>

#include "bsp_task_signal.h"
#include "main.h"

/** @brief Key debounce interval in milliseconds. */
#define TASK_KEY_DEBOUNCE_MS 20U
/** @brief Long-press threshold in milliseconds. */
#define TASK_LONG_PRESS_MS   800U
/** @brief Selection and vision-status LED toggle period in milliseconds. */
#define TASK_LED_PERIOD_MS   500U

/** @brief TASK1 proportional gain; independently tunable from TASK2. */
#define TASK1_PID_KP              0.0016f
/** @brief TASK1 integral gain; independently tunable from TASK2. */
#define TASK1_PID_KI              0.0005f
/** @brief TASK1 derivative gain; independently tunable from TASK2. */
#define TASK1_PID_KD              0.00100f
/** @brief TASK1 integral state limit; independently tunable from TASK2. */
#define TASK1_PID_I_LIMIT         0.006f
/** @brief TASK1 vision-speed multiplier for positive-error derivative feedback. */
#define TASK1_SPEED_D_SCALE       0.0f

/** @brief TASK2 target position in centimeters. */
#define TASK2_TARGET_POS          0.0f
/** @brief TASK2 motor direction correction. */
#define TASK2_MOTOR_DIR           1.0f
/** @brief TASK2 relative angle limit in radians. */
#define TASK2_ANGLE_LIMIT_RAD     0.8f
/** @brief TASK2 control period in milliseconds. */
#define TASK2_CONTROL_PERIOD_MS   17U
/** @brief TASK2 vision timeout in milliseconds. */
#define TASK2_VISION_TIMEOUT_MS   100U
/** @brief TASK2 proportional gain. */
#define TASK2_PID_KP              0.0016f
/** @brief TASK2 integral gain. */
#define TASK2_PID_KI              0.000204f
/** @brief TASK2 derivative gain. */
#define TASK2_PID_KD              0.00066f
/** @brief TASK2 integral state limit. */
#define TASK2_PID_I_LIMIT         0.05f

/** @brief TASK3 target position in centimeters. */
#define TASK3_TARGET_POS          0.0f
/** @brief TASK3 motor direction correction. */
#define TASK3_MOTOR_DIR           1.0f
/** @brief TASK3 relative angle limit in radians. */
#define TASK3_ANGLE_LIMIT_RAD     0.8f
/** @brief TASK3 control period in milliseconds. */
#define TASK3_CONTROL_PERIOD_MS   17U
/** @brief TASK3 vision timeout in milliseconds. */
#define TASK3_VISION_TIMEOUT_MS   100U
/** @brief TASK3 proportional gain; currently matches TASK2. */
#define TASK3_PID_KP              0.0010f
/** @brief TASK3 integral gain; currently matches TASK2. */
#define TASK3_PID_KI             0.000204f
/** @brief TASK3 derivative gain; currently matches TASK2. */
#define TASK3_PID_KD               0.00066f
/** @brief TASK3 integral state limit; currently matches TASK2. */
#define TASK3_PID_I_LIMIT        0.05f

/** @brief Vision command that selects the TASK1 scene before control starts. */
static const uint8_t TASK1_START_CMD[] = "aatask01ff\n";
/** @brief Vision command that selects the TASK2 scene before control starts. */
static const uint8_t TASK2_START_CMD[] = "aatask02ff\n";
/** @brief Vision command that selects the TASK3 scene before control starts. */
static const uint8_t TASK3_START_CMD[] = "aatask03ff\n";

/** @brief Debounced key event. */
typedef enum
{
    TASK_KEY_NONE = 0U,
    TASK_KEY_SHORT,
    TASK_KEY_LONG
} task_key_event_t;

/**
 * @brief Clear the raw vision UART activity state.
 * @param mgr Task manager context.
 */
static void TASK_ResetVisionRx(task_mgr_t *mgr)
{
    mgr->vision_rx_ms = 0U;
    mgr->vision_rx_seen = 0U;
    mgr->vision_rx_ok = 0U;
}

/**
 * @brief Reset all RGB channels and apply the current task indication.
 * @param mgr Task manager context.
 */
static void TASK_UpdateLed(const task_mgr_t *mgr)
{
    GPIO_TypeDef *port;
    uint16_t pin;
    task_id_t id;

    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);

    if ((mgr->state == TASK_STATE_FAULT) || (mgr->led_on == 0U))
    {
        return;
    }

    id = mgr->selected;
    if ((mgr->state == TASK_STATE_RUN) &&
        ((mgr->active == TASK_ID_1) || (mgr->active == TASK_ID_2) ||
         (mgr->active == TASK_ID_3)))
    {
        id = TASK_ID_1;
        if (mgr->vision_rx_ok != 0U)
        {
            id = TASK_ID_2;
        }
    }
    else if (mgr->active != TASK_ID_NONE)
    {
        id = mgr->active;
    }

    port = LED_R_GPIO_Port;
    pin = LED_R_Pin;
    if (id == TASK_ID_2)
    {
        port = LED_G_GPIO_Port;
        pin = LED_G_Pin;
    }
    else if (id == TASK_ID_3)
    {
        port = LED_B_GPIO_Port;
        pin = LED_B_Pin;
    }

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

/**
 * @brief Convert the raw key level into short- and long-press events.
 * @param mgr Task manager context.
 * @param now_ms Current HAL tick in milliseconds.
 * @param pressed Nonzero when the key is pressed.
 * @return Debounced key event.
 */
static task_key_event_t TASK_UpdateKey(task_mgr_t *mgr, uint32_t now_ms,
                                       uint8_t pressed)
{
    task_key_event_t event = TASK_KEY_NONE;

    pressed = (pressed != 0U) ? 1U : 0U;
    if (pressed != mgr->key_raw)
    {
        mgr->key_raw = pressed;
        mgr->key_raw_ms = now_ms;
    }

    if (((now_ms - mgr->key_raw_ms) >= TASK_KEY_DEBOUNCE_MS) &&
        (mgr->key_stable != mgr->key_raw))
    {
        mgr->key_stable = mgr->key_raw;
        if (mgr->key_stable != 0U)
        {
            mgr->key_down = 1U;
            mgr->key_ms = now_ms;
            mgr->long_sent = 0U;
        }
        else
        {
            mgr->key_down = 0U;
            if (mgr->long_sent == 0U)
            {
                event = TASK_KEY_SHORT;
            }
        }
    }

    if ((mgr->key_down != 0U) && (mgr->long_sent == 0U) &&
        ((now_ms - mgr->key_ms) >= TASK_LONG_PRESS_MS))
    {
        mgr->long_sent = 1U;
        event = TASK_KEY_LONG;
    }

    return event;
}

/**
 * @brief Synchronize all task contexts with the shared motor enable state.
 * @param mgr Task manager context.
 * @param enabled Nonzero when the motor is enabled.
 */
static void TASK_SyncMotorState(task_mgr_t *mgr, uint8_t enabled)
{
    APP_Control_AdoptMotorState(mgr->control, enabled);
    APP_Control_AdoptMotorState(&mgr->task3_control, enabled);
    TASK1_SetMotorEnabled(&mgr->task1, enabled);
}

/**
 * @brief Handle a vision timing-end event without stopping balance control.
 * @param mgr Task manager context.
 */
static void TASK_HandleTimingEnd(task_mgr_t *mgr)
{
    uint8_t ended = 0U;

    if (mgr->active == TASK_ID_1)
    {
        ended = APP_Control_TakeTimingEnd(mgr->task1.control);
    }
    else if (mgr->active == TASK_ID_2)
    {
        ended = APP_Control_TakeTimingEnd(mgr->control);
    }
    else if (mgr->active == TASK_ID_3)
    {
        ended = APP_Control_TakeTimingEnd(&mgr->task3_control);
    }

    if (ended != 0U)
    {
        BSP_TaskSignal_Set(0U);
    }
}

/**
 * @brief Return to selection mode after the active task has finished exiting.
 * @param mgr Task manager context.
 * @param now_ms Current HAL tick in milliseconds.
 */
static void TASK_EnterSelect(task_mgr_t *mgr, uint32_t now_ms)
{
    BSP_TaskSignal_Set(0U);
    mgr->active = TASK_ID_NONE;
    mgr->state = TASK_STATE_SELECT;
    mgr->err = TASK_OK;
    mgr->led_ms = now_ms;
    mgr->led_on = 1U;
    TASK_ResetVisionRx(mgr);
}

/**
 * @brief Enter the terminal task fault state.
 * @param mgr Task manager context.
 * @param err Fault result.
 * @return The supplied fault result.
 */
static task_err_t TASK_Fault(task_mgr_t *mgr, task_err_t err)
{
    BSP_TaskSignal_Set(0U);
    mgr->state = TASK_STATE_FAULT;
    mgr->err = err;
    mgr->led_on = 0U;
    return err;
}

/**
 * @brief Poll the power-up enable and zero-angle sequence.
 * @param mgr Task manager context.
 * @param now_ms Current HAL tick in milliseconds.
 * @return TASK_OK after zeroing, TASK_BUSY while pending, or an error.
 */
static task_err_t TASK_RunStartup(task_mgr_t *mgr, uint32_t now_ms)
{
    mgr->control_err = APP_Control_Tick(mgr->control, now_ms);
    if (mgr->control_err != APP_CONTROL_OK)
    {
        return TASK_Fault(mgr, TASK_ERR_CONTROL);
    }

    if ((mgr->control->state == APP_CONTROL_READY) &&
        (mgr->control->motor_op == APP_CONTROL_MOTOR_NONE))
    {
        if (mgr->control->motor_enabled == 0U)
        {
            return TASK_Fault(mgr, TASK_ERR_CONTROL);
        }

        TASK_SyncMotorState(mgr, 1U);
        TASK_EnterSelect(mgr, now_ms);
        return TASK_OK;
    }

    return TASK_BUSY;
}

/**
 * @brief Start the selected task and lock it as the active task.
 * @param mgr Task manager context.
 * @param now_ms Current HAL tick in milliseconds.
 * @return TASK_BUSY when started, otherwise a task error.
 */
static task_err_t TASK_StartSelected(task_mgr_t *mgr, uint32_t now_ms)
{
    task_err_t tx_err;

    mgr->active = mgr->selected;
    TASK_ResetVisionRx(mgr);
    if (mgr->active == TASK_ID_1)
    {
        /** @brief Select the TASK1 vision scene before enabling its controller. */
        tx_err = TASK_SerialSend(mgr, TASK1_START_CMD,
                                 (uint16_t)(sizeof(TASK1_START_CMD) - 1U));
        if (tx_err != TASK_OK)
        {
            return TASK_Fault(mgr, tx_err);
        }

        TASK_SyncMotorState(mgr, mgr->control->motor_enabled);
        mgr->control_err = TASK1_Start(&mgr->task1);
        if (mgr->control_err != APP_CONTROL_OK)
        {
            return TASK_Fault(mgr, TASK_ERR_CONTROL);
        }
    }
    else if (mgr->active == TASK_ID_2)
    {
        /** @brief Select the TASK2 vision scene before enabling its controller. */
        tx_err = TASK_SerialSend(mgr, TASK2_START_CMD,
                                 (uint16_t)(sizeof(TASK2_START_CMD) - 1U));
        if (tx_err != TASK_OK)
        {
            return TASK_Fault(mgr, tx_err);
        }

        TASK_SyncMotorState(mgr, mgr->control->motor_enabled);
        mgr->control_err = TASK2_Start(&mgr->task2);
        if (mgr->control_err != APP_CONTROL_OK)
        {
            return TASK_Fault(mgr, TASK_ERR_CONTROL);
        }
    }
    else
    {
        /** @brief Select the TASK3 vision scene before enabling its controller. */
        tx_err = TASK_SerialSend(mgr, TASK3_START_CMD,
                                 (uint16_t)(sizeof(TASK3_START_CMD) - 1U));
        if (tx_err != TASK_OK)
        {
            return TASK_Fault(mgr, tx_err);
        }

        TASK_SyncMotorState(mgr, mgr->control->motor_enabled);
        mgr->control_err = APP_Control_Start(&mgr->task3_control);
        if (mgr->control_err != APP_CONTROL_OK)
        {
            return TASK_Fault(mgr, TASK_ERR_CONTROL);
        }
    }

    mgr->state = TASK_STATE_RUN;
    BSP_TaskSignal_Set(1U);
    mgr->led_ms = now_ms;
    mgr->led_on = 1U;
    mgr->err = TASK_BUSY;
    return TASK_BUSY;
}

/**
 * @brief Poll the dedicated TASK2 vision balance controller.
 * @param mgr Task manager context.
 * @param now_ms Current HAL tick in milliseconds.
 * @return TASK_BUSY while balancing, otherwise TASK_ERR_CONTROL.
 */
static task_err_t TASK_2(task_mgr_t *mgr, uint32_t now_ms)
{
    mgr->control_err = TASK2_Run(&mgr->task2, now_ms);
    if (mgr->control_err != APP_CONTROL_OK)
    {
        return TASK_ERR_CONTROL;
    }
    return TASK_BUSY;
}

/**
 * @brief Poll the TASK3-owned visual balance controller.
 * @details TASK3 keeps a separate PID state and visual control context so its
 *          parameters do not affect TASK2.
 * @param mgr Task manager context.
 * @param now_ms Current HAL tick in milliseconds.
 * @return TASK_BUSY while balancing, otherwise TASK_ERR_CONTROL.
 */
static task_err_t TASK_3(task_mgr_t *mgr, uint32_t now_ms)
{
    mgr->control_err = APP_Control_Tick(&mgr->task3_control, now_ms);
    if (mgr->control_err != APP_CONTROL_OK)
    {
        return TASK_ERR_CONTROL;
    }
    return TASK_BUSY;
}

/**
 * @brief Poll the task locked in the active slot.
 * @param mgr Task manager context.
 * @param now_ms Current HAL tick in milliseconds.
 * @return Current task result.
 */
static task_err_t TASK_RunActive(task_mgr_t *mgr, uint32_t now_ms)
{
    task_err_t err;

    if (mgr->active == TASK_ID_1)
    {
        mgr->control_err = TASK1_Run(&mgr->task1, now_ms);
        if (mgr->control_err != APP_CONTROL_OK)
        {
            return TASK_Fault(mgr, TASK_ERR_CONTROL);
        }
        return TASK_BUSY;
    }

    if (mgr->active == TASK_ID_2)
    {
        err = TASK_2(mgr, now_ms);
    }
    else if (mgr->active == TASK_ID_3)
    {
        err = TASK_3(mgr, now_ms);
    }
    else
    {
        return TASK_Fault(mgr, TASK_ERR_ARG);
    }

    if (err != TASK_BUSY)
    {
        return TASK_Fault(mgr, err);
    }
    return TASK_BUSY;
}

/**
 * @brief Request a safe exit from the active task.
 * @param mgr Task manager context.
 * @return TASK_BUSY while exiting, otherwise a task error.
 */
static task_err_t TASK_RequestExit(task_mgr_t *mgr)
{
    if (mgr->active == TASK_ID_1)
    {
        mgr->control_err = TASK1_Stop(&mgr->task1);
        if (mgr->control_err != APP_CONTROL_OK)
        {
            return TASK_Fault(mgr, TASK_ERR_CONTROL);
        }
    }
    else if (mgr->active == TASK_ID_2)
    {
        mgr->control_err = TASK2_Stop(&mgr->task2);
        if (mgr->control_err != APP_CONTROL_OK)
        {
            return TASK_Fault(mgr, TASK_ERR_CONTROL);
        }
    }
    else
    {
        mgr->control_err = APP_Control_Stop(&mgr->task3_control);
        if (mgr->control_err != APP_CONTROL_OK)
        {
            return TASK_Fault(mgr, TASK_ERR_CONTROL);
        }
    }

    mgr->state = TASK_STATE_EXIT;
    TASK_ResetVisionRx(mgr);
    mgr->err = TASK_BUSY;
    return TASK_BUSY;
}

/**
 * @brief Poll the active task until its asynchronous exit is complete.
 * @param mgr Task manager context.
 * @param now_ms Current HAL tick in milliseconds.
 * @return TASK_OK after exit, TASK_BUSY while exiting, or an error.
 */
static task_err_t TASK_RunExit(task_mgr_t *mgr, uint32_t now_ms)
{
    if (mgr->active == TASK_ID_1)
    {
        mgr->control_err = TASK1_Run(&mgr->task1, now_ms);
        if (mgr->control_err != APP_CONTROL_OK)
        {
            return TASK_Fault(mgr, TASK_ERR_CONTROL);
        }
        if (TASK1_IsStopped(&mgr->task1) != 0U)
        {
            TASK_SyncMotorState(mgr, 0U);
            TASK_EnterSelect(mgr, now_ms);
            return TASK_OK;
        }
        return TASK_BUSY;
    }

    if (mgr->active == TASK_ID_2)
    {
        mgr->control_err = TASK2_Run(&mgr->task2, now_ms);
    }
    else if (mgr->active == TASK_ID_3)
    {
        mgr->control_err = APP_Control_Tick(&mgr->task3_control, now_ms);
    }
    else
    {
        return TASK_Fault(mgr, TASK_ERR_ARG);
    }
    if (mgr->control_err != APP_CONTROL_OK)
    {
        return TASK_Fault(mgr, TASK_ERR_CONTROL);
    }
    if (((mgr->active == TASK_ID_2) && TASK2_IsStopped(&mgr->task2)) ||
        ((mgr->active == TASK_ID_3) &&
         (mgr->task3_control.state == APP_CONTROL_DISABLED) &&
         (mgr->task3_control.motor_op == APP_CONTROL_MOTOR_NONE)))
    {
        TASK_SyncMotorState(mgr, 0U);
        TASK_EnterSelect(mgr, now_ms);
        return TASK_OK;
    }

    return TASK_BUSY;
}

/**
 * @brief Drain an already-started visual motor transaction after a fault.
 * @param mgr Task manager context.
 * @param now_ms Current HAL tick in milliseconds.
 */
static void TASK_DrainFault(task_mgr_t *mgr, uint32_t now_ms)
{
    if ((mgr->active == TASK_ID_1) &&
        (mgr->task1.control->motor_op != APP_CONTROL_MOTOR_NONE))
    {
        (void)TASK1_Run(&mgr->task1, now_ms);
    }
    else if ((mgr->active == TASK_ID_2) &&
             (mgr->control->motor_op != APP_CONTROL_MOTOR_NONE))
    {
        (void)TASK2_Run(&mgr->task2, now_ms);
    }
    else if ((mgr->active == TASK_ID_3) &&
             (mgr->task3_control.motor_op != APP_CONTROL_MOTOR_NONE))
    {
        (void)APP_Control_Tick(&mgr->task3_control, now_ms);
    }
}

task_err_t TASK_Init(task_mgr_t *mgr, bsp_motor_t *motor,
                     app_control_t *control, task_tx_fn_t tx_fn,
                     void *tx_ctx)
{
    app_control_err_t control_err;
    app_pid_err_t pid_err;
    uint32_t now_ms;

    if ((mgr == NULL) || (motor == NULL) || (control == NULL))
    {
        return TASK_ERR_ARG;
    }

    mgr->motor = motor;
    mgr->control = control;
    mgr->tx_fn = tx_fn;
    mgr->tx_ctx = tx_ctx;

    pid_err = APP_PID_Init(&mgr->task1_pid,
                           TASK1_PID_KP, TASK1_PID_KI, TASK1_PID_KD,
                           -TASK2_ANGLE_LIMIT_RAD, TASK2_ANGLE_LIMIT_RAD,
                           -TASK1_PID_I_LIMIT, TASK1_PID_I_LIMIT);
    if (pid_err != APP_PID_OK)
    {
        return TASK_ERR_CONTROL;
    }

    control_err = APP_Control_Init(&mgr->task1_control, motor,
                                   &mgr->task1_pid, TASK2_TARGET_POS,
                                   TASK2_MOTOR_DIR, TASK2_ANGLE_LIMIT_RAD,
                                   TASK2_CONTROL_PERIOD_MS,
                                   TASK2_VISION_TIMEOUT_MS);
    if (control_err != APP_CONTROL_OK)
    {
        return TASK_ERR_CONTROL;
    }
    /** @brief TASK1 alone uses gated vision-speed derivative feedback. */
    APP_Control_SetSpeedDScale(&mgr->task1_control,
                               TASK1_SPEED_D_SCALE);

    pid_err = APP_PID_Init(&mgr->task2_pid,
                           TASK2_PID_KP, TASK2_PID_KI, TASK2_PID_KD,
                           -TASK2_ANGLE_LIMIT_RAD, TASK2_ANGLE_LIMIT_RAD,
                           -TASK2_PID_I_LIMIT, TASK2_PID_I_LIMIT);
    if (pid_err != APP_PID_OK)
    {
        return TASK_ERR_CONTROL;
    }

    control_err = APP_Control_Init(control, motor, &mgr->task2_pid,
                                   TASK2_TARGET_POS, TASK2_MOTOR_DIR,
                                   TASK2_ANGLE_LIMIT_RAD,
                                   TASK2_CONTROL_PERIOD_MS,
                                   TASK2_VISION_TIMEOUT_MS);
    if (control_err != APP_CONTROL_OK)
    {
        return TASK_ERR_CONTROL;
    }

    pid_err = APP_PID_Init(&mgr->task3_pid,
                           TASK3_PID_KP, TASK3_PID_KI, TASK3_PID_KD,
                           -TASK3_ANGLE_LIMIT_RAD, TASK3_ANGLE_LIMIT_RAD,
                           -TASK3_PID_I_LIMIT, TASK3_PID_I_LIMIT);
    if (pid_err != APP_PID_OK)
    {
        return TASK_ERR_CONTROL;
    }

    control_err = APP_Control_Init(&mgr->task3_control, motor,
                                   &mgr->task3_pid, TASK3_TARGET_POS,
                                   TASK3_MOTOR_DIR, TASK3_ANGLE_LIMIT_RAD,
                                   TASK3_CONTROL_PERIOD_MS,
                                   TASK3_VISION_TIMEOUT_MS);
    if (control_err != APP_CONTROL_OK)
    {
        return TASK_ERR_CONTROL;
    }

    mgr->control_err = TASK1_Init(&mgr->task1, &mgr->task1_control);
    if (mgr->control_err != APP_CONTROL_OK)
    {
        return TASK_ERR_CONTROL;
    }

    mgr->control_err = TASK2_Init(&mgr->task2, control);
    if (mgr->control_err != APP_CONTROL_OK)
    {
        return TASK_ERR_CONTROL;
    }

    now_ms = HAL_GetTick();
    mgr->key_ms = now_ms;
    mgr->key_raw_ms = now_ms;
    mgr->led_ms = now_ms;
    mgr->control_err = APP_CONTROL_OK;
    mgr->err = TASK_OK;
    mgr->selected = TASK_ID_1;
    mgr->active = TASK_ID_NONE;
    mgr->state = TASK_STATE_STARTUP;
    mgr->key_raw = 0U;
    mgr->key_stable = 0U;
    mgr->key_down = 0U;
    mgr->long_sent = 0U;
    mgr->led_on = 1U;
    TASK_ResetVisionRx(mgr);
    BSP_TaskSignal_Set(0U);
    TASK_UpdateLed(mgr);

    mgr->control_err = APP_Control_Start(mgr->control);
    if (mgr->control_err != APP_CONTROL_OK)
    {
        return TASK_Fault(mgr, TASK_ERR_CONTROL);
    }

    return TASK_OK;
}

uint8_t TASK_NeedsVisionRx(const task_mgr_t *mgr)
{
    if (mgr == NULL)
    {
        return 0U;
    }
    if (mgr->state != TASK_STATE_RUN)
    {
        return 0U;
    }

    return (uint8_t)((mgr->active == TASK_ID_1) ||
                     (mgr->active == TASK_ID_2) ||
                     (mgr->active == TASK_ID_3));
}

void TASK_VisionRxByte(task_mgr_t *mgr, uint8_t byte)
{
    if ((mgr == NULL) || (mgr->control == NULL) ||
        (TASK_NeedsVisionRx(mgr) == 0U))
    {
        return;
    }

    mgr->vision_rx_ms = HAL_GetTick();
    mgr->vision_rx_seen = 1U;
    if (mgr->active == TASK_ID_1)
    {
        TASK1_RxByte(&mgr->task1, byte);
    }
    else if (mgr->active == TASK_ID_2)
    {
        TASK2_RxByte(&mgr->task2, byte);
    }
    else if (mgr->active == TASK_ID_3)
    {
        APP_Control_RxByte(&mgr->task3_control, byte);
    }
}

task_err_t TASK_SerialSend(task_mgr_t *mgr, const uint8_t *data,
                           uint16_t len)
{
    if ((mgr == NULL) || (data == NULL) || (len == 0U))
    {
        return TASK_ERR_ARG;
    }
    if (mgr->tx_fn == NULL)
    {
        return TASK_ERR_TX;
    }

    mgr->tx_fn(mgr->tx_ctx, data, len);
    return TASK_OK;
}

task_err_t TASK_Run(task_mgr_t *mgr, uint32_t now_ms,
                    uint8_t key_pressed)
{
    const app_control_t *vision_ctl;
    task_key_event_t event;
    task_err_t result = TASK_OK;
    uint8_t rx_ok;

    if ((mgr == NULL) || (mgr->motor == NULL) || (mgr->control == NULL))
    {
        return TASK_ERR_ARG;
    }

    mgr->control_err = APP_CONTROL_OK;
    if (TASK_NeedsVisionRx(mgr) != 0U)
    {
        if (mgr->active == TASK_ID_1)
        {
            mgr->control_err = TASK1_ProcessRx(&mgr->task1, now_ms);
        }
        else if (mgr->active == TASK_ID_2)
        {
            mgr->control_err = TASK2_ProcessRx(&mgr->task2, now_ms);
        }
        else if (mgr->active == TASK_ID_3)
        {
            mgr->control_err = APP_Control_ProcessRx(&mgr->task3_control,
                                                     now_ms);
        }
        if (mgr->control_err == APP_CONTROL_OK)
        {
            TASK_HandleTimingEnd(mgr);
        }
    }
    if (mgr->control_err != APP_CONTROL_OK)
    {
        result = TASK_Fault(mgr, TASK_ERR_CONTROL);
    }
    else
    {
        event = TASK_UpdateKey(mgr, now_ms, key_pressed);
        if (mgr->state == TASK_STATE_STARTUP)
        {
            result = TASK_RunStartup(mgr, now_ms);
        }
        else if (mgr->state == TASK_STATE_SELECT)
        {
            if (event == TASK_KEY_SHORT)
            {
                mgr->selected = (task_id_t)(((uint8_t)mgr->selected + 1U) %
                                             TASK_COUNT);
                mgr->led_ms = now_ms;
                mgr->led_on = 1U;
            }
            else if (event == TASK_KEY_LONG)
            {
                result = TASK_StartSelected(mgr, now_ms);
            }
        }
        else if (mgr->state == TASK_STATE_RUN)
        {
            if (event == TASK_KEY_LONG)
            {
                result = TASK_RequestExit(mgr);
            }
            else
            {
                result = TASK_RunActive(mgr, now_ms);
            }
        }
        else if (mgr->state == TASK_STATE_EXIT)
        {
            result = TASK_RunExit(mgr, now_ms);
        }
        else
        {
            TASK_DrainFault(mgr, now_ms);
            result = mgr->err;
        }
    }

    if ((mgr->state == TASK_STATE_RUN) &&
        ((mgr->active == TASK_ID_1) || (mgr->active == TASK_ID_2) ||
         (mgr->active == TASK_ID_3)))
    {
        if (mgr->active == TASK_ID_1)
        {
            vision_ctl = mgr->task1.control;
        }
        else if (mgr->active == TASK_ID_3)
        {
            vision_ctl = &mgr->task3_control;
        }
        else
        {
            vision_ctl = mgr->control;
        }
        rx_ok = 0U;
        if ((mgr->vision_rx_seen != 0U) &&
            ((now_ms - mgr->vision_rx_ms) <=
             vision_ctl->vision_timeout_ms))
        {
            rx_ok = 1U;
        }

        if (rx_ok != mgr->vision_rx_ok)
        {
            mgr->vision_rx_ok = rx_ok;
            mgr->led_ms = now_ms;
            mgr->led_on = 1U;
        }
        else if ((now_ms - mgr->led_ms) >= TASK_LED_PERIOD_MS)
        {
            mgr->led_ms = now_ms;
            mgr->led_on = (uint8_t)(mgr->led_on == 0U);
        }
    }
    else if (mgr->state == TASK_STATE_SELECT)
    {
        if ((now_ms - mgr->led_ms) >= TASK_LED_PERIOD_MS)
        {
            mgr->led_ms = now_ms;
            mgr->led_on = (uint8_t)(mgr->led_on == 0U);
        }
    }
    else if ((mgr->state == TASK_STATE_RUN) ||
             (mgr->state == TASK_STATE_EXIT))
    {
        mgr->led_on = 1U;
    }
    else if (mgr->state == TASK_STATE_STARTUP)
    {
        mgr->led_on = 1U;
    }
    else
    {
        mgr->led_on = 0U;
    }
    TASK_UpdateLed(mgr);

    mgr->err = result;
    return result;
}

task_id_t TASK_GetSelected(const task_mgr_t *mgr)
{
    if (mgr == NULL)
    {
        return TASK_ID_NONE;
    }
    return mgr->selected;
}

task_id_t TASK_GetActive(const task_mgr_t *mgr)
{
    if (mgr == NULL)
    {
        return TASK_ID_NONE;
    }
    return mgr->active;
}

task_state_t TASK_GetState(const task_mgr_t *mgr)
{
    if (mgr == NULL)
    {
        return TASK_STATE_FAULT;
    }
    return mgr->state;
}
