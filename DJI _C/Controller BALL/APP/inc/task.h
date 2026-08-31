/**
 * @file task.h
 * @brief Task selection, lifecycle, and polling interface.
 */

#ifndef APP_TASK_H
#define APP_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_control.h"
#include "bsp_motor.h"
#include "task1.h"
#include "task2.h"

/** @brief Number of selectable task slots, one slot per RGB color. */
#define TASK_COUNT 3U

/** @brief Selectable task identifiers. */
typedef enum
{
    TASK_ID_1 = 0U,
    TASK_ID_2,
    TASK_ID_3,
    TASK_ID_NONE = 0xFFU
} task_id_t;

/** @brief Task manager lifecycle state. */
typedef enum
{
    TASK_STATE_SELECT = 0U,
    TASK_STATE_RUN,
    TASK_STATE_EXIT,
    TASK_STATE_FAULT,
    TASK_STATE_STARTUP
} task_state_t;

/** @brief Task manager operation result. */
typedef enum
{
    TASK_OK = 0U,
    TASK_BUSY,
    TASK_ERR_ARG,
    TASK_ERR_MOTOR,
    TASK_ERR_CONTROL,
    TASK_ERR_TX
} task_err_t;

/**
 * @brief Application serial transmit callback owned by the board layer.
 * @param ctx Board-specific callback context.
 * @param data Bytes to transmit.
 * @param len Number of bytes to transmit.
 */
typedef void (*task_tx_fn_t)(void *ctx, const uint8_t *data, uint16_t len);

/** @brief Task selector and dispatcher runtime context. */
typedef struct
{
    bsp_motor_t *motor;
    /** @brief TASK2 visual control context supplied by the board layer. */
    app_control_t *control;
    /** @brief TASK1-owned visual control context. */
    app_control_t task1_control;
    /** @brief TASK3-owned visual control context. */
    app_control_t task3_control;
    /** @brief TASK1 PID state; independently tunable from TASK2. */
    app_pid_t task1_pid;
    /** @brief TASK2 PID state. */
    app_pid_t task2_pid;
    /** @brief TASK3 PID state; never shared with another task. */
    app_pid_t task3_pid;
    /** @brief TASK1 visual wrapper using its independent controller. */
    task1_t task1;
    task2_t task2;
    task_tx_fn_t tx_fn;
    void *tx_ctx;
    uint32_t key_ms;
    uint32_t key_raw_ms;
    uint32_t led_ms;
    volatile uint32_t vision_rx_ms;
    bsp_motor_err_t motor_err;
    app_control_err_t control_err;
    task_err_t err;
    task_id_t selected;
    task_id_t active;
    task_state_t state;
    uint8_t key_raw;
    uint8_t key_stable;
    uint8_t key_down;
    uint8_t long_sent;
    uint8_t led_on;
    volatile uint8_t vision_rx_seen;
    uint8_t vision_rx_ok;
} task_mgr_t;

/**
 * @brief Initialize the task manager and start the power-up zeroing sequence.
 * @param mgr Task manager context.
 * @param motor Initialized motor context shared by startup and active tasks.
 * @param control Storage for the TASK2 visual balance controller.
 *                TASK_Init owns separate TASK1 and TASK3 controller storage.
 * @param tx_fn Application serial transmit callback used by TASK1 startup.
 * @param tx_ctx Context passed to the transmit callback.
 * @return TASK_OK when initialized, otherwise a task, motor, or control error.
 */
task_err_t TASK_Init(task_mgr_t *mgr, bsp_motor_t *motor,
                     app_control_t *control, task_tx_fn_t tx_fn,
                     void *tx_ctx);

/**
 * @brief Report whether the active task requires vision UART reception.
 * @param mgr Task manager context.
 * @return 1 when TASK1, TASK2, or TASK3 is running, otherwise 0.
 */
uint8_t TASK_NeedsVisionRx(const task_mgr_t *mgr);

/**
 * @brief Push one received vision byte and record raw UART activity.
 * @details Every accepted byte refreshes the visual-task communication LED
 *          timeout, independently of whether the complete frame is valid.
 * @param mgr Task manager context.
 * @param byte Received byte.
 */
void TASK_VisionRxByte(task_mgr_t *mgr, uint8_t byte);

/**
 * @brief Send an application frame through the configured task callback.
 * @param mgr Task manager context.
 * @param data Bytes to transmit.
 * @param len Number of bytes to transmit.
 * @return TASK_OK when accepted, TASK_ERR_TX when no callback is configured.
 */
task_err_t TASK_SerialSend(task_mgr_t *mgr, const uint8_t *data,
                           uint16_t len);

/**
 * @brief Poll communication services, key input, and the active task once.
 * @param mgr Task manager context.
 * @param now_ms Current HAL tick in milliseconds.
 * @param key_pressed Nonzero when the active-low task key is pressed.
 * @return TASK_OK in selection mode, TASK_BUSY during startup or running, or an
 *         error.
 */
task_err_t TASK_Run(task_mgr_t *mgr, uint32_t now_ms,
                    uint8_t key_pressed);

/**
 * @brief Return the task currently highlighted in selection mode.
 * @param mgr Task manager context.
 * @return Selected task identifier, or TASK_ID_NONE for an invalid context.
 */
task_id_t TASK_GetSelected(const task_mgr_t *mgr);

/**
 * @brief Return the locked active task identifier.
 * @param mgr Task manager context.
 * @return Active task identifier, or TASK_ID_NONE when no task is running.
 */
task_id_t TASK_GetActive(const task_mgr_t *mgr);

/**
 * @brief Return the task manager lifecycle state.
 * @param mgr Task manager context.
 * @return Current state, or TASK_STATE_FAULT for an invalid context.
 */
task_state_t TASK_GetState(const task_mgr_t *mgr);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK_H */
