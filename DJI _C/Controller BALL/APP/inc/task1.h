/**
 * @file task1.h
 * @brief TASK1 visual balance-control interface.
 */

#ifndef APP_TASK1_H
#define APP_TASK1_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_control.h"

/** @brief TASK1 wrapper around the shared visual balance controller. */
typedef struct
{
    app_control_t *control;
} task1_t;

/**
 * @brief Initialize TASK1 with a configured visual balance controller.
 * @param task TASK1 context.
 * @param control TASK1 visual balance controller.
 * @return APP_CONTROL_OK on success, otherwise APP_CONTROL_ERR_ARG.
 */
app_control_err_t TASK1_Init(task1_t *task, app_control_t *control);

/**
 * @brief Synchronize TASK1 with the shared motor enable state.
 * @param task TASK1 context.
 * @param enabled Nonzero when the shared motor is enabled.
 */
void TASK1_SetMotorEnabled(task1_t *task, uint8_t enabled);

/**
 * @brief Start TASK1 visual balance control.
 * @param task TASK1 context.
 * @return The result from the TASK1 balance controller.
 */
app_control_err_t TASK1_Start(task1_t *task);

/**
 * @brief Request TASK1 to stop and disable the motor.
 * @param task TASK1 context.
 * @return The result from the TASK1 balance controller.
 */
app_control_err_t TASK1_Stop(task1_t *task);

/**
 * @brief Queue one received vision byte for TASK1.
 * @param task TASK1 context.
 * @param byte Received USART1 byte.
 */
void TASK1_RxByte(task1_t *task, uint8_t byte);

/**
 * @brief Parse all queued vision bytes for TASK1.
 * @param task TASK1 context.
 * @param now_ms Current HAL tick in milliseconds.
 * @return The result from the TASK1 balance controller.
 */
app_control_err_t TASK1_ProcessRx(task1_t *task, uint32_t now_ms);

/**
 * @brief Run one TASK1 visual balance-control cycle.
 * @param task TASK1 context.
 * @param now_ms Current HAL tick in milliseconds.
 * @return The result from the TASK1 balance controller.
 */
app_control_err_t TASK1_Run(task1_t *task, uint32_t now_ms);

/**
 * @brief Report whether TASK1 has completed its asynchronous stop request.
 * @param task TASK1 context.
 * @return Nonzero when the motor is disabled and no motor transfer is active.
 */
uint8_t TASK1_IsStopped(const task1_t *task);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK1_H */
