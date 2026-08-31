/**
 * @file task2.h
 * @brief TASK2 vision-based center-position control interface.
 */

#ifndef APP_TASK2_H
#define APP_TASK2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_control.h"

/** @brief TASK2 wrapper around its private visual balance controller. */
typedef struct
{
    app_control_t *control;
} task2_t;

/**
 * @brief Initialize TASK2 with an already configured balance controller.
 * @param task TASK2 context.
 * @param control TASK2-owned visual balance controller.
 * @return APP_CONTROL_OK on success, otherwise APP_CONTROL_ERR_ARG.
 */
app_control_err_t TASK2_Init(task2_t *task, app_control_t *control);

/**
 * @brief Start TASK2 and request motor enable when required.
 * @param task TASK2 context.
 * @return The result from the TASK2 balance controller.
 */
app_control_err_t TASK2_Start(task2_t *task);

/**
 * @brief Request TASK2 to stop and disable the motor after its current transfer.
 * @param task TASK2 context.
 * @return The result from the TASK2 balance controller.
 */
app_control_err_t TASK2_Stop(task2_t *task);

/**
 * @brief Queue one received vision byte for TASK2.
 * @param task TASK2 context.
 * @param byte Received USART1 byte.
 */
void TASK2_RxByte(task2_t *task, uint8_t byte);

/**
 * @brief Parse all queued vision bytes into the latest ball position.
 * @param task TASK2 context.
 * @param now_ms Current HAL tick in milliseconds.
 * @return The result from the TASK2 balance controller.
 */
app_control_err_t TASK2_ProcessRx(task2_t *task, uint32_t now_ms);

/**
 * @brief Run one TASK2 control cycle after reception and parsing.
 * @param task TASK2 context.
 * @param now_ms Current HAL tick in milliseconds.
 * @return The result from the TASK2 balance controller.
 */
app_control_err_t TASK2_Run(task2_t *task, uint32_t now_ms);

/**
 * @brief Report whether TASK2 has completed its asynchronous stop request.
 * @param task TASK2 context.
 * @return Nonzero when the motor is disabled and no motor transfer is active.
 */
uint8_t TASK2_IsStopped(const task2_t *task);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK2_H */
