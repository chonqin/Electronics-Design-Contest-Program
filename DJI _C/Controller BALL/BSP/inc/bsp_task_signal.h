/**
 * @file bsp_task_signal.h
 * @brief Task running signal output interface.
 */

#ifndef BSP_TASK_SIGNAL_H
#define BSP_TASK_SIGNAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Set the external task running signal.
 * @param running Nonzero outputs high; zero outputs low.
 */
void BSP_TaskSignal_Set(uint8_t running);

#ifdef __cplusplus
}
#endif

#endif /* BSP_TASK_SIGNAL_H */
