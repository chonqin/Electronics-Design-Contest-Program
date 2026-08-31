/**
 * @file bsp_task_signal.c
 * @brief Task running signal output implementation.
 */

#include "bsp_task_signal.h"

#include "main.h"

void BSP_TaskSignal_Set(uint8_t running)
{
    GPIO_PinState state = GPIO_PIN_RESET;

    if (running != 0U)
    {
        state = GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(SEND_TO_M0_GPIO_Port, SEND_TO_M0_Pin, state);
}
