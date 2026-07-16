/**
 * @file main.c
 * @brief 主程序：通过多级菜单选择并启动对应功能
 */

#include "board.h"
#include "bsp_test.h"
#include "bsp_encoder.h"
#include "imu.h"
#include "ui.h"

int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    lc_printf("Hello %s\r\n", "World");

    /* 初始化 UI（OLED + 按键） */
    UI_Init();

    while (1) {
        /* 运行菜单，阻塞至用户确认选择 */
        Task_ID task = UI_Process();

        /* 根据选择分发任务（各任务内部含死循环，返回后重回菜单） */
        switch (task) {
            case TASK_1: BSP_Test_IMU();        break;
            case TASK_2: BSP_Test_Encoder();    break;
            case TASK_3: BSP_Test_Motor();      break;
            case TASK_4: BSP_Test_OLED();       break;
            default:                            break;
        }
    }
}

