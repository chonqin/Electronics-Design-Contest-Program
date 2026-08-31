/**
 * @file main.c
 * @brief 主程序入口
 */

#include "board.h"
#include "bsp_encoder.h"
#include "bsp_uart.h"
#include "task.h"
#include "test.h"
#include "ui.h"
#include <stdio.h>

int main(void)
{
    SYSCFG_DL_init();
    BSP_Uart_Init();
    Encoder_TimeInit();

    printf("Hello %s\r\n", "World");

    UI_Init();
    while (1) { 
        uint32_t confirm_ms;
        Task_ID task = UI_Process(&confirm_ms);

        switch (task) {
            case TASK_1:    task1_run(); break;
            case TASK_2:    task2_run(confirm_ms); break;
            case TASK_3:    task3_run(); break;
            case TASK_4:    task4_run(confirm_ms); break;
            case TASK_5:    task5_run(confirm_ms); break;
            case TASK_6:    task6_run(confirm_ms); break;
            case TASK_7:    Test_UartReceive();break;
            case TASK_8:    Test_Track();    break;
            default:                        break;
        }
    }
}
