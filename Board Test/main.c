/**
 * @file main.c
 * @brief 主程序：通过多级菜单选择并启动对应功能
 */

#include "board.h"
#include "bsp_uart.h"
#include "pid_test.h"
#include "test.h"
#include "ui.h"

int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();

    BSP_Uart_Init();

    lc_printf("Hello %s\r\n", "World");

    /* 初始化 UI（OLED + 按键） */
    UI_Init();
    while (1) {
        /* 运行菜单，阻塞至用户确认选择 */
        Task_ID task = UI_Process();

        /* 根据选择分发任务（各任务内部含死循环，返回后重回菜单） */
        switch (task) {
            case TASK_1: LineTrack_Run();          break;
            case TASK_2: Test_Encoder();        break;
            case TASK_3: Test_Motor();          break;
            case TASK_4: Test_IMU();            break;
            case TASK_5: Test_OLED();        break;
            case TASK_6: Test_UartReceive();    break;
            case TASK_7: Test_Track();          break;
            default:                            break;
        }
    }
}

