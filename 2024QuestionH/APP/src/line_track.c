/**
 * @file line_track.c
 * @brief Basic continuous line tracking task based on car control layer.
 */

#include "line_track.h"
#include "bsp_key.h"
#include "car.h"
#include "debug.h"
#include "ui.h"

#define LINE_BASE        10
#define LINE_DEBUG_DIV   20U

void LineTrack_Run(void)
{
    Car_Status st;
    uint16_t dbg_cnt = 0U;

    UI_Init();
    Car_Init();
    Car_SetTrack(LINE_BASE);

    while (1) {
        if (Key_Scan() == KEY_3) {
            Car_Stop();
        }

        Car_Update();

        /* Keep output frequency controlled by the caller, not by debug module. */
        if (++dbg_cnt >= LINE_DEBUG_DIV) {
            dbg_cnt = 0U;
            Car_GetStatus(&st);
            Debug_Output(&st);
        }
    }
}
