/**
 * @file line_track.c
 * @brief 基于八路循迹模块的基础连续循迹任务
 */

#include "line_track.h"
#include "board.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
#include "bsp_motor.h"
#include "bsp_track.h"
#include "bsp_uart.h"
#include "oled.h"
#include "pid.h"
#include "ui.h"
#include <stdint.h>
#include <stdio.h>

#define TRACK_KP             0.05f
#define TRACK_KI             0.0f
#define TRACK_KD             0.0f

#define SPEED_KP             90.0f
#define SPEED_KI             8.0f
#define SPEED_KD             0.0f

#define LINE_BASE            10
#define LINE_SEARCH_BASE     8
#define LINE_TURN_MAX        18
#define LINE_LOST_MAX        10
#define LINE_TURN_SIGN       1
#define LINE_DEBUG_DIV       20U

#define LEFT_MOTOR           MOTOR_B
#define RIGHT_MOTOR          MOTOR_A
#define LEFT_ENCODER         ENCODER_2
#define RIGHT_ENCODER        ENCODER_1

static const int weight[TRACK_NUM] = {
    -2000, -1100, -600, -100, 100, 600, 1100, 2000
};

/**
 * @brief 将整数限制到闭区间
 * @param val 输入值
 * @param min 下限
 * @param max 上限
 * @return 限幅后的整数
 */
static int limit_int(int val, int min, int max)
{
    if (val > max) {
        return max;
    }

    if (val < min) {
        return min;
    }

    return val;
}

/**
 * @brief 根据循迹位图计算黑线位置
 * @param mask bit0-bit7 对应 X1-X8，1 表示检测到黑线
 * @param pos 输出黑线加权位置
 * @return 检测到黑线的通道数量
 */
static uint8_t calc_pos(uint8_t mask, int *pos)
{
    int sum = 0;
    uint8_t cnt = 0U;

    for (uint8_t i = 0U; i < TRACK_NUM; i++) {
        if ((mask & (uint8_t)(1U << i)) != 0U) {
            sum += weight[i];
            cnt++;
        }
    }

    if (cnt > 0U) {
        *pos = sum / (int)cnt;
    }

    return cnt;
}

/**
 * @brief 刷新 OLED 并发送循迹调试数据
 * @param mask 循迹状态位图
 * @param pos 黑线位置
 * @param base 基础速度目标
 * @param turn 外环输出的转向速度差
 * @param duty_l 左轮占空比
 * @param duty_r 右轮占空比
 * @param lost 1=丢线，0=正常循迹
 */
static void show_debug(uint8_t mask, int pos, int base, int turn,
                       int duty_l, int duty_r, uint8_t lost)
{
    char buf[18];

    OLED_ShowString(0, 0, (u8 *)"Line Track      ", 16, 1);
    (void)snprintf(buf, sizeof(buf), "M:%02X P:%+4d", mask, pos);
    OLED_ShowString(0, 16, (u8 *)buf, 16, 1);
    (void)snprintf(buf, sizeof(buf), "B:%+3d T:%+3d", base, turn);
    OLED_ShowString(0, 32, (u8 *)buf, 16, 1);
    (void)snprintf(buf, sizeof(buf), "L:%+4d R:%+4d", duty_l, duty_r);
    OLED_ShowString(0, 48, (u8 *)buf, 16, 1);
    OLED_Refresh();

    lc_printf("TRK,pos:%d,turn:%d,mask:0x%02X,state:%s,base:%d,dl:%d,dr:%d\r\n",
              pos,
              turn,
              mask,
              lost ? "LOST" : "RUN",
              base,
              duty_l,
              duty_r);
}

void LineTrack_Run(void)
{
    PID trk_pid;
    PID spd_l_pid;
    PID spd_r_pid;
    uint8_t mask;
    uint8_t cnt;
    uint8_t lost = 0U;
    uint16_t dbg_cnt = 0U;
    int pos = 0;
    int last_pos = 0;
    int base = LINE_BASE;
    int turn = 0;
    int target_l = 0;
    int target_r = 0;
    int speed_l = 0;
    int speed_r = 0;
    int duty_l = 0;
    int duty_r = 0;

    UI_Init();
    encoder_init();
    Motor_Init();

    PID_Init(&trk_pid, TRACK_KP, TRACK_KI, TRACK_KD,
             (float)(-LINE_TURN_MAX), (float)LINE_TURN_MAX);
    PID_Init(&spd_l_pid, SPEED_KP, SPEED_KI, SPEED_KD,
             (float)(-MOTOR_PWM_PERIOD), (float)MOTOR_PWM_PERIOD);
    PID_Init(&spd_r_pid, SPEED_KP, SPEED_KI, SPEED_KD,
             (float)(-MOTOR_PWM_PERIOD), (float)MOTOR_PWM_PERIOD);

    while (1) {
        if (Key_Scan() == KEY_3) {
            lost = 0U;
            base = 0;
            turn = 0;
            duty_l = 0;
            duty_r = 0;
            PID_Reset(&trk_pid);
            PID_Reset(&spd_l_pid);
            PID_Reset(&spd_r_pid);
            Motor_Stop(LEFT_MOTOR);
            Motor_Stop(RIGHT_MOTOR);
            show_debug(0U, pos, base, turn, duty_l, duty_r, 1U);
            continue;
        }

        mask = Track_ReadMask();
        cnt = calc_pos(mask, &pos);

        if (cnt > 0U) {
            lost = 0U;
            last_pos = pos;
            base = LINE_BASE;
            turn = (int)PID_CalcTarget(&trk_pid, 0.0f, (float)pos);
            turn = limit_int(turn * LINE_TURN_SIGN,
                             -LINE_TURN_MAX,
                             LINE_TURN_MAX);
        } else {
            if (lost < 255U) {
                lost++;
            }

            if (lost > LINE_LOST_MAX) {
                base = 0;
                turn = 0;
                duty_l = 0;
                duty_r = 0;
                PID_Reset(&trk_pid);
                PID_Reset(&spd_l_pid);
                PID_Reset(&spd_r_pid);
                Motor_Stop(LEFT_MOTOR);
                Motor_Stop(RIGHT_MOTOR);

                if (++dbg_cnt >= LINE_DEBUG_DIV) {
                    dbg_cnt = 0U;
                    show_debug(mask, pos, base, turn, duty_l, duty_r, 1U);
                }

                continue;
            } else {
                base = LINE_SEARCH_BASE;
                turn = LINE_TURN_MAX;

                if (last_pos > 0) {
                    turn = -LINE_TURN_MAX;
                }

                turn *= LINE_TURN_SIGN;
            }
        }

        target_l = base - turn;
        target_r = base + turn;
        speed_l = encoder_get_count(LEFT_ENCODER);
        speed_r = encoder_get_count(RIGHT_ENCODER);

        /* 外环给速度目标，内环直接输出左右轮 PWM 占空比。 */
        duty_l = (int)PID_CalcTarget(&spd_l_pid,
                                     (float)target_l,
                                     (float)speed_l);
        duty_r = (int)PID_CalcTarget(&spd_r_pid,
                                     (float)target_r,
                                     (float)speed_r);

        Motor_SetDuty(LEFT_MOTOR, (int16_t)duty_l);
        Motor_SetDuty(RIGHT_MOTOR, (int16_t)duty_r);

        if (++dbg_cnt >= LINE_DEBUG_DIV) {
            dbg_cnt = 0U;
            show_debug(mask, pos, base, turn, duty_l, duty_r, cnt == 0U);
        }
    }
}
