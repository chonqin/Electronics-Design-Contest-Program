/**
 * @file task.c
 * @brief 任务 1、任务 2 与任务 3 的 20 ms 底盘调度
 */
#include "task.h"
#include "board.h"
#include "bsp_encoder.h"
#include "bsp_track.h"
#include "car.h"
#include "debug.h"

#define TASK_TRACK_DUTY 1100
#define TASK_DEBUG_TICKS 5U

/** @brief Task3基于20 ms控制周期的拐角参数 */
#define TASK_CORNER_LOST_TICKS 3U
#define TASK_CORNER_GO_TICKS 9U
#define TASK_FIND_LINE_TICKS 3U
#define TASK_FIND_MAX_TICKS 50U
#define TASK_CORNER_DEG (90.0f)
#define TASK_TRACK_CENTER_MASK ((1U << TRACK_X4) | (1U << TRACK_X5))

/** @brief Task3正方形循迹状态 */
typedef enum {
    TASK_RUN_TRACK = 0,
    TASK_RUN_ADVANCE,
    TASK_RUN_TURN,
    TASK_RUN_FIND
} Task_RunMode;

/**
 * @brief 原子获取一个 20 ms 底盘控制 tick
 * @return 1 表示已获取控制 tick，0 表示尚未到达
 */
static uint8_t task_take_tick(void)
{
    uint32_t primask;
    uint8_t tick;

    primask = __get_PRIMASK();
    __disable_irq();
    tick = encoder_tick;
    encoder_tick = 0U;
    if (primask == 0U) {
        __enable_irq();
    }

    return tick;
}

/** @brief 每 100 ms 输出一帧 VOFA+ 遥测数据 */
static void task_debug_tick(void)
{
    static uint8_t cnt;

    cnt++;
    if (cnt < TASK_DEBUG_TICKS) {
        return;
    }
    cnt = 0U;

    Debug_Output();
}

/** @brief 输出任务开始提示脉冲 */
static void task_ring(void)
{
    DL_GPIO_setPins(GPIO_RING_PORT, GPIO_RING_PIN_27_PIN);
    delay_ms(20);
    DL_GPIO_clearPins(GPIO_RING_PORT, GPIO_RING_PIN_27_PIN);
}

/**
 * @brief 将航向角归一化到 [-180, 180]
 * @param deg 输入角度，单位为度
 * @return 归一化后的角度
 */
static float task_wrap_deg(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

/**
 * @brief 判断中间探头是否检测到黑线
 * @param mask 循迹状态位图
 * @return 1 表示 X4 或 X5 检测到黑线，0 表示均未检测到
 */
static uint8_t task_center_on_line(uint8_t mask)
{
    if ((mask & TASK_TRACK_CENTER_MASK) != TASK_TRACK_CENTER_MASK) {
        return 1U;
    }
    return 0U;
}

/**
 * @brief 执行任务 1 循迹
 */
void task1_run(void)
{
    Car_Init();
    Car_SetTrack(TASK_TRACK_DUTY);
    task_ring();

    while (1) {
        Debug_CommandPoll();
        if (task_take_tick() != 0U) {
            Car_Update();
            task_debug_tick();
        }
    }
}

/**
 * @brief 执行任务 2 定角转向
 */
void task2_run(void)
{
    Car_Init();
    task_ring();
    delay_ms(3000);
    Car_SetTurnAngle(90.0f);

    while (1) {
        Debug_CommandPoll();
        if (task_take_tick() != 0U) {
            Car_Update();
            task_debug_tick();
        }
    }
}

/**
 * @brief 执行任务 3 的循迹与丢线转向流程
 */
void task3_run(void)
{
    Car_Status st;
    Task_RunMode mode = TASK_RUN_TRACK;
    uint8_t corner = 0U;
    uint8_t lost = 0U;
    uint8_t go = 0U;
    uint8_t find = 0U;
    uint8_t found = 0U;
    uint8_t seen = 0U;
    float yaw0;
    float yaw_tar;

    Car_Init();
    task_ring();
    delay_ms(3000);

    /* 记录第一条边的航向，后续四个目标角按正方形绝对方向循环。 */
    Car_Update();
    Car_GetStatus(&st);
    yaw0 = st.yaw;
    yaw_tar = yaw0;
    /* Task3自行判定直角，丢线确认期间制动，禁止底层提前旋转寻线。 */
    Car_SetTrackLostSearch(0U);
    Car_SetTrack(TASK_TRACK_DUTY);

    while (1) {
        Debug_CommandPoll();
        if (task_take_tick() != 0U) {
            Car_Update();
            Car_GetStatus(&st);

            if (st.mode == CAR_MODE_STOP) {
                lost = 0U;
                go = 0U;
                find = 0U;
                found = 0U;
                seen = 0U;
                task_debug_tick();
                continue;
            }

            if (mode == TASK_RUN_TRACK) {
                if (st.track_mask == TRACK_MASK_NO_LINE) {
                    if ((seen != 0U) && (lost < TASK_CORNER_LOST_TICKS)) {
                        lost++;
                    }
                } else {
                    lost = 0U;
                    seen = 1U;
                }

                if ((seen != 0U) && (lost >= TASK_CORNER_LOST_TICKS)) {
                    /* 保持当前边航向前移，使车轮旋转中心到达直角附近。 */
                    Car_SetHeading(TASK_TRACK_DUTY, yaw_tar);
                    mode = TASK_RUN_ADVANCE;
                    go = 0U;
                    lost = 0U;
                }
            } else if (mode == TASK_RUN_ADVANCE) {
                go++;
                if (go >= TASK_CORNER_GO_TICKS) {
                    corner++;
                    if (corner >= 4U) {
                        corner = 0U;
                    }
                    yaw_tar = task_wrap_deg(yaw0 + TASK_CORNER_DEG * corner);
                    Car_SetTurnAngle(task_wrap_deg(yaw_tar - st.yaw));
                    mode = TASK_RUN_TURN;
                    go = 0U;
                }
            } else if (mode == TASK_RUN_TURN) {
                if (st.done != 0U) {
                    /* 第四次转向完成后回到初始航向，提示已跑完一圈。 */
                    if (corner == 0U) {
                        task_ring();
                    }
                    Car_SetHeading(TASK_TRACK_DUTY, yaw_tar);
                    mode = TASK_RUN_FIND;
                    find = 0U;
                    found = 0U;
                }
            } else {
                /* 仅在中间探头连续稳定压线后恢复循迹，避免误抓旧线或侧边线。 */
                if (task_center_on_line(st.track_mask) != 0U) {
                    if (found < TASK_FIND_LINE_TICKS) {
                        found++;
                    }
                } else {
                    found = 0U;
                }

                if (found >= TASK_FIND_LINE_TICKS) {
                    Car_SetTrack(TASK_TRACK_DUTY);
                    mode = TASK_RUN_TRACK;
                    lost = 0U;
                    found = 0U;
                    seen = 1U;
                } else {
                    find++;
                    if (find >= TASK_FIND_MAX_TICKS) {
                        Car_Stop();
                    }
                }
            }

            task_debug_tick();
        }
    }
}
