/**
 * @file task.c
 * @brief 任务 1 至任务 6 的应用层调度
 */
#include "task.h"
#include "board.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
#include "car.h"
#include "debug.h"
#include "ui.h"

/** @brief 普通循迹的基础 duty。 */
#define TASK_TRACK_DUTY 1200
/** @brief 任务 4/5/6 缓启动完成后的循迹 duty。 */
#define TASK_SLOW_TRACK_DUTY 880
/** @brief 任务 4/5/6 缓启动的初始 duty。 */
#define TASK_TRACK_START_DUTY 300
/** @brief 任务 4/5/6 从初始 duty 上升到目标 duty 的时间。 */
#define TASK_TRACK_RAMP_MS 4000U
/** @brief 任务 2 停止计时的里程计数。 */
#define TASK2_TIME_STOP_ODO 20730U
/** @brief 任务 2 停车的里程计数。 */
#define TASK2_PARK_ODO 20730U
/** @brief 任务 4 停止计时的里程计数。 */
#define TASK4_TIME_STOP_ODO 7000U
/** @brief 任务 4 停车的里程计数。 */
#define TASK4_PARK_ODO 10000U
/** @brief 任务 5 停止计时的里程计数。 */
#define TASK5_TIME_STOP_ODO 20730U
/** @brief 任务 5 停车的里程计数。 */
#define TASK5_PARK_ODO 23000U
/** @brief 任务 6 停止计时的里程计数。 */
#define TASK6_TIME_STOP_ODO 20730U
/** @brief 任务 6 停车的里程计数。 */
#define TASK6_PARK_ODO 23000U
/** @brief OLED 运行界面刷新间隔，25 个控制周期为 500 ms。 */
#define TASK_UI_TICKS 25U
/** @brief 任务 3 运行中 OLED 的时间刷新周期。 */
#define TASK3_TIME_REFRESH_MS 100U
/** @brief VOFA+ 遥测刷新间隔，5 个控制周期为 100 ms。 */
#define TASK_DEBUG_TICKS 5U

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

/** @brief 每 100 ms 输出一帧 VOFA+ 遥测数据。 */
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

/** @brief 输出任务开始提示脉冲。 */
static void task_ring(void)
{
    DL_GPIO_setPins(GPIO_RING_PORT, GPIO_RING_PIN_27_PIN);
    delay_ms(20);
    DL_GPIO_clearPins(GPIO_RING_PORT, GPIO_RING_PIN_27_PIN);
}

/**
 * @brief 判断正向累计里程是否达到阈值
 * @param odo 当前车体中心累计编码器计数
 * @param limit 目标编码器计数
 * @return 1 表示达到阈值，0 表示尚未达到
 */
static uint8_t task_odo_reached(int32_t odo, uint32_t limit)
{
    if (odo < 0) {
        return 0U;
    }
    return (uint8_t)((uint32_t)odo >= limit);
}

/**
 * @brief 按发车后的经过时间更新缓启动循迹 duty
 * @param start_ms 发车时间戳，单位为 ms
 */
static void task_track_ramp(uint32_t start_ms)
{
    uint32_t ms;
    int duty;

    ms = Encoder_GetMs() - start_ms;
    if (ms >= TASK_TRACK_RAMP_MS) {
        Car_SetTrackDuty(TASK_SLOW_TRACK_DUTY);
        return;
    }

    duty = TASK_TRACK_START_DUTY +
           (int)(((uint32_t)(TASK_SLOW_TRACK_DUTY - TASK_TRACK_START_DUTY) *
                  ms) / TASK_TRACK_RAMP_MS);
    Car_SetTrackDuty(duty);
}

/**
 * @brief 执行带独立计时和停车里程阈值的普通循迹任务
 * @param task 任务编号
 * @param start_ms 确认按键被识别时的时间戳，单位为 ms
 * @param time_odo 停止计时的里程计数
 * @param park_odo 停车的里程计数
 * @param ramp 是否启用缓启动
 */
static void task_track_run(uint8_t task, uint32_t start_ms,
                           uint32_t time_odo, uint32_t park_odo,
                           uint8_t ramp)
{
    Car_Status st;
    uint32_t live_ms;
    uint32_t show_ms = 0U;
    uint8_t time_stopped = 0U;
    uint8_t ui_cnt = 0U;
    uint8_t park;
    int start_duty;

    Debug_SetTask1Time(0U);
    UI_TaskRunning(task, 0U, 0);
    task_ring();
    Car_Init();
    start_duty = TASK_TRACK_DUTY;
    if (ramp != 0U) {
        start_duty = TASK_TRACK_START_DUTY;
    }
    Car_SetTrack(start_duty);

    while (1) {
        Debug_CommandPoll();
        if (task_take_tick() == 0U) {
            continue;
        }

        if (ramp != 0U) {
            task_track_ramp(start_ms);
        }
        Car_Update();
        Car_GetStatus(&st);
        live_ms = Encoder_GetMs() - start_ms;
        park = task_odo_reached(st.odo, park_odo);
        if (Key_Scan() == KEY_3) {
            park = 1U;
        }

        if (park != 0U) {
            /* 任务 2 的两个阈值相同，因此先制动，再在同一控制周期锁定时间。 */
            Car_Stop();
            if (time_stopped == 0U) {
                show_ms = live_ms;
                time_stopped = 1U;
            }
            Debug_SetTask1Time(show_ms);
            UI_TaskResult(task, show_ms, st.odo);
            Debug_Output();
            while (1) {
                Debug_CommandPoll();
            }
        }

        if ((time_stopped == 0U) &&
            (task_odo_reached(st.odo, time_odo) != 0U)) {
            /* 任务 4/5/6 仅锁定计时，底盘继续循迹至各自停车阈值。 */
            show_ms = live_ms;
            time_stopped = 1U;
        }
        if (time_stopped == 0U) {
            show_ms = live_ms;
        }
        Debug_SetTask1Time(show_ms);

        ui_cnt++;
        if (ui_cnt >= TASK_UI_TICKS) {
            ui_cnt = 0U;
            UI_TaskRunning(task, show_ms, st.odo);
        }
        task_debug_tick();
    }
}

/** @brief 执行空任务 1，仅显示图传测试提示。 */
void task1_run(void)
{
    UI_Task1VideoTest();
    while (1) {
        Debug_CommandPoll();
    }
}

/**
 * @brief 执行任务 2 普通循迹，并在 20730 计数处停车后停止计时
 * @param start_ms 确认按键被识别时的时间戳，单位为 ms
 */
void task2_run(uint32_t start_ms)
{
    task_track_run(2U, start_ms, TASK2_TIME_STOP_ODO, TASK2_PARK_ODO, 0U);
}

/** @brief 执行任务 3 的 PB0 状态机门控计时。 */
void task3_run(void)
{
    UI_TimerState state = UI_TIMER_WAIT;
    uint32_t start_ms = 0U;
    uint32_t last_ms = 0U;
    uint32_t now_ms;
    uint32_t ms = 0U;
    uint8_t high;

    high = (DL_GPIO_readPins(BOARD_TASK3_GATE_PORT,
                             BOARD_TASK3_GATE_PIN) != 0U);
    if (high != 0U) {
        /* 进入任务时 PB0 已为高电平，状态机直接进入计时状态。 */
        start_ms = Encoder_GetMs();
        last_ms = start_ms;
        state = UI_TIMER_RUNNING;
    }
    UI_TaskGateTimer(3U, 0U, state);

    while (1) {
        Debug_CommandPoll();
        high = (DL_GPIO_readPins(BOARD_TASK3_GATE_PORT,
                                 BOARD_TASK3_GATE_PIN) != 0U);

        switch (state) {
            case UI_TIMER_WAIT:
                if (high != 0U) {
                    /* 高电平沿开启计时，菜单阶段的时间不计入结果。 */
                    start_ms = Encoder_GetMs();
                    last_ms = start_ms;
                    ms = 0U;
                    state = UI_TIMER_RUNNING;
                    UI_TaskGateTimer(3U, ms, state);
                }
                break;

            case UI_TIMER_RUNNING:
                now_ms = Encoder_GetMs();
                ms = now_ms - start_ms;
                if (high == 0U) {
                    /* 低电平停止并锁定时间，状态机不再重新开启。 */
                    state = UI_TIMER_STOPPED;
                    UI_TaskGateTimer(3U, ms, state);
                } else if ((now_ms - last_ms) >= TASK3_TIME_REFRESH_MS) {
                    last_ms = now_ms;
                    UI_TaskGateTimer(3U, ms, state);
                }
                break;

            case UI_TIMER_STOPPED:
            default:
                /* 停止态保持最终显示，即使 PB0 再次变高也不重新计时。 */
                break;
        }
    }
}

/**
 * @brief 执行任务 4，7000 计数停止计时，10000 计数停车
 * @param start_ms 确认按键被识别时的时间戳，单位为 ms
 */
void task4_run(uint32_t start_ms)
{
    task_track_run(4U, start_ms, TASK4_TIME_STOP_ODO, TASK4_PARK_ODO, 1U);
}

/**
 * @brief 执行任务 5，20730 计数停止计时，23000 计数停车
 * @param start_ms 确认按键被识别时的时间戳，单位为 ms
 */
void task5_run(uint32_t start_ms)
{
    task_track_run(5U, start_ms, TASK5_TIME_STOP_ODO, TASK5_PARK_ODO, 1U);
}

/**
 * @brief 执行与任务 5 阈值相同的任务 6
 * @param start_ms 确认按键被识别时的时间戳，单位为 ms
 */
void task6_run(uint32_t start_ms)
{
    task_track_run(6U, start_ms, TASK6_TIME_STOP_ODO, TASK6_PARK_ODO, 1U);
}
