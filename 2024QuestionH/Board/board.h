
#ifndef __BOARD_H__
#define __BOARD_H__

#include "ti_msp_dl_config.h"

/** @brief VOFA+ JustFloat 模式下是否允许 UART0 文本日志 */
#ifndef BOARD_TEXT_LOG_ENABLE
#define BOARD_TEXT_LOG_ENABLE 0U
#endif

#ifndef u8
#define u8 uint8_t
#endif

#ifndef u16
#define u16 uint16_t
#endif

#ifndef u32
#define u32 uint32_t
#endif

#ifndef u64
#define u64 uint64_t
#endif

/** @brief Task3 计时门控输入端口，对应 SysConfig 中的 PB0。 */
#define BOARD_TASK3_GATE_PORT GPIOB
/** @brief Task3 计时门控输入引脚，高电平开始、低电平停止。 */
#define BOARD_TASK3_GATE_PIN DL_GPIO_PIN_0

int LOG_Debug_Out(const char* __file, const char* __func, int __line, const char* format, ...);

#define LOG_D(fmt, ...) \
    do { \
        LOG_Debug_Out(__FILE__, (const char*)__func__, __LINE__, fmt, ##__VA_ARGS__); \
    } while (0)



/* 延时函数 */
void delay_us(int __us);
void delay_ms(int __ms);

void delay_1us(int __us);
void delay_1ms(int __ms);

#endif
