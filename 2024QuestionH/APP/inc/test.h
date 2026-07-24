/**
 * @file test.h
 * @brief Hardware bring-up test entry points.
 */
#ifndef TEST_H
#define TEST_H

#include "ti_msp_dl_config.h"

/**
 * @brief Run the motor duty test with OLED and encoder feedback.
 */
void Test_Motor(void);

/**
 * @brief Run the OLED display test.
 */
void Test_OLED(void);

/**
 * @brief Run the IMU attitude display test.
 */
void Test_IMU(void);

/**
 * @brief Run the UART receive echo test.
 */
void Test_UartReceive(void);

/**
 * @brief Run the track sensor display test.
 */
void Test_Track(void);

#endif
