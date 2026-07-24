/**
 * @file debug.h
 * @brief Unified car debug output interface.
 */
#ifndef DEBUG_H
#define DEBUG_H

#include "car.h"

/**
 * @brief Output current car debug data to UART and OLED.
 * @param st Current car status snapshot.
 */
void Debug_Output(const Car_Status *st);

#endif
