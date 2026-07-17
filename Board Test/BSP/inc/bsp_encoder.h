/**
 * @file bsp_encoder.h
 * @brief Dual quadrature encoder BSP driver interface.
 */
#ifndef _BSP_ENCODER_H_
#define _BSP_ENCODER_H_

#include "ti_msp_dl_config.h"

typedef enum {
    FORWARD,
    REVERSAL
} ENCODER_DIR;

typedef enum {
    ENCODER_1 = 0,
    ENCODER_2,
    ENCODER_NUM
} ENCODER_ID;

typedef struct {
    volatile long long temp_count;
    int count;
    ENCODER_DIR dir;
} ENCODER_RES;

/**
 * @brief Initialize encoder GPIO and periodic latch timer interrupts.
 */
void encoder_init(void);

/**
 * @brief Get encoder 1 latest latched count.
 * @return Encoder 1 count.
 */
int get_encoder_count(void);

/**
 * @brief Get encoder 1 latest direction.
 * @return Encoder 1 direction.
 */
ENCODER_DIR get_encoder_dir(void);

/**
 * @brief Get selected encoder latest latched count.
 * @param id Encoder selector.
 * @return Selected encoder count.
 */
int encoder_get_count(ENCODER_ID id);

/**
 * @brief Get selected encoder latest direction.
 * @param id Encoder selector.
 * @return Selected encoder direction.
 */
ENCODER_DIR encoder_get_dir(ENCODER_ID id);

/**
 * @brief Latch all encoder pulse accumulations into public state.
 */
void encoder_update(void);

#endif
