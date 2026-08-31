/**
 * @file app_vision.h
 * @brief Vision text-frame parsing interface.
 */

#ifndef APP_VISION_H
#define APP_VISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief Value required in both fields to end external task timing. */
#define APP_VISION_TIMING_END_VALUE 8888

/** @brief Vision frame parsing result. */
typedef enum
{
    APP_VISION_OK = 0,
    APP_VISION_ERR_ARG,
    APP_VISION_ERR_FORMAT,
    APP_VISION_ERR_RANGE
} app_vision_err_t;

/** @brief Parsed vision position and speed data. */
typedef struct
{
    uint32_t frame_seq;
    uint8_t valid;
    /** @brief Signed target-distance value in pixels. */
    int32_t position_raw;
    /** @brief Signed target-distance change rate in pixels per second. */
    int32_t speed_raw;
} app_vision_data_t;

/**
 * @brief Parse one vision position and speed frame.
 * @param frame Raw frame, for example "AA340,21FF\n".
 * @param len Raw frame length excluding any trailing '\0'.
 * @param data Parsed signed position and speed values.
 * @return APP_VISION_OK on success, otherwise an argument, format, or range error.
 * @note Data format: AA + signed position + ',' + signed speed + FF.
 *       Timing-end format: AA8888,8888FF. An optional line ending may follow.
 */
app_vision_err_t APP_Vision_Parse(const uint8_t *frame, uint16_t len,
                                  app_vision_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* APP_VISION_H */
