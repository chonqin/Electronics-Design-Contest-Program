/**
 * @file app_vision.c
 * @brief Vision pixel position and speed frame parsing.
 */

#include "app_vision.h"

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Check whether one byte is an ASCII decimal digit.
 * @param byte Byte to check.
 * @return Nonzero when byte is between '0' and '9'.
 */
static uint8_t APP_Vision_IsDigit(uint8_t byte)
{
    return (uint8_t)((byte >= (uint8_t)'0') && (byte <= (uint8_t)'9'));
}

/**
 * @brief Parse one signed decimal field without consuming its delimiter.
 * @param frame Raw frame buffer.
 * @param end Exclusive frame payload end.
 * @param pos Input and output cursor.
 * @param value Parsed signed 32-bit value.
 * @return APP_VISION_OK on success, otherwise a format or range error.
 */
static app_vision_err_t APP_Vision_ParseInt(const uint8_t *frame,
                                            uint16_t end, uint16_t *pos,
                                            int32_t *value)
{
    uint16_t start;
    uint32_t raw = 0U;
    uint32_t limit;
    uint8_t negative = 0U;

    if (*pos >= end)
    {
        return APP_VISION_ERR_FORMAT;
    }
    if (frame[*pos] == (uint8_t)'-')
    {
        negative = 1U;
        ++(*pos);
    }

    start = *pos;
    limit = (negative != 0U) ? 2147483648U : 2147483647U;
    while ((*pos < end) && APP_Vision_IsDigit(frame[*pos]))
    {
        if (raw > ((limit - (uint32_t)(frame[*pos] - (uint8_t)'0')) /
                   10U))
        {
            return APP_VISION_ERR_RANGE;
        }
        raw = (raw * 10U) +
              (uint32_t)(frame[*pos] - (uint8_t)'0');
        ++(*pos);
    }

    if (*pos == start)
    {
        return APP_VISION_ERR_FORMAT;
    }
    if (negative == 0U)
    {
        *value = (int32_t)raw;
    }
    else if (raw == 2147483648U)
    {
        *value = INT32_MIN;
    }
    else
    {
        *value = -(int32_t)raw;
    }

    return APP_VISION_OK;
}

app_vision_err_t APP_Vision_Parse(const uint8_t *frame, uint16_t len,
                                  app_vision_data_t *data)
{
    uint16_t end;
    uint16_t pos;
    app_vision_err_t err;

    if ((frame == NULL) || (data == NULL) || (len < 3U))
    {
        return APP_VISION_ERR_ARG;
    }

    end = len;
    while ((end > 0U) &&
           ((frame[end - 1U] == (uint8_t)'\r') ||
            (frame[end - 1U] == (uint8_t)'\n')))
    {
        --end;
    }

    if ((end < 7U) || (frame[0] != (uint8_t)'A') ||
        (frame[1] != (uint8_t)'A'))
    {
        return APP_VISION_ERR_FORMAT;
    }

    pos = 2U;
    err = APP_Vision_ParseInt(frame, end, &pos, &data->position_raw);
    if (err != APP_VISION_OK)
    {
        return err;
    }

    if ((pos >= end) || (frame[pos] != (uint8_t)','))
    {
        return APP_VISION_ERR_FORMAT;
    }
    ++pos;

    err = APP_Vision_ParseInt(frame, end, &pos, &data->speed_raw);
    if (err != APP_VISION_OK)
    {
        return err;
    }
    if ((pos + 1U >= end) || (frame[pos] != (uint8_t)'F') ||
        (frame[pos + 1U] != (uint8_t)'F') ||
        ((uint16_t)(pos + 2U) != end))
    {
        return APP_VISION_ERR_FORMAT;
    }

    /** @brief A syntactically valid frame contains valid vision data. */
    data->frame_seq = 0U;
    data->valid = 1U;

    return APP_VISION_OK;
}
