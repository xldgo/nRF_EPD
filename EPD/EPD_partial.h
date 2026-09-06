#ifndef __EPD_PARTIAL_H
#define __EPD_PARTIAL_H

#include <stdint.h>

/* BW partial image protocol extension.
 *
 * BEGIN request (little-endian):
 *   [0]    0x34
 *   [1:2]  x
 *   [3:4]  y
 *   [5:6]  width
 *   [7:8]  height
 *   [9]    flags (must be 0 in v1)
 *
 * REFRESH request:
 *   [0]    0x35
 *
 * Response:
 *   [0]    0xA2
 *   [1]    command (0x34 or 0x35)
 *   [2]    status
 *   [3:4]  partial refresh count (little-endian)
 *
 * v1 intentionally accepts only the full 800x480 region. The region fields
 * are part of the protocol now so arbitrary byte-aligned rectangles can be
 * enabled without changing the packet format later.
 */
#define EPD_CMD_BW_PARTIAL_BEGIN   0x34
#define EPD_CMD_BW_PARTIAL_REFRESH 0x35
#define EPD_RSP_BW_PARTIAL         0xA2

typedef enum {
    EPD_PARTIAL_OK = 0x00,
    EPD_PARTIAL_ERR_PARAM = 0x01,
    EPD_PARTIAL_ERR_UNSUPPORTED = 0x02,
    EPD_PARTIAL_ERR_NEED_FULL_REFRESH = 0x03,
    EPD_PARTIAL_ERR_STATE = 0x04,
    EPD_PARTIAL_ERR_INCOMPLETE = 0x05,
} epd_partial_status_t;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
    uint16_t bytes_per_line;
    uint32_t total_bytes;
    uint16_t block_payload_size;
    uint16_t partial_refresh_count;
    uint8_t flags;
    uint8_t active;
    uint8_t base_valid;
} bw_partial_state_t;

#endif
