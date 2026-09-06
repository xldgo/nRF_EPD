#ifndef __GUI_H
#define __GUI_H

#include "Adafruit_GFX.h"

/* Legacy values are retained because display_mode is persisted in epd_config_t
 * and older clients may still send these numeric values. Runtime firmware in
 * feature/bw-partial-image normalizes every non-picture value to picture. */
typedef enum {
    MODE_PICTURE = 0,
    MODE_CALENDAR = 1,
    MODE_CLOCK_CALENDAR = 2,
    MODE_CLOCK = 3,
} display_mode_t;

/* Generic drawing context retained for text/GFX users. Clock/calendar drawing
 * is no longer part of the firmware runtime. */
typedef struct {
    display_mode_t mode;
    uint16_t color;
    uint16_t width;
    uint16_t height;
    uint32_t timestamp;
    uint8_t week_start;
    int8_t temperature;
    uint16_t voltage_mv;
    char ssid[32];
} gui_data_t;

#endif
