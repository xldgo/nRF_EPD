/*
 * Diagnostic build wrapper around the original f455f3a EPD_service.c.
 *
 * The original service implementation is intentionally left untouched. This
 * translation unit redirects only its sd_ble_gatts_hvx() calls through
 * epd_diag_hvx(), renames the original ble_epd_init(), then exposes a thin
 * wrapper that appends the read-only diagnostic characteristic.
 */

#include "EPD_service.h"
#include "EPD_diag.h"
#include "nrf_log.h"

/* Device under test runs 0x20. The diagnostic OTA must be a monotonic upgrade. */
#undef APP_VERSION
#define APP_VERSION 0x21

/* Redirect HVX calls made by the original EPD service only. */
#define sd_ble_gatts_hvx epd_diag_hvx

/* Rename the original initializer so we can append diagnostics afterwards. */
#define ble_epd_init ble_epd_init_original

#include "EPD_service.c"

#undef ble_epd_init
#undef sd_ble_gatts_hvx

uint32_t ble_epd_init(ble_epd_t *p_epd) {
    uint32_t rc = ble_epd_init_original(p_epd);
    if (rc != NRF_SUCCESS) return rc;

    /*
     * Diagnostics must never make an otherwise bootable EPD application fail
     * its initialization. If the extra attribute cannot be added, keep the
     * original application alive and report the failure through RTT when
     * available.
     */
    uint32_t diag_rc = epd_diag_attach(p_epd);
    if (diag_rc != NRF_SUCCESS) {
        NRF_LOG_ERROR("[EPD_DIAG] attach failed: 0x%08x", (unsigned)diag_rc);
    }

    return rc;
}
