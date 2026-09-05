#ifndef __EPD_DIAG_H
#define __EPD_DIAG_H

#include <stdint.h>

#include "ble.h"

#define BLE_UUID_EPD_DIAG 0x0004
#define EPD_DIAG_VERSION  1
#define EPD_DIAG_RC_NOT_ATTEMPTED 0xFFFFFFFFu

/**
 * Read-only diagnostic characteristic payload.
 * UUID: 62750004-d828-918d-fb46-b6c11c675aec
 *
 * All multi-byte fields are little-endian on nRF52.
 * The structure is intentionally fixed at 32 bytes so an external BLE central
 * can inspect Notification/HVX state without RTT, UART or SWD.
 */
typedef struct __attribute__((packed)) {
    uint8_t  version;               /* 0: diagnostic format version */
    uint8_t  notify_enabled;        /* 1: application-side CCCD state */
    uint16_t conn_handle;           /* 2: current BLE connection handle */

    uint32_t config_hvx_rc;         /* 4: config notification after CCCD */
    uint32_t mtu_hvx_rc;            /* 8: "mtu=..." notification */
    uint32_t time_hvx_rc;           /* 12: "t=..." notification */
    uint32_t last_a0_hvx_rc;        /* 16: last 0xA0 ACK/NACK */
    uint32_t last_a1_hvx_rc;        /* 20: last 0xA1 transfer status */

    uint16_t received_blocks;       /* 24: nRF transfer_ctx.received_blocks */
    uint16_t hvx_success_count;     /* 26: intercepted HVX calls returning NRF_SUCCESS */
    uint16_t hvn_tx_complete_count; /* 28: completed notification transmissions */
    uint16_t total_blocks;          /* 30: nRF transfer_ctx.total_blocks */
} epd_ble_diag_t;

typedef char epd_ble_diag_size_must_be_32[(sizeof(epd_ble_diag_t) == 32) ? 1 : -1];

struct ble_epd_s;

uint32_t epd_diag_attach(void *p_epd);
uint32_t epd_diag_hvx(uint16_t conn_handle, ble_gatts_hvx_params_t const *p_hvx_params);

#endif /* __EPD_DIAG_H */
