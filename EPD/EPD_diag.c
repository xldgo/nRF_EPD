#include "EPD_diag.h"

#include <string.h>

#include "EPD_service.h"
#include "ble_srv_common.h"
#include "nrf_log.h"
#include "nrf_sdh_ble.h"

#define EPD_DIAG_BLE_OBSERVER_PRIO 3

typedef enum {
    EPD_DIAG_HVX_OTHER = 0,
    EPD_DIAG_HVX_CONFIG,
    EPD_DIAG_HVX_MTU,
    EPD_DIAG_HVX_TIME,
    EPD_DIAG_HVX_A0,
    EPD_DIAG_HVX_A1,
} epd_diag_hvx_kind_t;

static ble_epd_t *m_epd;
static ble_gatts_char_handles_t m_diag_handles;
static epd_ble_diag_t m_diag __attribute__((aligned(4)));

static void epd_diag_snapshot_transfer(void) {
    if (m_epd == NULL) return;

    m_diag.notify_enabled = m_epd->is_notification_enabled ? 1 : 0;
    m_diag.conn_handle = m_epd->conn_handle;
    m_diag.received_blocks = m_epd->transfer_ctx.received_blocks;
    m_diag.total_blocks = m_epd->transfer_ctx.total_blocks;
}

static void epd_diag_reset(void) {
    memset(&m_diag, 0xFF, sizeof(m_diag));
    m_diag.version = EPD_DIAG_VERSION;
    m_diag.notify_enabled = 0;
    m_diag.conn_handle = BLE_CONN_HANDLE_INVALID;
    m_diag.received_blocks = 0;
    m_diag.hvx_success_count = 0;
    m_diag.hvn_tx_complete_count = 0;
    m_diag.total_blocks = 0;
}

static epd_diag_hvx_kind_t epd_diag_classify_hvx(ble_gatts_hvx_params_t const *p_hvx_params) {
    if (p_hvx_params == NULL || p_hvx_params->p_data == NULL ||
        p_hvx_params->p_len == NULL || *p_hvx_params->p_len == 0) {
        return EPD_DIAG_HVX_OTHER;
    }

    uint8_t const *data = p_hvx_params->p_data;
    uint16_t len = *p_hvx_params->p_len;

    if (m_epd != NULL &&
        p_hvx_params->handle == m_epd->char_handles.value_handle &&
        data == (uint8_t const *)&m_epd->config &&
        len == sizeof(epd_config_t)) {
        return EPD_DIAG_HVX_CONFIG;
    }

    if (len >= 4 && memcmp(data, "mtu=", 4) == 0) return EPD_DIAG_HVX_MTU;
    if (len >= 2 && data[0] == 't' && data[1] == '=') return EPD_DIAG_HVX_TIME;
    if (data[0] == EPD_RSP_BLOCK_ACK) return EPD_DIAG_HVX_A0;
    if (data[0] == EPD_RSP_STATUS) return EPD_DIAG_HVX_A1;

    return EPD_DIAG_HVX_OTHER;
}

static void epd_diag_record_hvx(epd_diag_hvx_kind_t kind, uint32_t rc) {
    switch (kind) {
        case EPD_DIAG_HVX_CONFIG:
            m_diag.config_hvx_rc = rc;
            break;
        case EPD_DIAG_HVX_MTU:
            m_diag.mtu_hvx_rc = rc;
            break;
        case EPD_DIAG_HVX_TIME:
            m_diag.time_hvx_rc = rc;
            break;
        case EPD_DIAG_HVX_A0:
            m_diag.last_a0_hvx_rc = rc;
            break;
        case EPD_DIAG_HVX_A1:
            m_diag.last_a1_hvx_rc = rc;
            break;
        default:
            break;
    }

    if (rc == NRF_SUCCESS && m_diag.hvx_success_count != UINT16_MAX) {
        m_diag.hvx_success_count++;
    }

    epd_diag_snapshot_transfer();
}

uint32_t epd_diag_hvx(uint16_t conn_handle, ble_gatts_hvx_params_t const *p_hvx_params) {
    epd_diag_hvx_kind_t kind = epd_diag_classify_hvx(p_hvx_params);
    uint32_t rc = sd_ble_gatts_hvx(conn_handle, p_hvx_params);

    epd_diag_record_hvx(kind, rc);

    NRF_LOG_INFO("[EPD_DIAG] HVX kind=%u handle=0x%04x rc=0x%08x",
                 (unsigned)kind,
                 p_hvx_params != NULL ? p_hvx_params->handle : 0u,
                 (unsigned)rc);

    return rc;
}

static uint32_t epd_diag_add_characteristic(ble_epd_t *p_epd) {
    ble_add_char_params_t params;
    memset(&params, 0, sizeof(params));

    params.uuid = BLE_UUID_EPD_DIAG;
    params.uuid_type = EPD_SVC_UUID_TYPE;
    params.max_len = sizeof(m_diag);
    params.init_len = sizeof(m_diag);
    params.p_init_value = (uint8_t *)&m_diag;
    params.is_var_len = false;
    params.char_props.read = 1;
    params.read_access = SEC_OPEN;
    params.write_access = SEC_NO_ACCESS;
    params.is_value_user = true;

    return characteristic_add(p_epd->service_handle, &params, &m_diag_handles);
}

uint32_t epd_diag_attach(void *p_context) {
    ble_epd_t *p_epd = (ble_epd_t *)p_context;
    if (p_epd == NULL) return NRF_ERROR_NULL;

    m_epd = p_epd;
    epd_diag_reset();
    epd_diag_snapshot_transfer();

    uint32_t rc = epd_diag_add_characteristic(p_epd);
    if (rc == NRF_SUCCESS) {
        NRF_LOG_INFO("[EPD_DIAG] read-only diagnostic characteristic added at handle 0x%04x",
                     m_diag_handles.value_handle);
    } else {
        NRF_LOG_ERROR("[EPD_DIAG] failed to add diagnostic characteristic: 0x%08x",
                      (unsigned)rc);
    }
    return rc;
}

static void epd_diag_ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
    (void)p_context;
    if (p_ble_evt == NULL || m_epd == NULL) return;

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
        case BLE_GAP_EVT_DISCONNECTED:
        case BLE_GATTS_EVT_WRITE:
            epd_diag_snapshot_transfer();
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE: {
            uint16_t count = p_ble_evt->evt.gatts_evt.params.hvn_tx_complete.count;
            uint32_t total = (uint32_t)m_diag.hvn_tx_complete_count + count;
            m_diag.hvn_tx_complete_count = (total > UINT16_MAX) ? UINT16_MAX : (uint16_t)total;
            epd_diag_snapshot_transfer();
            break;
        }

        default:
            break;
    }
}

NRF_SDH_BLE_OBSERVER(m_epd_diag_obs,
                     EPD_DIAG_BLE_OBSERVER_PRIO,
                     epd_diag_ble_evt_handler,
                     NULL);
