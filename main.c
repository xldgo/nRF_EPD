/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved. */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_dfu.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_ble_gatt.h"
#include "nrf_bootloader_info.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "EPD_service.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "main.h"
#include "nrf_delay.h"
#include "nrf_drv_wdt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_power.h"
#include "nrf_pwr_mgmt.h"

#define CENTRAL_LINK_COUNT 0
#define PERIPHERAL_LINK_COUNT 1

#define DEVICE_NAME "NRF_EPD"
#define APP_ADV_INTERVAL 1600
#define APP_ADV_TIMEOUT_IN_SECONDS 120

#define APP_BLE_CONN_CFG_TAG 1
#define APP_BLE_OBSERVER_PRIO 3

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(7.5, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(30, UNIT_1_25_MS)
#define SLAVE_LATENCY 6
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000)
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000)
#define MAX_CONN_PARAMS_UPDATE_COUNT 3

/* The clock/calendar scheduler is gone. The scheduler is now only used by
 * command processing and FDS maintenance. */
#define SCHED_MAX_EVENT_DATA_SIZE EPD_GUI_SCHD_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE 10

/* Keep one system timer solely to service the watchdog while the CPU sleeps.
 * This timer has no wall-clock, calendar, or display-refresh semantics. */
#define WATCHDOG_SERVICE_INTERVAL APP_TIMER_TICKS(10000)

#define DEAD_BEEF 0xDEADBEEF

NRF_BLE_GATT_DEF(m_gatt);
BLE_ADVERTISING_DEF(m_advertising);
BLE_EPD_DEF(m_epd);
APP_TIMER_DEF(m_watchdog_timer_id);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_EPD_SVC, EPD_SVC_UUID_TYPE}};
static nrf_drv_wdt_channel_id m_wdt_channel_id;
static uint32_t m_resetreas;

/* Retained as a compatibility value for the legacy SET_TIME commands and
 * NRF_LOG timestamp callback. It is no longer advanced by a display clock. */
static uint32_t m_timestamp = 1735689600;

void assert_nrf_callback(uint16_t line_num, const uint8_t* p_file_name) {
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

uint32_t timestamp(void) { return m_timestamp; }

void set_timestamp(uint32_t value) { m_timestamp = value; }

void app_feed_wdt(void) { nrf_drv_wdt_channel_feed(m_wdt_channel_id); }

static void advertising_config_get(ble_adv_modes_config_t* p_config) {
    memset(p_config, 0, sizeof(*p_config));
    p_config->ble_adv_fast_enabled = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout = APP_ADV_TIMEOUT_IN_SECONDS * 100;
}

static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void* p_context) {
    UNUSED_PARAMETER(p_context);
    if (state == NRF_SDH_EVT_STATE_DISABLED) {
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) = {
    .handler = buttonless_dfu_sdh_state_observer,
};

static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event) {
    switch (event) {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE: {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);
            APP_ERROR_CHECK(sd_ble_gap_disconnect(
                m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION));
            break;
        }
        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;
        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchronously.");
            APP_ERROR_CHECK(false);
            break;
        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a DFU response failed.");
            APP_ERROR_CHECK(false);
            break;
        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}

static void watchdog_timer_timeout_handler(void* p_context) {
    UNUSED_PARAMETER(p_context);
    app_feed_wdt();
}

static void scheduler_init(void) {
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

static void timers_init(void) {
    APP_ERROR_CHECK(app_timer_init());
    APP_ERROR_CHECK(app_timer_create(
        &m_watchdog_timer_id, APP_TIMER_MODE_REPEATED, watchdog_timer_timeout_handler));
}

static void application_timers_start(void) {
    APP_ERROR_CHECK(app_timer_start(m_watchdog_timer_id, WATCHDOG_SERVICE_INTERVAL, NULL));
}

void sleep_mode_enter(void) {
    NRF_LOG_DEBUG("Entering deep sleep mode\n");
    NRF_LOG_FINAL_FLUSH();
    nrf_delay_ms(100);
    ble_epd_sleep_prepare(&m_epd);
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
}

static void services_init(void) {
    memset(&m_epd, 0, sizeof(m_epd));
    APP_ERROR_CHECK(ble_epd_init(&m_epd));

    ble_dfu_buttonless_init_t dfus_init = {0};
    dfus_init.evt_handler = ble_dfu_evt_handler;
    APP_ERROR_CHECK(ble_dfu_buttonless_init(&dfus_init));
}

static void gap_params_init(void) {
    char device_name[20];
    ble_gap_addr_t addr;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    APP_ERROR_CHECK(sd_ble_gap_addr_get(&addr));

    NRF_LOG_INFO("Bluetooth MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                 addr.addr[5], addr.addr[4], addr.addr[3],
                 addr.addr[2], addr.addr[1], addr.addr[0]);

    snprintf(device_name, sizeof(device_name), "%s_%02X%02X",
             DEVICE_NAME, addr.addr[1], addr.addr[0]);
    APP_ERROR_CHECK(sd_ble_gap_device_name_set(
        &sec_mode, (const uint8_t*)device_name, strlen(device_name)));

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;
    APP_ERROR_CHECK(sd_ble_gap_ppcp_set(&gap_conn_params));
}

static void on_conn_params_evt(ble_conn_params_evt_t* p_evt) {
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        NRF_LOG_WARNING("Conn params update failed, keeping connection");
    }
}

static void conn_params_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void) {
    ble_conn_params_init_t cp_init;
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    APP_ERROR_CHECK(ble_conn_params_init(&cp_init));
}

static void advertising_start(void) {
    NRF_LOG_INFO("advertising start\n");
    APP_ERROR_CHECK(ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST));
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
    switch (ble_adv_evt) {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("advertising timeout\n");
            if (m_epd.config.wakeup_pin != 0xFF) {
                sleep_mode_enter();
            } else {
                advertising_start();
            }
            break;
        default:
            break;
    }
}

static void on_ble_evt(ble_evt_t* p_ble_evt) {
    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("CONNECTED\n");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("DISCONNECTED, reason=0x%02X\n",
                         p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
            ble_gap_phys_t const phys = {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            APP_ERROR_CHECK(sd_ble_gap_phy_update(
                p_ble_evt->evt.gap_evt.conn_handle, &phys));
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            APP_ERROR_CHECK(sd_ble_gap_sec_params_reply(
                m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL));
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            APP_ERROR_CHECK(sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0));
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            APP_ERROR_CHECK(sd_ble_gap_disconnect(
                p_ble_evt->evt.gattc_evt.conn_handle,
                BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION));
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            APP_ERROR_CHECK(sd_ble_gap_disconnect(
                p_ble_evt->evt.gatts_evt.conn_handle,
                BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION));
            break;

        default:
            break;
    }
}

static void ble_evt_handler(ble_evt_t const* p_ble_evt, void* p_context) {
    UNUSED_PARAMETER(p_context);
    on_ble_evt((ble_evt_t*)p_ble_evt);
}

static void ble_stack_init(void) {
    APP_ERROR_CHECK(nrf_sdh_enable_request());

    uint32_t ram_start = 0;
    APP_ERROR_CHECK(nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start));
    APP_ERROR_CHECK(nrf_sdh_ble_enable(&ram_start));

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO,
                         ble_evt_handler, NULL);
}

void gatt_evt_handler(nrf_ble_gatt_t* p_gatt, nrf_ble_gatt_evt_t const* p_evt) {
    if ((m_conn_handle == p_evt->conn_handle) &&
        (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
        m_epd.max_data_len = p_evt->params.att_mtu_effective - 3;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)",
                     m_epd.max_data_len, m_epd.max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}

static void gatt_init(void) {
    APP_ERROR_CHECK(nrf_ble_gatt_init(&m_gatt, gatt_evt_handler));
    APP_ERROR_CHECK(nrf_ble_gatt_att_mtu_periph_set(
        &m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE));
}

static void advertising_init(void) {
    ble_advertising_init_t init;
    memset(&init, 0, sizeof(init));

    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids = m_adv_uuids;
    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_TIMEOUT_IN_SECONDS * 100;
    init.evt_handler = on_adv_evt;

    APP_ERROR_CHECK(ble_advertising_init(&m_advertising, &init));
    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void log_init(void) {
    APP_ERROR_CHECK(NRF_LOG_INIT(timestamp));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void power_management_init(void) {
    APP_ERROR_CHECK(nrf_pwr_mgmt_init());
}

static void idle_state_handle(void) {
    app_feed_wdt();
    if (NRF_LOG_PROCESS() == false) {
        nrf_pwr_mgmt_run();
    }
}

void wdt_event_handler(void) {
    NRF_LOG_ERROR("WDT Reset!\r\n");
    NRF_LOG_FINAL_FLUSH();
}

int main(void) {
    log_init();

    m_resetreas = NRF_POWER->RESETREAS;
    NRF_POWER->RESETREAS |= NRF_POWER->RESETREAS;
    NRF_LOG_DEBUG("== RESET REASON: %d ===\n", m_resetreas);

    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    APP_ERROR_CHECK(nrf_drv_wdt_init(&config, wdt_event_handler));
    APP_ERROR_CHECK(nrf_drv_wdt_channel_alloc(&m_wdt_channel_id));
    nrf_drv_wdt_enable();

    timers_init();
    power_management_init();
    ble_stack_init();
    scheduler_init();
    gap_params_init();
    gatt_init();
    ble_dfu_buttonless_async_svci_init();
    services_init();
    advertising_init();
    conn_params_init();

    application_timers_start();
    advertising_start();

    /* No boot-time calendar/clock render. The panel retains its image until an
     * explicit image/text command asks to change it. */
    for (;;) {
        app_sched_execute();
        idle_state_handle();
    }
}
