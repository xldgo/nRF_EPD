/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved. */

#include "EPD_service.h"

#include <stdio.h>
#include <string.h>

#include "app_scheduler.h"
#include "ble_srv_common.h"
#include "main.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_pwr_mgmt.h"
#include "sdk_macros.h"

/* Default nRF52811 pin configuration. The persisted epd_config_t layout is
 * intentionally unchanged in this refactor. */
#define EPD_CFG_52811 {0x14, 0x13, 0x06, 0x05, 0x04, 0x03, 0x02, 0x07, 0xFF, 0x12, 0x07}

static uint16_t crc16_compute(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
        }
    }
    return crc;
}

static void send_block_response(ble_epd_t* p_epd, uint16_t block_id, uint8_t status) {
    uint8_t rsp[] = {
        EPD_RSP_BLOCK_ACK,
        (uint8_t)(block_id & 0xFF),
        (uint8_t)(block_id >> 8),
        status,
    };
    ble_epd_string_send(p_epd, rsp, sizeof(rsp));
}

static void send_transfer_status(ble_epd_t* p_epd) {
    uint8_t rsp[7 + EPD_BLOCK_BITMAP_SIZE];
    rsp[0] = EPD_RSP_STATUS;
    rsp[1] = (uint8_t)(p_epd->transfer_ctx.total_blocks & 0xFF);
    rsp[2] = (uint8_t)(p_epd->transfer_ctx.total_blocks >> 8);
    rsp[3] = (uint8_t)(p_epd->transfer_ctx.received_blocks & 0xFF);
    rsp[4] = (uint8_t)(p_epd->transfer_ctx.received_blocks >> 8);
    rsp[5] = p_epd->transfer_ctx.session_id;
    rsp[6] = p_epd->transfer_ctx.transfer_active ? 1 : 0;

    uint16_t bitmap_len = (p_epd->transfer_ctx.total_blocks + 7) / 8;
    if (bitmap_len > EPD_BLOCK_BITMAP_SIZE) {
        bitmap_len = EPD_BLOCK_BITMAP_SIZE;
    }
    memcpy(&rsp[7], p_epd->transfer_ctx.block_bitmap, bitmap_len);
    ble_epd_string_send(p_epd, rsp, 7 + bitmap_len);
}

static void epd_cmd_queue_init(epd_cmd_queue_t* queue) {
    memset(queue, 0, sizeof(*queue));
}

static bool epd_cmd_queue_push(epd_cmd_queue_t* queue,
                               const uint8_t* p_data,
                               uint16_t length) {
    if (queue->count >= EPD_CMD_QUEUE_SIZE) {
        NRF_LOG_WARNING("[EPD]: command queue full, dropping 0x%02x\n", p_data[0]);
        return false;
    }

    if (length > EPD_CMD_MAX_DATA_LEN) {
        NRF_LOG_WARNING("[EPD]: command 0x%02x too large (%d)\n", p_data[0], length);
        return false;
    }

    epd_cmd_queue_item_t* item = &queue->items[queue->tail];
    item->cmd = p_data[0];
    item->length = length;
    memcpy(item->data, p_data, length);

    queue->tail = (queue->tail + 1) % EPD_CMD_QUEUE_SIZE;
    queue->count++;
    return true;
}

static bool epd_cmd_queue_pop(epd_cmd_queue_t* queue, epd_cmd_queue_item_t* item) {
    if (queue->count == 0) {
        return false;
    }

    memcpy(item, &queue->items[queue->head], sizeof(*item));
    queue->head = (queue->head + 1) % EPD_CMD_QUEUE_SIZE;
    queue->count--;
    return true;
}

static bool is_urgent_command(uint8_t cmd) {
    return cmd == EPD_CMD_SYS_RESET ||
           cmd == EPD_CMD_SYS_SLEEP ||
           cmd == EPD_CMD_CFG_ERASE ||
           cmd == EPD_CMD_WRITE_BLOCK ||
           cmd == EPD_CMD_QUERY_STATUS ||
           cmd == EPD_CMD_RESET_TRANSFER;
}

static void epd_process_next_command(void* p_event_data, uint16_t event_size);

/* Legacy display modes remain encoded in flash for layout compatibility, but
 * the runtime now has exactly one persisted display mode: picture. */
static void epd_use_picture_mode(ble_epd_t* p_epd) {
    if (p_epd->config.display_mode != MODE_PICTURE) {
        NRF_LOG_INFO("[EPD]: legacy display mode %d -> picture\n",
                     p_epd->config.display_mode);
        p_epd->config.display_mode = MODE_PICTURE;
        epd_config_write(&p_epd->config);
    }
}

static uint32_t parse_legacy_time_payload(const uint8_t* p_data, uint16_t length) {
    uint32_t value = ((uint32_t)p_data[1] << 24) |
                     ((uint32_t)p_data[2] << 16) |
                     ((uint32_t)p_data[3] << 8) |
                     (uint32_t)p_data[4];
    int8_t timezone_hours = (length > 5) ? (int8_t)p_data[5] : 8;
    return value + ((int32_t)timezone_hours * 60 * 60);
}

static void epd_send_voltage(ble_epd_t* p_epd) {
    char buf[16] = {0};
    uint16_t voltage_mv = EPD_ReadVoltageAndCache();
    snprintf(buf, sizeof(buf), "v=%u", voltage_mv);
    ble_epd_string_send(p_epd, (uint8_t*)buf, strlen(buf));
}

static void epd_send_mtu(ble_epd_t* p_epd) {
    char buf[10] = {0};
    snprintf(buf, sizeof(buf), "mtu=%d", p_epd->max_data_len);
    ble_epd_string_send(p_epd, (uint8_t*)buf, strlen(buf));
}

static void on_connect(ble_epd_t* p_epd, ble_evt_t* p_ble_evt) {
    p_epd->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    EPD_GPIO_Init();
    EPD_LED_BLINK();
}

static void on_disconnect(ble_epd_t* p_epd, ble_evt_t* p_ble_evt) {
    UNUSED_PARAMETER(p_ble_evt);
    p_epd->conn_handle = BLE_CONN_HANDLE_INVALID;
    EPD_LED_BLINK();

    if (p_epd->epd != NULL) {
        p_epd->epd->drv->sleep(p_epd->epd);
        nrf_delay_ms(200);
        EPD_GPIO_Uninit();
    }
    app_feed_wdt();
}

static void epd_execute_command(ble_epd_t* p_epd, uint8_t* p_data, uint16_t length) {
    if (p_data == NULL || length == 0) {
        return;
    }

    NRF_LOG_DEBUG("[EPD]: executing command 0x%02x, len=%d\n", p_data[0], length);
    app_feed_wdt();

    switch (p_data[0]) {
        case EPD_CMD_SET_PINS:
            if (length < 8) {
                break;
            }
            p_epd->config.mosi_pin = p_data[1];
            p_epd->config.sclk_pin = p_data[2];
            p_epd->config.cs_pin = p_data[3];
            p_epd->config.dc_pin = p_data[4];
            p_epd->config.rst_pin = p_data[5];
            p_epd->config.busy_pin = p_data[6];
            p_epd->config.bs_pin = p_data[7];
            if (length > 8) {
                p_epd->config.en_pin = p_data[8];
            }
            epd_config_write(&p_epd->config);
            EPD_GPIO_Uninit();
            EPD_GPIO_Load(&p_epd->config);
            EPD_GPIO_Init();
            break;

        case EPD_CMD_INIT:
            p_epd->epd = epd_init((epd_model_id_t)(
                length > 1 ? p_data[1] : p_epd->config.model_id));
            if (p_epd->epd != NULL && p_epd->epd->id != p_epd->config.model_id) {
                p_epd->config.model_id = p_epd->epd->id;
                epd_config_write(&p_epd->config);
            }
            epd_send_mtu(p_epd);
            epd_send_voltage(p_epd);
            break;

        case EPD_CMD_CLEAR:
            if (p_epd->epd == NULL) {
                break;
            }
            epd_use_picture_mode(p_epd);
            p_epd->epd->drv->clear(p_epd->epd, length > 1 ? p_data[1] : true);
            break;

        case EPD_CMD_SEND_COMMAND:
            if (length < 2) {
                break;
            }
            epd_use_picture_mode(p_epd);
            EPD_WriteCmd(p_data[1]);
            break;

        case EPD_CMD_SEND_DATA:
            if (length < 2) {
                break;
            }
            epd_use_picture_mode(p_epd);
            EPD_WriteData(&p_data[1], length - 1);
            break;

        case EPD_CMD_REFRESH:
            if (p_epd->epd == NULL) {
                break;
            }
            /* 0x05 intentionally remains full-refresh only. */
            epd_use_picture_mode(p_epd);
            p_epd->is_refreshing = true;
            p_epd->epd->drv->refresh(p_epd->epd);
            p_epd->is_refreshing = false;
            break;

        case EPD_CMD_SLEEP:
            if (p_epd->epd != NULL) {
                p_epd->epd->drv->sleep(p_epd->epd);
            }
            break;

        /* Legacy time/config commands are retained only for wire compatibility.
         * They no longer select a clock/calendar screen or trigger a refresh. */
        case EPD_CMD_SET_TIME:
        case EPD_CMD_SYNC_TIME_SILENT:
            if (length >= 5) {
                set_timestamp(parse_legacy_time_payload(p_data, length));
            }
            break;

        case EPD_CMD_SET_WEEK_START:
            if (length >= 2 && p_data[1] < 7 &&
                p_data[1] != p_epd->config.week_start) {
                p_epd->config.week_start = p_data[1];
                epd_config_write(&p_epd->config);
            }
            break;

        case EPD_CMD_SET_MODE:
            if (length >= 2 && p_data[1] != MODE_PICTURE) {
                NRF_LOG_INFO("[EPD]: legacy display mode %d ignored\n", p_data[1]);
            }
            epd_use_picture_mode(p_epd);
            break;

        case EPD_CMD_WRITE_IMAGE:
            if (length < 3 || p_epd->epd == NULL) {
                break;
            }
            epd_use_picture_mode(p_epd);
            p_epd->epd->drv->write_ram(
                p_epd->epd, p_data[1], &p_data[2], length - 2);
            break;

        case EPD_CMD_WRITE_BLOCK: {
            if (length < 8 || p_epd->epd == NULL) {
                break;
            }

            uint16_t block_id = (uint16_t)p_data[1] | ((uint16_t)p_data[2] << 8);
            uint16_t total_blocks = (uint16_t)p_data[3] | ((uint16_t)p_data[4] << 8);
            uint8_t cfg = p_data[5];
            uint16_t payload_len = length - 8;
            uint8_t* payload = &p_data[6];
            uint16_t recv_crc = (uint16_t)p_data[length - 2] |
                                ((uint16_t)p_data[length - 1] << 8);
            uint16_t calc_crc = crc16_compute(payload, payload_len);

            if (calc_crc != recv_crc) {
                NRF_LOG_WARNING("[EPD]: block %d CRC mismatch\n", block_id);
                send_block_response(p_epd, block_id, 0x01);
                break;
            }

            if (block_id >= EPD_MAX_BLOCKS || block_id >= total_blocks ||
                total_blocks == 0 || total_blocks > EPD_MAX_BLOCKS) {
                send_block_response(p_epd, block_id, 0x02);
                break;
            }

            if (block_id == 0 || !p_epd->transfer_ctx.transfer_active) {
                p_epd->transfer_ctx.total_blocks = total_blocks;
                p_epd->transfer_ctx.received_blocks = 0;
                memset(p_epd->transfer_ctx.block_bitmap, 0, EPD_BLOCK_BITMAP_SIZE);
                p_epd->transfer_ctx.transfer_active = true;
                epd_use_picture_mode(p_epd);
            } else if (p_epd->transfer_ctx.total_blocks != total_blocks) {
                p_epd->transfer_ctx.total_blocks = total_blocks;
                p_epd->transfer_ctx.received_blocks = 0;
                memset(p_epd->transfer_ctx.block_bitmap, 0, EPD_BLOCK_BITMAP_SIZE);
            }

            uint16_t byte_idx = block_id / 8;
            uint8_t bit_idx = block_id % 8;
            if (!(p_epd->transfer_ctx.block_bitmap[byte_idx] & (1u << bit_idx))) {
                p_epd->epd->drv->write_ram(p_epd->epd, cfg, payload, payload_len);
                p_epd->transfer_ctx.block_bitmap[byte_idx] |= (1u << bit_idx);
                p_epd->transfer_ctx.received_blocks++;
            }

            send_block_response(p_epd, block_id, 0x00);
            break;
        }

        case EPD_CMD_QUERY_STATUS:
            send_transfer_status(p_epd);
            break;

        case EPD_CMD_RESET_TRANSFER:
            memset(&p_epd->transfer_ctx, 0, sizeof(p_epd->transfer_ctx));
            if (length > 1) {
                p_epd->transfer_ctx.session_id = p_data[1];
            }
            break;

        case EPD_CMD_SET_CONFIG:
            if (length < 2) {
                break;
            }
            memcpy(&p_epd->config, &p_data[1],
                   (length - 1 > EPD_CONFIG_SIZE) ? EPD_CONFIG_SIZE : length - 1);
            /* Preserve struct size/field order, but normalize deprecated modes. */
            p_epd->config.display_mode = MODE_PICTURE;
            epd_config_write(&p_epd->config);
            break;

        case EPD_CMD_SYS_SLEEP:
            sleep_mode_enter();
            break;

        case EPD_CMD_SYS_RESET:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_RESET);
            break;

        case EPD_CMD_CFG_ERASE:
            epd_config_clear(&p_epd->config);
            nrf_delay_ms(100);
            NVIC_SystemReset();
            break;

        default:
            break;
    }

    app_feed_wdt();
}

static void epd_process_next_command(void* p_event_data, uint16_t event_size) {
    UNUSED_PARAMETER(event_size);
    ble_epd_t* p_epd = *((ble_epd_t**)p_event_data);
    epd_cmd_queue_item_t item;

    p_epd->cmd_queue.is_processing = true;
    if (!epd_cmd_queue_pop(&p_epd->cmd_queue, &item)) {
        p_epd->cmd_queue.is_processing = false;
        return;
    }

    epd_execute_command(p_epd, item.data, item.length);

    if (p_epd->cmd_queue.count > 0) {
        uint32_t err_code = app_sched_event_put(
            &p_epd, sizeof(p_epd), epd_process_next_command);
        if (err_code != NRF_SUCCESS) {
            p_epd->cmd_queue.is_processing = false;
            NRF_LOG_ERROR("[EPD]: failed to schedule next command: 0x%08x\n", err_code);
        }
    } else {
        p_epd->cmd_queue.is_processing = false;
    }
}

static void epd_service_on_write(ble_epd_t* p_epd, uint8_t* p_data, uint16_t length) {
    if (p_data == NULL || length == 0) {
        return;
    }

    if (is_urgent_command(p_data[0])) {
        epd_execute_command(p_epd, p_data, length);
        return;
    }

    if (!epd_cmd_queue_push(&p_epd->cmd_queue, p_data, length)) {
        return;
    }

    if (!p_epd->cmd_queue.is_processing) {
        uint32_t err_code = app_sched_event_put(
            &p_epd, sizeof(p_epd), epd_process_next_command);
        if (err_code != NRF_SUCCESS) {
            NRF_LOG_ERROR("[EPD]: failed to schedule command: 0x%08x\n", err_code);
        }
    }
}

static void on_write(ble_epd_t* p_epd, ble_evt_t* p_ble_evt) {
    ble_gatts_evt_write_t* p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_epd->char_handles.cccd_handle &&
        p_evt_write->len == 2) {
        if (ble_srv_is_notification_enabled(p_evt_write->data)) {
            p_epd->is_notification_enabled = true;
            uint16_t length = sizeof(epd_config_t);
            uint32_t err_code = ble_epd_string_send(
                p_epd, (uint8_t*)&p_epd->config, length);
            if (err_code != NRF_ERROR_INVALID_STATE) {
                APP_ERROR_CHECK(err_code);
            }
        } else {
            p_epd->is_notification_enabled = false;
        }
    } else if (p_evt_write->handle == p_epd->char_handles.value_handle) {
        epd_service_on_write(p_epd, p_evt_write->data, p_evt_write->len);
    }
}

void ble_epd_evt_handler(ble_evt_t const* p_ble_evt, void* p_context) {
    if (p_context == NULL || p_ble_evt == NULL) {
        return;
    }
    ble_epd_on_ble_evt((ble_epd_t*)p_context, (ble_evt_t*)p_ble_evt);
}

void ble_epd_on_ble_evt(ble_epd_t* p_epd, ble_evt_t* p_ble_evt) {
    if (p_epd == NULL || p_ble_evt == NULL) {
        return;
    }

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_epd, p_ble_evt);
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_epd, p_ble_evt);
            break;
        case BLE_GATTS_EVT_WRITE:
            on_write(p_epd, p_ble_evt);
            break;
        default:
            break;
    }
}

static uint32_t epd_service_init(ble_epd_t* p_epd) {
    ble_uuid_t ble_uuid = {0};
    ble_uuid128_t base_uuid = BLE_UUID_EPD_SVC_BASE;
    ble_add_char_params_t add_char_params;
    uint8_t app_version = APP_VERSION;

    VERIFY_SUCCESS(sd_ble_uuid_vs_add(&base_uuid, &ble_uuid.type));

    ble_uuid.uuid = BLE_UUID_EPD_SVC;
    VERIFY_SUCCESS(sd_ble_gatts_service_add(
        BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_epd->service_handle));

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = BLE_UUID_EPD_CHAR;
    add_char_params.uuid_type = ble_uuid.type;
    add_char_params.max_len = BLE_EPD_MAX_DATA_LEN;
    add_char_params.init_len = sizeof(uint8_t);
    add_char_params.is_var_len = true;
    add_char_params.char_props.notify = 1;
    add_char_params.char_props.write = 1;
    add_char_params.char_props.write_wo_resp = 1;
    add_char_params.read_access = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;
    VERIFY_SUCCESS(characteristic_add(
        p_epd->service_handle, &add_char_params, &p_epd->char_handles));

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = BLE_UUID_APP_VER;
    add_char_params.uuid_type = ble_uuid.type;
    add_char_params.max_len = sizeof(uint8_t);
    add_char_params.init_len = sizeof(uint8_t);
    add_char_params.p_init_value = &app_version;
    add_char_params.char_props.read = 1;
    add_char_params.read_access = SEC_OPEN;

    return characteristic_add(
        p_epd->service_handle, &add_char_params, &p_epd->app_ver_handles);
}

void ble_epd_sleep_prepare(ble_epd_t* p_epd) {
    EPD_LED_OFF();
    if (p_epd->config.wakeup_pin != 0xFF) {
        nrf_gpio_cfg_sense_input(
            p_epd->config.wakeup_pin, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
    }
}

uint32_t ble_epd_init(ble_epd_t* p_epd) {
    if (p_epd == NULL) {
        return NRF_ERROR_NULL;
    }

    p_epd->max_data_len = BLE_EPD_MAX_DATA_LEN;
    p_epd->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_epd->is_notification_enabled = false;
    p_epd->force_full_refresh = false;
    p_epd->is_refreshing = false;
    p_epd->low_battery_screen_active = false;
    memset(&p_epd->transfer_ctx, 0, sizeof(p_epd->transfer_ctx));
    epd_cmd_queue_init(&p_epd->cmd_queue);

    epd_config_init(&p_epd->config);
    epd_config_read(&p_epd->config);

    if (epd_config_empty(&p_epd->config)) {
        uint8_t cfg[] = EPD_CFG_52811;
        memcpy(&p_epd->config, cfg, sizeof(cfg));
        p_epd->config.display_mode = MODE_PICTURE;
        if (p_epd->config.week_start == 0xFF) {
            p_epd->config.week_start = 0;
        }
        if (p_epd->config.full_refresh_minutes == 0xFF) {
            p_epd->config.full_refresh_minutes = 0;
        }
        epd_config_write(&p_epd->config);
    } else if (p_epd->config.display_mode != MODE_PICTURE) {
        /* Old MODE_CALENDAR / MODE_CLOCK_CALENDAR / MODE_CLOCK values fall
         * back to picture without changing the persisted struct layout. */
        p_epd->config.display_mode = MODE_PICTURE;
        epd_config_write(&p_epd->config);
    }

    EPD_GPIO_Load(&p_epd->config);
    EPD_GPIO_Init();
    p_epd->epd = epd_init((epd_model_id_t)p_epd->config.model_id);
    EPD_GPIO_Uninit();
    EPD_LED_BLINK();

    return epd_service_init(p_epd);
}

uint32_t ble_epd_string_send(ble_epd_t* p_epd,
                             uint8_t* p_string,
                             uint16_t length) {
    if (p_epd->conn_handle == BLE_CONN_HANDLE_INVALID ||
        !p_epd->is_notification_enabled) {
        return NRF_ERROR_INVALID_STATE;
    }
    if (length > p_epd->max_data_len) {
        return NRF_ERROR_INVALID_PARAM;
    }

    ble_gatts_hvx_params_t hvx_params;
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_epd->char_handles.value_handle;
    hvx_params.p_data = p_string;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    return sd_ble_gatts_hvx(p_epd->conn_handle, &hvx_params);
}

/* Compatibility symbol for older callers. Automatic clock/calendar refresh
 * has been removed; this function deliberately performs no display work. */
void ble_epd_on_timer(ble_epd_t* p_epd, uint32_t current_timestamp, bool force_update) {
    UNUSED_PARAMETER(p_epd);
    UNUSED_PARAMETER(current_timestamp);
    UNUSED_PARAMETER(force_update);
}
