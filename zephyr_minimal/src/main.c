#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#define EPD_WIDTH 800U
#define EPD_HEIGHT 480U
#define EPD_APP_VERSION 0x21U

#define EPD_MAX_BLOCKS 512U
#define EPD_BLOCK_BITMAP_SIZE 64U

#define EPD_CMD_INIT 0x01U
#define EPD_CMD_REFRESH 0x05U
#define EPD_CMD_SLEEP 0x06U
#define EPD_CMD_WRITE_BLOCK 0x31U
#define EPD_CMD_QUERY_STATUS 0x32U
#define EPD_CMD_RESET_TRANSFER 0x33U

#define EPD_RSP_BLOCK_ACK 0xA0U
#define EPD_RSP_STATUS 0xA1U

#define UC81XX_PSR 0x00U
#define UC81XX_POF 0x02U
#define UC81XX_PON 0x04U
#define UC81XX_DSLP 0x07U
#define UC81XX_DTM1 0x10U
#define UC81XX_DRF 0x12U
#define UC81XX_DTM2 0x13U
#define UC81XX_CDI 0x50U
#define UC81XX_PTL 0x90U

#define EPD_SPI_FREQUENCY 4000000U

#define BT_UUID_EPD_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x62750001, 0xd828, 0x918d, 0xfb46, 0xb6c11c675aecULL)
#define BT_UUID_EPD_CHAR_VAL \
	BT_UUID_128_ENCODE(0x62750002, 0xd828, 0x918d, 0xfb46, 0xb6c11c675aecULL)
#define BT_UUID_EPD_VERSION_VAL \
	BT_UUID_128_ENCODE(0x62750003, 0xd828, 0x918d, 0xfb46, 0xb6c11c675aecULL)

static const struct bt_uuid_128 epd_service_uuid = BT_UUID_INIT_128(BT_UUID_EPD_SERVICE_VAL);
static const struct bt_uuid_128 epd_char_uuid = BT_UUID_INIT_128(BT_UUID_EPD_CHAR_VAL);
static const struct bt_uuid_128 epd_version_uuid = BT_UUID_INIT_128(BT_UUID_EPD_VERSION_VAL);

/* Keep the existing browser/ESP configuration payload byte-for-byte compatible. */
static const uint8_t epd_config[] = {
	0x14, 0x13, 0x06, 0x05, 0x04, 0x03, 0x02,
	0x07, 0xff, 0x12, 0x07, 0x01, 0x01, 0x00,
};

static const uint8_t epd_app_version = EPD_APP_VERSION;

struct transfer_context {
	uint8_t session_id;
	uint16_t total_blocks;
	uint16_t received_blocks;
	uint8_t block_bitmap[EPD_BLOCK_BITMAP_SIZE];
	bool active;
};

static struct transfer_context transfer_ctx;
static bool panel_initialized;

#define USER_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec epd_cs = GPIO_DT_SPEC_GET(USER_NODE, epd_cs_gpios);
static const struct gpio_dt_spec epd_dc = GPIO_DT_SPEC_GET(USER_NODE, epd_dc_gpios);
static const struct gpio_dt_spec epd_reset = GPIO_DT_SPEC_GET(USER_NODE, epd_reset_gpios);
static const struct gpio_dt_spec epd_busy = GPIO_DT_SPEC_GET(USER_NODE, epd_busy_gpios);
static const struct gpio_dt_spec epd_bs = GPIO_DT_SPEC_GET(USER_NODE, epd_bs_gpios);
static const struct gpio_dt_spec epd_enable = GPIO_DT_SPEC_GET(USER_NODE, epd_enable_gpios);

static const struct device *const epd_spi = DEVICE_DT_GET(DT_NODELABEL(spi1));
static const struct spi_config epd_spi_cfg = {
	.frequency = EPD_SPI_FREQUENCY,
	.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8),
	.slave = 0,
};

K_SEM_DEFINE(refresh_sem, 0, 1);

static int notify_payload(const void *data, uint16_t len);
static void handle_command(struct bt_conn *conn, const uint8_t *data, uint16_t len);

static int epd_spi_write(const uint8_t *data, size_t len)
{
	struct spi_buf buf = {
		.buf = (void *)data,
		.len = len,
	};
	const struct spi_buf_set tx = {
		.buffers = &buf,
		.count = 1,
	};

	int err = gpio_pin_set_dt(&epd_cs, 1);
	if (err != 0) {
		return err;
	}

	err = spi_write(epd_spi, &epd_spi_cfg, &tx);
	(void)gpio_pin_set_dt(&epd_cs, 0);
	return err;
}

static int epd_write_cmd(uint8_t cmd)
{
	int err = gpio_pin_set_dt(&epd_dc, 0);
	if (err != 0) {
		return err;
	}
	return epd_spi_write(&cmd, sizeof(cmd));
}

static int epd_write_data(const uint8_t *data, size_t len)
{
	int err = gpio_pin_set_dt(&epd_dc, 1);
	if (err != 0) {
		return err;
	}
	return epd_spi_write(data, len);
}

static int epd_write_cmd_data(uint8_t cmd, const uint8_t *data, size_t len)
{
	int err = epd_write_cmd(cmd);
	if (err != 0 || len == 0U) {
		return err;
	}
	return epd_write_data(data, len);
}

static int epd_wait_ready(uint32_t timeout_ms)
{
	const int64_t deadline = k_uptime_get() + (int64_t)timeout_ms;

	for (;;) {
		int busy = gpio_pin_get_dt(&epd_busy);
		if (busy < 0) {
			return busy;
		}
		if (busy == 0) {
			return 0;
		}
		if (k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}
		k_msleep(1);
	}
}

static int epd_reset_panel(void)
{
	int err = gpio_pin_set_dt(&epd_reset, 0); /* physical HIGH */
	if (err != 0) {
		return err;
	}
	k_msleep(10);

	err = gpio_pin_set_dt(&epd_reset, 1); /* physical LOW */
	if (err != 0) {
		return err;
	}
	k_msleep(10);

	err = gpio_pin_set_dt(&epd_reset, 0); /* physical HIGH */
	k_msleep(10);
	return err;
}

static int epd_panel_init(void)
{
	static const uint8_t psr = 0x0f;
	static const uint8_t cdi = 0x77;

	int err = epd_reset_panel();
	if (err != 0) {
		return err;
	}

	err = epd_write_cmd_data(UC81XX_PSR, &psr, sizeof(psr));
	if (err != 0) {
		return err;
	}

	err = epd_write_cmd_data(UC81XX_CDI, &cdi, sizeof(cdi));
	if (err == 0) {
		panel_initialized = true;
	}
	return err;
}

static int epd_power_on(void)
{
	int err = epd_write_cmd(UC81XX_PON);
	if (err != 0) {
		return err;
	}
	return epd_wait_ready(200);
}

static int epd_power_off(void)
{
	int err = epd_write_cmd(UC81XX_POF);
	if (err != 0) {
		return err;
	}
	return epd_wait_ready(200);
}

static int epd_set_full_area(void)
{
	/* x: 0..799, y: 0..479, byte-aligned, normal scan. */
	static const uint8_t area[] = {
		0x00, 0x00,
		0x03, 0x1f,
		0x00, 0x00,
		0x01, 0xdf,
		0x00,
	};

	return epd_write_cmd_data(UC81XX_PTL, area, sizeof(area));
}

static int epd_refresh(void)
{
	if (!panel_initialized) {
		return -EACCES;
	}

	int err = epd_power_on();
	if (err != 0) {
		return err;
	}

	err = epd_set_full_area();
	if (err != 0) {
		return err;
	}

	err = epd_write_cmd(UC81XX_DRF);
	if (err != 0) {
		return err;
	}

	k_msleep(100);
	err = epd_wait_ready(30000);
	if (err != 0) {
		return err;
	}

	return epd_power_off();
}

static int epd_sleep(void)
{
	static const uint8_t key = 0xa5;

	if (!panel_initialized) {
		return -EACCES;
	}

	int err = epd_power_off();
	if (err != 0) {
		return err;
	}
	k_msleep(100);
	return epd_write_cmd_data(UC81XX_DSLP, &key, sizeof(key));
}

static int epd_write_ram(uint8_t cfg, const uint8_t *data, size_t len)
{
	if (!panel_initialized) {
		return -EACCES;
	}

	const bool begin = (cfg >> 4) == 0U;
	const bool black = (cfg & 0x0fU) == 0x0fU;

	if (begin) {
		int err = epd_write_cmd(black ? UC81XX_DTM1 : UC81XX_DTM2);
		if (err != 0) {
			return err;
		}
	}

	return epd_write_data(data, len);
}

static int epd_io_init(void)
{
	if (!device_is_ready(epd_spi) ||
	    !gpio_is_ready_dt(&epd_cs) ||
	    !gpio_is_ready_dt(&epd_dc) ||
	    !gpio_is_ready_dt(&epd_reset) ||
	    !gpio_is_ready_dt(&epd_busy) ||
	    !gpio_is_ready_dt(&epd_bs) ||
	    !gpio_is_ready_dt(&epd_enable)) {
		return -ENODEV;
	}

	int err = gpio_pin_configure_dt(&epd_cs, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		return err;
	}
	err = gpio_pin_configure_dt(&epd_dc, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		return err;
	}
	err = gpio_pin_configure_dt(&epd_reset, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		return err;
	}
	err = gpio_pin_configure_dt(&epd_busy, GPIO_INPUT);
	if (err != 0) {
		return err;
	}
	err = gpio_pin_configure_dt(&epd_bs, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		return err;
	}
	return gpio_pin_configure_dt(&epd_enable, GPIO_OUTPUT_ACTIVE);
}

static uint16_t crc16_compute(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xffffU;

	for (size_t i = 0; i < len; ++i) {
		crc ^= data[i];
		for (uint8_t bit = 0; bit < 8U; ++bit) {
			crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0x8408U) : (uint16_t)(crc >> 1);
		}
	}

	return crc;
}

static size_t u32_to_dec(char *dst, uint32_t value)
{
	char reversed[10];
	size_t count = 0;

	do {
		reversed[count++] = (char)('0' + (value % 10U));
		value /= 10U;
	} while (value != 0U);

	for (size_t i = 0; i < count; ++i) {
		dst[i] = reversed[count - 1U - i];
	}
	return count;
}

static void send_mtu(struct bt_conn *conn)
{
	char msg[10] = {'m', 't', 'u', '='};
	uint16_t mtu = bt_gatt_get_mtu(conn);
	uint32_t payload = (mtu >= 3U) ? (uint32_t)(mtu - 3U) : 0U;
	size_t len = 4U + u32_to_dec(&msg[4], payload);
	(void)notify_payload(msg, (uint16_t)len);
}

static void send_time_stub(void)
{
	static const char msg[] = "t=0";
	(void)notify_payload(msg, sizeof(msg) - 1U);
}

static void send_block_response(uint16_t block_id, uint8_t status)
{
	uint8_t response[4] = {
		EPD_RSP_BLOCK_ACK,
		(uint8_t)(block_id & 0xffU),
		(uint8_t)(block_id >> 8),
		status,
	};
	(void)notify_payload(response, sizeof(response));
}

static void send_transfer_status(void)
{
	uint8_t response[7U + EPD_BLOCK_BITMAP_SIZE];
	uint16_t bitmap_len = (uint16_t)((transfer_ctx.total_blocks + 7U) / 8U);
	bitmap_len = MIN(bitmap_len, (uint16_t)EPD_BLOCK_BITMAP_SIZE);

	response[0] = EPD_RSP_STATUS;
	sys_put_le16(transfer_ctx.total_blocks, &response[1]);
	sys_put_le16(transfer_ctx.received_blocks, &response[3]);
	response[5] = transfer_ctx.session_id;
	response[6] = transfer_ctx.active ? 1U : 0U;
	memcpy(&response[7], transfer_ctx.block_bitmap, bitmap_len);
	(void)notify_payload(response, (uint16_t)(7U + bitmap_len));
}

static void reset_transfer(uint8_t session_id)
{
	memset(&transfer_ctx, 0, sizeof(transfer_ctx));
	transfer_ctx.session_id = session_id;
}

static void handle_write_block(const uint8_t *data, uint16_t len)
{
	if (len < 8U || !panel_initialized) {
		return;
	}

	const uint16_t block_id = sys_get_le16(&data[1]);
	const uint16_t total_blocks = sys_get_le16(&data[3]);
	const uint8_t cfg = data[5];
	const uint16_t payload_len = (uint16_t)(len - 8U);
	const uint8_t *payload = &data[6];
	const uint16_t recv_crc = sys_get_le16(&data[len - 2U]);

	if (crc16_compute(payload, payload_len) != recv_crc) {
		send_block_response(block_id, 0x01U);
		return;
	}

	if (total_blocks == 0U || total_blocks > EPD_MAX_BLOCKS ||
	    block_id >= total_blocks || block_id >= EPD_MAX_BLOCKS) {
		send_block_response(block_id, 0x02U);
		return;
	}

	if (block_id == 0U || !transfer_ctx.active) {
		transfer_ctx.total_blocks = total_blocks;
		transfer_ctx.received_blocks = 0U;
		memset(transfer_ctx.block_bitmap, 0, sizeof(transfer_ctx.block_bitmap));
		transfer_ctx.active = true;
	} else if (transfer_ctx.total_blocks != total_blocks) {
		transfer_ctx.total_blocks = total_blocks;
		transfer_ctx.received_blocks = 0U;
		memset(transfer_ctx.block_bitmap, 0, sizeof(transfer_ctx.block_bitmap));
	}

	const uint16_t byte_index = block_id / 8U;
	const uint8_t bit_mask = (uint8_t)BIT(block_id % 8U);

	if ((transfer_ctx.block_bitmap[byte_index] & bit_mask) == 0U) {
		if (epd_write_ram(cfg, payload, payload_len) != 0) {
			send_block_response(block_id, 0x02U);
			return;
		}

		transfer_ctx.block_bitmap[byte_index] |= bit_mask;
		transfer_ctx.received_blocks++;
	}

	send_block_response(block_id, 0x00U);
}

static ssize_t read_version(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &epd_app_version, sizeof(epd_app_version));
}

static ssize_t write_epd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len == 0U) {
		return 0;
	}

	handle_command(conn, buf, len);
	return len;
}

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	if (value == BT_GATT_CCC_NOTIFY) {
		(void)notify_payload(epd_config, sizeof(epd_config));
	}
}

BT_GATT_SERVICE_DEFINE(epd_svc,
	BT_GATT_PRIMARY_SERVICE(&epd_service_uuid),
	BT_GATT_CHARACTERISTIC(&epd_char_uuid.uuid,
		BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_WRITE, NULL, write_epd, NULL),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&epd_version_uuid.uuid,
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_version, NULL, NULL)
);

static int notify_payload(const void *data, uint16_t len)
{
	/* Attribute 2 is the value attribute of UUID ...0002. */
	return bt_gatt_notify(NULL, &epd_svc.attrs[2], data, len);
}

static void handle_command(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
	switch (data[0]) {
	case EPD_CMD_INIT:
		if (epd_panel_init() == 0) {
			send_mtu(conn);
			send_time_stub();
		}
		break;

	case EPD_CMD_REFRESH:
		if (panel_initialized) {
			k_sem_give(&refresh_sem);
		}
		break;

	case EPD_CMD_SLEEP:
		(void)epd_sleep();
		break;

	case EPD_CMD_WRITE_BLOCK:
		handle_write_block(data, len);
		break;

	case EPD_CMD_QUERY_STATUS:
		send_transfer_status();
		break;

	case EPD_CMD_RESET_TRANSFER:
		reset_transfer((len > 1U) ? data[1] : 0U);
		break;

	default:
		break;
	}
}

static const struct bt_data advertising_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1U),
};

static const struct bt_data scan_response_data[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_EPD_SERVICE_VAL),
};

static int start_advertising(void)
{
	return bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
			       advertising_data, ARRAY_SIZE(advertising_data),
			       scan_response_data, ARRAY_SIZE(scan_response_data));
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(reason);
	(void)start_advertising();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.disconnected = disconnected,
};

int main(void)
{
	if (epd_io_init() != 0) {
		return 0;
	}

	if (bt_enable(NULL) != 0) {
		return 0;
	}

	if (start_advertising() != 0) {
		return 0;
	}

	for (;;) {
		k_sem_take(&refresh_sem, K_FOREVER);
		(void)epd_refresh();
	}

	return 0;
}
