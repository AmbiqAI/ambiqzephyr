/** @file
 *  @brief Throughput Service
 */

/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/throughput.h>

#define LOG_LEVEL CONFIG_BT_THROUGHPUT_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(throughput);

#define THROUGHPUT_TX_PACKET_MAX  65535
#define THROUGHPUT_TRX_PACKET_MAX 10000
#define THROUGHPUT_MTU_REQ_CMD    0xFE
#define THROUGHPUT_TX_CMD         0xA5
#define THROUGHPUT_RX_CMD         0x5A
#define THROUGHPUT_TRX_CMD        0x88
#define THROUGHPUT_TX_DUMMY_BYTE  0xAA

struct bt_throughput throughput;
static uint16_t tx_length = 0;
static struct bt_gatt_attr *throughput_notify_ch;
static bool throughput_notif_enabled = false;

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	throughput.data.mtu = tx;
	tx_length = throughput.data.mtu - 3;
	LOG_INF("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {.att_mtu_updated = mtu_updated};

static void throughput_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	throughput_notif_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Throughput notifications %s", throughput_notif_enabled ? "enabled" : "disabled");
}

static ssize_t write_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

/* Throughput Service Declaration */
BT_GATT_SERVICE_DEFINE(throughput_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_THROUGHPUT),
		       BT_GATT_CHARACTERISTIC(BT_UUID_THROUGHPUT_TX_CHAR, BT_GATT_CHRC_NOTIFY,
					      BT_GATT_PERM_READ, NULL, NULL, NULL),
		       BT_GATT_CCC(throughput_ccc_cfg_changed,
				   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
		       BT_GATT_CHARACTERISTIC(BT_UUID_THROUGHPUT_RX_CHAR,
					      BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE,
					      NULL, write_callback, NULL), );

static ssize_t write_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t rx_data[4];

	memcpy(rx_data, buf, sizeof(rx_data));
	throughput.data.rx_index = (rx_data[1] << 8) | rx_data[0];

	if (throughput.data.rx_index == 1) {
		switch (rx_data[3]) {
		case THROUGHPUT_MTU_REQ_CMD:
			throughput.data.tx_index = 1;
			throughput.data.rx_last_index = 0;
			throughput.data.max_trans_packet = 1;
			memset(throughput.data.tx_payload, THROUGHPUT_MTU_REQ_CMD, tx_length);
			throughput.data.tx_payload[6] = throughput.data.mtu;
			throughput.data.tx_payload[7] = throughput.data.mtu;
			throughput.data.tx_payload[8] = throughput.data.mtu;
			throughput.data.tx_payload[9] = THROUGHPUT_TRX_PACKET_MAX & 0xFF;
			throughput.data.tx_payload[10] = (THROUGHPUT_TRX_PACKET_MAX >> 8) & 0xFF;
			bt_gatt_notify(NULL, throughput_notify_ch, throughput.data.tx_payload,
				       tx_length);
			break;

		case THROUGHPUT_TX_CMD:
			throughput.data.tx_index = 1;
			throughput.data.rx_last_index = 0;
			throughput.data.max_trans_packet = THROUGHPUT_TX_PACKET_MAX;
			throughput.data.trans_mode = THROUGHPUT_TRANS_MODE_TX;
			memset(throughput.data.tx_payload, THROUGHPUT_TX_DUMMY_BYTE, tx_length);
			bt_throughput_notify();
			break;

		case THROUGHPUT_RX_CMD:
			throughput.data.tx_index = 0;
			throughput.data.rx_last_index = 1;
			throughput.data.max_trans_packet = 0;
			throughput.data.trans_mode = THROUGHPUT_TRANS_MODE_RX;
			break;

		case THROUGHPUT_TRX_CMD:
			throughput.data.tx_index = 1;
			throughput.data.rx_last_index = 0;
			throughput.data.max_trans_packet = THROUGHPUT_TRX_PACKET_MAX;
			throughput.data.trans_mode = THROUGHPUT_TRANS_MODE_TRX;
			memset(throughput.data.tx_payload, THROUGHPUT_TX_DUMMY_BYTE, tx_length);
			bt_throughput_notify();
			break;

		default:
			break;
		}
	}

	if (throughput.data.rx_index > throughput.data.rx_last_index) {
		if (throughput.data.rx_index - throughput.data.rx_last_index == 1) {
			if (throughput.data.rx_index >= THROUGHPUT_TX_PACKET_MAX) {
				throughput.data.rx_last_index = 0;
				throughput.data.rx_index = 0;
				LOG_INF("Device received 65535 packages done");
			}
		} else {
			LOG_ERR("Device received error package index, last index is %d, now is %d!",
				throughput.data.rx_last_index, throughput.data.rx_index);
		}
	} else if ((throughput.data.rx_index < throughput.data.rx_last_index) &&
		   (throughput.data.rx_index > 0)) {
		LOG_ERR("Device received error package index, last index is %d, now is %d!!!",
			throughput.data.rx_last_index, throughput.data.rx_index);
	}

	/* Update previous package index*/
	throughput.data.rx_last_index = throughput.data.rx_index;

	return 0;
}

static void notify_callback(struct bt_conn *conn, void *user_data)
{
	ARG_UNUSED(conn);

	/* We can decide what the user_data can be further. */
	ARG_UNUSED(user_data);
}

static struct bt_gatt_notify_params notify_params = {
	.data = throughput.data.tx_payload,
	.func = notify_callback,
	.uuid = NULL,
};

static int bt_throughput_init(void)
{
	throughput_notify_ch =
		bt_gatt_find_by_uuid(throughput_svc.attrs, 0, BT_UUID_THROUGHPUT_TX_CHAR);
	notify_params.attr = throughput_notify_ch;

	bt_gatt_cb_register(&gatt_callbacks);

	return 0;
}

int bt_throughput_notify(void)
{
	int ret = 0;

	if (!throughput_notif_enabled) {
		return -EPERM;
	}

	do {
		if ((throughput.data.trans_mode != THROUGHPUT_TRANS_MODE_TX) &&
		    (throughput.data.trans_mode != THROUGHPUT_TRANS_MODE_TRX)) {
			ret = -ENOTSUP;
			break;
		}

		if (throughput.data.tx_index > throughput.data.max_trans_packet) {
			if (throughput.data.trans_mode == THROUGHPUT_TRANS_MODE_TRX) {
				/* Start to receive packets. */
				break;
			} else {
				/* Continue the next round TX testing. */
				throughput.data.tx_index = 1;
			}
		}

		notify_params.len = tx_length;
		notify_params.user_data = &tx_length;
		throughput.data.tx_payload[0] = (uint8_t)throughput.data.tx_index;
		throughput.data.tx_payload[1] = (uint8_t)(throughput.data.tx_index >> 8);
		ret = bt_gatt_notify_cb(throughput.conn, &notify_params);
		if (ret) {
			break;
		}

		throughput.data.tx_index++;
	} while (0);

	return ret;
}

int bt_throughput_conn_init(struct bt_conn *conn)
{
	if (!conn) {
		return -EINVAL;
	}

	throughput.data.tx_index = 0;
	throughput.data.rx_last_index = 0;
	throughput.data.trans_mode = THROUGHPUT_TRANS_MODE_NONE;
	throughput.data.mtu = bt_gatt_get_mtu(conn);
	throughput.conn = conn;

	return 0;
}

SYS_INIT(bt_throughput_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
