/* main.c - Application main entry point */

/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/throughput.h>

/* The connection interval may affect the throughput rate, especially the
 * uplink throughput rate. We can set the different interval for testing.
 * The throughput rate can be higher if the ACL data TX/RX duty cycle is
 * higher in each connection event.
 * 
 * Uplink (peripheral_throughput -> connected central):
 * For Apollo3 Blue series, the BLE controller has 4 TX buffer and maybe
 * unavailable to send the packets to the peer device very timely. Setting
 * interval to 15ms ~ 18ms may have higher uplink throughput rate in the
 * conditions of MTU = 247, PHY = 1M.
 * For Apollo4 Blue series, the BLE controller has 14 TX buffer, it seems
 * higher interval can have higher uplink throughput rate because the BLE
 * controller is always able to send the packets timely then the host can
 * send the next packets soon.
 * 
 * Downlink (peripheral_throughput <- connected central):
 * Higher connection interval can have higher RX duty cycle in each
 * connection event for Apollo3/4 Blue series so the downlink throughput
 * rate can be higher.
 */
#if (CONFIG_SOC_SERIES_APOLLO3X)
#define UPDATE_PARAM_INTERVAL_MIN 12 /* 15ms */
#define UPDATE_PARAM_INTERVAL_MAX 15 /* 18.75ms */
#else
#define UPDATE_PARAM_INTERVAL_MIN 32 /* 40ms */
#define UPDATE_PARAM_INTERVAL_MAX 40 /* 50ms */
#endif /* CONFIG_SOC_SERIES_APOLLO3X */
#define UPDATE_PARAM_LATENCY      0
#define UPDATE_PARAM_TIMEOUT      300

static struct bt_le_conn_param update_params = {
	.interval_min = UPDATE_PARAM_INTERVAL_MIN,
	.interval_max = UPDATE_PARAM_INTERVAL_MAX,
	.latency = UPDATE_PARAM_LATENCY,
	.timeout = UPDATE_PARAM_TIMEOUT,
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR))};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
		bt_throughput_conn_init(conn);

		err = bt_conn_le_param_update(conn, &update_params);
		if (err) {
			printk("Parameter update failed (err %d)\n", err);
		}
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.cancel = auth_cancel,
};

static void throughput_notify(void)
{
	bt_throughput_notify();
}

int main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return err;
	}

	bt_ready();

	bt_conn_auth_cb_register(&auth_cb_display);

	while (1) {
		k_sleep(K_USEC(1));
		throughput_notify();
	}
	return 0;
}
