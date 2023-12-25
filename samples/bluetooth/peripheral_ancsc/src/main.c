/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/ancsc.h>

#include <zephyr/settings/settings.h>

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* Allocated size for attribute data. */
#define ATTR_DATA_SIZE BT_ANCS_ATTR_DATA_MAX

enum {
	DISCOVERY_ANCS_INIT,
	DISCOVERY_ANCS_SERVICE,
	DISCOVERY_NOTIFICATION_SOURCE_CHRC,
	DISCOVERY_CONTROL_POINT_CHRC,
	DISCOVERY_DATA_SOURCE_CHRC,
	DISCOVERY_ANCS_FINISH,
	DISCOVERY_ANCS_FAIL
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_LIMITED | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_SOLICIT128, BT_UUID_ANCS_VAL),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static struct bt_uuid_16 discover_uuid_16 = BT_UUID_INIT_16(0);
static struct bt_uuid_128 discover_uuid_128 = BT_UUID_INIT_128(0);
static struct bt_gatt_discover_params discover_params;

static struct bt_ancsc ancsc;
static uint8_t discovery_state = DISCOVERY_ANCS_INIT;

static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	switch (discovery_state) {
	case DISCOVERY_ANCS_SERVICE:
		if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_ANCS)) {
			memcpy(&discover_uuid_128, BT_UUID_ANCS_NOTIFICATION_SOURCE,
			       sizeof(discover_uuid_128));
			discover_params.uuid = &discover_uuid_128.uuid;
			discover_params.start_handle = attr->handle + 1;
			discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				printk("Discover failed (err %d)\n", err);
				discovery_state = DISCOVERY_ANCS_FAIL;
			}
			discovery_state = DISCOVERY_NOTIFICATION_SOURCE_CHRC;
		} else {
			discovery_state = DISCOVERY_ANCS_FAIL;
		}
		break;

	case DISCOVERY_NOTIFICATION_SOURCE_CHRC:
		if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_ANCS_NOTIFICATION_SOURCE)) {
			ancsc.handle[BT_ANCS_NOTIFICATION_SOURCE_HDL_IDX] =
				bt_gatt_attr_value_handle(attr);

			/* Discover the CCCD of notification source characteristic */
			memcpy(&discover_uuid_16, BT_UUID_GATT_CCC, sizeof(discover_uuid_16));
			discover_params.uuid = &discover_uuid_16.uuid;
			discover_params.start_handle = attr->handle + 2;
			discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;

			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				printk("Discover failed (err %d)\n", err);
				discovery_state = DISCOVERY_ANCS_FAIL;
			}
		} else {
			ancsc.handle[BT_ANCS_NOTIFICATION_SOURCE_CCC_HDL_IDX] = attr->handle;

			memcpy(&discover_uuid_128, BT_UUID_ANCS_CONTROL_POINT,
			       sizeof(discover_uuid_128));
			discover_params.uuid = &discover_uuid_128.uuid;
			// discover_params.start_handle = attr->handle + 1;
			discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
			discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				printk("Discover failed (err %d)\n", err);
				discovery_state = DISCOVERY_ANCS_FAIL;
			}
			discovery_state = DISCOVERY_CONTROL_POINT_CHRC;
		}
		break;

	case DISCOVERY_CONTROL_POINT_CHRC:
		if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_ANCS_CONTROL_POINT)) {
			ancsc.handle[BT_ANCS_CONTROL_POINT_HDL_IDX] =
				bt_gatt_attr_value_handle(attr);

			memcpy(&discover_uuid_128, BT_UUID_ANCS_DATA_SOURCE,
			       sizeof(discover_uuid_128));
			discover_params.uuid = &discover_uuid_128.uuid;
			discover_params.start_handle = attr->handle + 1;
			discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				printk("Discover failed (err %d)\n", err);
				discovery_state = DISCOVERY_ANCS_FAIL;
			}
			discovery_state = DISCOVERY_DATA_SOURCE_CHRC;
		} else {
			discovery_state = DISCOVERY_ANCS_FAIL;
		}
		break;

	case DISCOVERY_DATA_SOURCE_CHRC:
		if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_ANCS_DATA_SOURCE)) {
			ancsc.handle[BT_ANCS_DATA_SOURCE_HDL_IDX] = bt_gatt_attr_value_handle(attr);

			/* Discover the CCCD of data source characteristic */
			memcpy(&discover_uuid_16, BT_UUID_GATT_CCC, sizeof(discover_uuid_16));
			discover_params.uuid = &discover_uuid_16.uuid;
			discover_params.start_handle = attr->handle + 2;
			discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;

			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				printk("Discover failed (err %d)\n", err);
				discovery_state = DISCOVERY_ANCS_FAIL;
			}
		} else {
			ancsc.handle[BT_ANCS_DATA_SOURCE_CCC_HDL_IDX] = attr->handle;
			discovery_state = DISCOVERY_ANCS_FINISH;
			err = bt_ancs_subscribe(&ancsc);
			if (err) {
				printk("Subscribe failed (err %d)\n", err);
			} else {
				printk("[SUBSCRIBED]\n");
			}
		}
		break;
	default:
		break;
	}

	return BT_GATT_ITER_STOP;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	int sec_err;
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Connected %s\n", addr);

	ancsc.conn = conn;
	sec_err = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (sec_err) {
		printk("Failed to set security (err %d)\n", sec_err);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Disconnected from %s (reason 0x%02x)\n", addr, reason);

	ancsc.conn = NULL;
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Security changed: %s level %u\n", addr, level);

		if (bt_conn_get_security(conn) >= BT_SECURITY_L2) {
			/* Start to discover the ANCS service */
			memcpy(&discover_uuid_128, BT_UUID_ANCS, sizeof(discover_uuid_128));
			discover_params.uuid = &discover_uuid_128.uuid;
			discover_params.func = discover_func;
			discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
			discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
			discover_params.type = BT_GATT_DISCOVER_PRIMARY;

			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				printk("Discover failed(err %d)\n", err);
				return;
			}
			discovery_state = DISCOVERY_ANCS_SERVICE;
		}
	} else {
		printk("Security failed: %s level %u err %d\n", addr, level, err);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);

	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing failed conn: %s, reason %d\n", addr, reason);

	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {.pairing_complete = pairing_complete,
							       .pairing_failed = pairing_failed};

int main(void)
{
	int err;

	printk("Starting Apple Notification Center Service client example\n");

	err = bt_ancsc_init(&ancsc);
	if (err) {
		printk("ANCS client init failed (err %d)\n", err);
		return 0;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("BLE init failed (err %d)\n", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		printk("Failed to register authorization callbacks\n");
		return 0;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		printk("Failed to register authorization info callbacks.\n");
		return 0;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

	printk("Advertising successfully started\n");

	return 0;
}
