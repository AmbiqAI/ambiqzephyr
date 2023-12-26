/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 */

#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/ancsc.h>

#define LOG_LEVEL CONFIG_BT_ANCSC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ancsc);

/* String print for the iOS notification categories. */
static const char *category_str[BT_ANCS_CATEGORY_ID_COUNT] = {
	"Other",    "Incoming Call", "Missed Call", "Voice Mail",         "Social",
	"Schedule", "Email",         "News",        "Health And Fitness", "Business And Finance",
	"Location", "Entertainment"};

/* String print for the iOS notification event flags. */
static const char *event_flag_str[BT_ANCS_EVENT_FLAG_COUNT] = {"Slient", "Important", "PreExisting",
							       "PositiveAction", "NegativeAction"};

/* String print for the iOS notification attribute types. */
static const char *notif_attr_str[BT_ANCS_NOTIF_ATTR_COUNT] = {"App Identifier",
							       "Title",
							       "Subtitle",
							       "Message",
							       "Message Size",
							       "Date",
							       "Positive Action Label",
							       "Negative Action Label"};

/* String print for the iOS notification event types. */
static const char *event_id_str[BT_ANCS_EVT_ID_COUNT] = {"Added", "Modified", "Removed"};

#if 0 /* To be implemented further */
/* String print for the iOS App attribute types. */
static const char *app_attr_str[BT_ANCS_APP_ATTR_COUNT] = {"Display Name"};
#endif

static void ancs_process_notif(struct bt_ancsc *ancsc, const uint8_t *data, const uint16_t len)
{
	struct bt_ancs_evt_notif notif = {0};

	LOG_INF("***Received Notification Data from iOS***");
	notif.evt_id = (enum bt_ancs_evt_id)data[BT_ANCS_NOTIF_EVT_ID_INDEX];
	LOG_INF("Event: %s", event_id_str[notif.evt_id]);

	notif.evt_flags = data[BT_ANCS_NOTIF_FLAGS_INDEX];

	LOG_INF("Events:");
	for (uint8_t i = 0; i < BT_ANCS_EVENT_FLAG_COUNT; i++) {
		if (notif.evt_flags & (0x1 << i)) {
			LOG_INF("    %s", event_flag_str[i]);
		}
	}

	notif.category_id = (enum bt_ancs_category_id)data[BT_ANCS_NOTIF_CATEGORY_ID_INDEX];
	LOG_INF("Category: %s", category_str[notif.category_id]);

	notif.category_count = data[BT_ANCS_NOTIF_CATEGORY_CNT_INDEX];
	LOG_INF("Category Count: %d", notif.category_count);

	notif.notif_uid = sys_get_le32(&data[BT_ANCS_NOTIF_NOTIF_UID]);
	LOG_INF("UID: 0x%08x", notif.notif_uid);

	bt_ancs_get_notif_attrs(ancsc, notif.notif_uid);
}

static uint8_t ancs_ns_notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
				 const void *data, uint16_t length)
{
	struct bt_ancsc *ancsc;

	/* Retrieve ANCS client module context. */
	ancsc = CONTAINER_OF(params, struct bt_ancsc, ns_notif_params);

	ancs_process_notif(ancsc, data, length);

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t ancs_ds_notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
				 const void *data, uint16_t length)
{
	int err;
	struct bt_ancsc *ancsc;

	/* Retrieve ANCS client module context. */
	ancsc = CONTAINER_OF(params, struct bt_ancsc, ds_notif_params);

	err = bt_ancs_process_ds_attrs(ancsc, data, length);
	if (err) {
		LOG_ERR("Process the Getting Attributes fails (err %d)", err);
	}

	return BT_GATT_ITER_CONTINUE;
}

int bt_ancs_subscribe(struct bt_ancsc *ancsc)
{
	int ret;

	if (!ancsc) {
		return -EINVAL;
	}

	if (atomic_test_and_set_bit(&ancsc->state, ANCS_NS_NOTIF_ENABLED)) {
		return -EALREADY;
	}

	ancsc->ns_notif_params.notify = ancs_ns_notify_cb;
	ancsc->ns_notif_params.value = BT_GATT_CCC_NOTIFY;
	ancsc->ns_notif_params.value_handle = ancsc->handle[BT_ANCS_NOTIFICATION_SOURCE_HDL_IDX];
	ancsc->ns_notif_params.ccc_handle = ancsc->handle[BT_ANCS_NOTIFICATION_SOURCE_CCC_HDL_IDX];
	atomic_set_bit(ancsc->ns_notif_params.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

	ret = bt_gatt_subscribe(ancsc->conn, &ancsc->ns_notif_params);
	if (ret) {
		atomic_clear_bit(&ancsc->state, ANCS_NS_NOTIF_ENABLED);
		LOG_ERR("Subscribe Notification Source failed (ret %d)", ret);
		return ret;
	} else {
		LOG_INF("Notification Source subscribed");
	}

	if (atomic_test_and_set_bit(&ancsc->state, ANCS_DS_NOTIF_ENABLED)) {
		return -EALREADY;
	}

	ancsc->ds_notif_params.notify = ancs_ds_notify_cb;
	ancsc->ds_notif_params.value = BT_GATT_CCC_NOTIFY;
	ancsc->ds_notif_params.value_handle = ancsc->handle[BT_ANCS_DATA_SOURCE_HDL_IDX];
	ancsc->ds_notif_params.ccc_handle = ancsc->handle[BT_ANCS_DATA_SOURCE_CCC_HDL_IDX];
	atomic_set_bit(ancsc->ds_notif_params.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

	ret = bt_gatt_subscribe(ancsc->conn, &ancsc->ds_notif_params);
	if (ret) {
		atomic_clear_bit(&ancsc->state, ANCS_DS_NOTIF_ENABLED);
		LOG_ERR("Subscribe Data Source failed (err %d)", ret);
	} else {
		LOG_DBG("Data Source subscribed");
	}

	return ret;
}

int bt_ancs_unsubscribe(struct bt_ancsc *ancsc)
{
	int ret;

	if (!ancsc) {
		return -EINVAL;
	}

	if (!atomic_test_bit(&ancsc->state, ANCS_NS_NOTIF_ENABLED)) {
		return -EFAULT;
	}

	ret = bt_gatt_unsubscribe(ancsc->conn, &ancsc->ns_notif_params);
	if (ret) {
		LOG_ERR("Unsubscribe Notification Source failed (ret %d)", ret);
		return ret;
	} else {
		atomic_clear_bit(&ancsc->state, ANCS_NS_NOTIF_ENABLED);
		LOG_INF("Notification Source unsubscribed");
	}

	if (!atomic_test_bit(&ancsc->state, ANCS_DS_NOTIF_ENABLED)) {
		return -EFAULT;
	}

	ret = bt_gatt_unsubscribe(ancsc->conn, &ancsc->ds_notif_params);
	if (ret) {
		LOG_ERR("Unsubscribe Data Source failed (err %d)", ret);
	} else {
		atomic_clear_bit(&ancsc->state, ANCS_DS_NOTIF_ENABLED);
		LOG_INF("Data Source unsubscribed");
	}

	return ret;
}

static uint16_t encode_notif_action(uint8_t *encoded_data, uint32_t uid,
				    enum bt_ancs_action_id_values action_id)
{
	uint8_t index = 0;

	encoded_data[index++] = BT_ANCS_COMMAND_ID_PERFORM_NOTIF_ACTION;
	sys_put_le32(uid, &encoded_data[index]);
	index += sizeof(uint32_t);
	encoded_data[index++] = (uint8_t)action_id;

	return index;
}

static void ancs_cp_write_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
	ARG_UNUSED(err);

	struct bt_ancsc *ancsc;
	bt_ancs_write_cb write_cb;

	/* Retrieve ANCS client module context. */
	ancsc = CONTAINER_OF(params, struct bt_ancsc, cp_write_params);

	write_cb = ancsc->cp_write_cb;
	atomic_clear_bit(&ancsc->state, ANCS_CP_WRITE_PENDING);
	if (write_cb) {
		write_cb(ancsc);
	}
}

int bt_ancs_cp_write(struct bt_ancsc *ancsc, uint16_t len)
{
	int err;
	struct bt_gatt_write_params *write_params = &ancsc->cp_write_params;

	write_params->func = ancs_cp_write_cb;
	write_params->handle = ancsc->handle[BT_ANCS_CONTROL_POINT_HDL_IDX];
	write_params->offset = 0;
	write_params->data = ancsc->cp_data;
	write_params->length = len;

	err = bt_gatt_write(ancsc->conn, write_params);
	if (err) {
		atomic_clear_bit(&ancsc->state, ANCS_CP_WRITE_PENDING);
	}

	return err;
}

int bt_ancs_notification_action(struct bt_ancsc *ancsc, uint32_t uuid,
				enum bt_ancs_action_id_values action_id, bt_ancs_write_cb func)
{
	if (atomic_test_and_set_bit(&ancsc->state, ANCS_CP_WRITE_PENDING)) {
		return -EBUSY;
	}

	uint8_t *data = ancsc->cp_data;
	uint16_t len = encode_notif_action(data, uuid, action_id);

	return bt_ancs_cp_write(ancsc, len);
}

int bt_ancs_get_notif_attrs(struct bt_ancsc *ancsc, const uint32_t uid)
{
	if (atomic_test_and_set_bit(&ancsc->state, ANCS_CP_WRITE_PENDING)) {
		return -EBUSY;
	}

	uint16_t length = 0;
	uint8_t *data = ancsc->cp_data;

	/* Encode Command ID. */
	*(data + length++) = BT_ANCS_COMMAND_ID_GET_NOTIF_ATTRIBUTES;

	/* Encode Notification UID. */
	sys_put_le32(uid, data + length);
	length += sizeof(uint32_t);

	/* Encode APP identifier. */
	*(data + length++) = BT_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER;

	/* Encode Title and length. */
	*(data + length++) = BT_ANCS_NOTIF_ATTR_ID_TITLE;
	sys_put_le16(BT_ANCS_ATTR_DATA_MAX, data + length);
	length += sizeof(uint16_t);

	/* Encode Subtitle and length. */
	*(data + length++) = BT_ANCS_NOTIF_ATTR_ID_SUBTITLE;
	sys_put_le16(BT_ANCS_ATTR_DATA_MAX, data + length);
	length += sizeof(uint16_t);

	/* Encode Message and length. */
	*(data + length++) = BT_ANCS_NOTIF_ATTR_ID_MESSAGE;
	sys_put_le16(BT_ANCS_ATTR_DATA_MAX, data + length);
	length += sizeof(uint16_t);

	/* Encode Message size. */
	*(data + length++) = BT_ANCS_NOTIF_ATTR_ID_MESSAGE_SIZE;

	/* Encode Date. */
	*(data + length++) = BT_ANCS_NOTIF_ATTR_ID_DATE;

	/* Encode Positive action label. */
	*(data + length++) = BT_ANCS_NOTIF_ATTR_ID_POSITIVE_ACTION_LABEL;

	/* Encode Negative action label. */
	*(data + length++) = BT_ANCS_NOTIF_ATTR_ID_NEGATIVE_ACTION_LABEL;

	int ret = bt_ancs_cp_write(ancsc, length);

	return ret;
}

int bt_ancs_get_app_attrs(struct bt_ancsc *ancsc, const uint8_t *app_id, uint16_t len)
{
	if (atomic_test_and_set_bit(&ancsc->state, ANCS_CP_WRITE_PENDING)) {
		return -EBUSY;
	}

	/* TODO */
	return 0;
}

static void ancs_process_notif_attrs(struct bt_ancsc *ancsc, const uint8_t *data,
				     const uint16_t len)
{
	uint16_t consume_bytes = 0;
	uint16_t proc_offset = 0;
	uint16_t packet_bytes = len;
	uint16_t accept_attr_len = 0;
	char notif_print[BT_ANCS_ATTR_DATA_MAX + 1];
	static uint8_t attr_index = BT_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER;
	static uint32_t notif_uid;
	static uint8_t state = BT_ANCS_ATTRS_PROC_STATE_NOTIF_UID;

	if (state == BT_ANCS_ATTRS_PROC_STATE_FINISH) {
		state = BT_ANCS_ATTRS_PROC_STATE_NOTIF_UID;
	}

	/* We may receive the attributes in multiple GATT packets. */
	while (packet_bytes) {
		switch (state) {
		case BT_ANCS_ATTRS_PROC_STATE_NOTIF_UID:
			notif_uid = sys_get_le32(&data[proc_offset]);
			consume_bytes = sizeof(uint32_t);
			state = BT_ANCS_ATTRS_PROC_STATE_ATTR_ID;
			break;

		case BT_ANCS_ATTRS_PROC_STATE_ATTR_ID:
			ancsc->ancs_notif_attr_list[attr_index].attr_id = data[proc_offset];
			consume_bytes = sizeof(uint8_t);
			state = BT_ANCS_ATTRS_PROC_STATE_ATTR_LEN1;
			break;

		case BT_ANCS_ATTRS_PROC_STATE_ATTR_LEN1:
			ancsc->ancs_notif_attr_list[attr_index].attr_len = data[proc_offset];
			consume_bytes = sizeof(uint8_t);
			state = BT_ANCS_ATTRS_PROC_STATE_ATTR_LEN2;
			break;

		case BT_ANCS_ATTRS_PROC_STATE_ATTR_LEN2:
			ancsc->ancs_notif_attr_list[attr_index].attr_len |=
				(data[proc_offset] << 8);
			consume_bytes = sizeof(uint8_t);
			if (ancsc->ancs_notif_attr_list[attr_index].attr_len > 0) {
				state = BT_ANCS_ATTRS_PROC_STATE_ATTR_DATA;
			} else {
				attr_index++;
				if (attr_index == BT_ANCS_NOTIF_ATTR_COUNT) {
					attr_index = BT_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER;
					state = BT_ANCS_ATTRS_PROC_STATE_FINISH;
				} else {
					/* Continue to process the attributes */
					state = BT_ANCS_ATTRS_PROC_STATE_ATTR_ID;
				}
			}
			break;

		case BT_ANCS_ATTRS_PROC_STATE_ATTR_DATA:
			if (packet_bytes >= ancsc->ancs_notif_attr_list[attr_index].attr_len) {
				if (ancsc->ancs_notif_attr_list[attr_index].attr_len >
				    sizeof(ancsc->ancs_notif_attr_list[attr_index].attr_data)) {
					accept_attr_len = sizeof(
						ancsc->ancs_notif_attr_list[attr_index].attr_data);
				} else {
					accept_attr_len =
						ancsc->ancs_notif_attr_list[attr_index].attr_len;
				}
				memcpy(ancsc->ancs_notif_attr_list[attr_index].attr_data,
				       &data[proc_offset], accept_attr_len);
				consume_bytes = ancsc->ancs_notif_attr_list[attr_index].attr_len;
			} else {
				/* TODO: may need to consider if the attribute data comes from
				 * different packets.
				 */
			}

			memset(notif_print, 0, sizeof(notif_print));
			strncpy(notif_print, ancsc->ancs_notif_attr_list[attr_index].attr_data,
				accept_attr_len);
			LOG_INF("[%s]: %s", notif_attr_str[attr_index], notif_print);
			attr_index++;
			if (attr_index == BT_ANCS_NOTIF_ATTR_COUNT) {
				attr_index = BT_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER;
				state = BT_ANCS_ATTRS_PROC_STATE_FINISH;
			} else {
				/* Continue to process the attributes */
				state = BT_ANCS_ATTRS_PROC_STATE_ATTR_ID;
			}
			break;

		default:
			break;
		}

		proc_offset += consume_bytes;
		packet_bytes -= consume_bytes;
	}
}

int bt_ancs_process_ds_attrs(struct bt_ancsc *ancsc, const uint8_t *data, const uint16_t len)
{
	uint8_t command_id = data[0];

	switch (command_id) {
	case BT_ANCS_COMMAND_ID_GET_NOTIF_ATTRIBUTES:
		ancs_process_notif_attrs(ancsc, &data[1], (len - 1));
		break;

	case BT_ANCS_COMMAND_ID_GET_APP_ATTRIBUTES:
	case BT_ANCS_COMMAND_ID_PERFORM_NOTIF_ACTION:
	default:
		break;
	}

	return 0;
}

int bt_ancsc_init(struct bt_ancsc *ancsc)
{
	if (!ancsc) {
		return -EINVAL;
	}

	memset(ancsc, 0, sizeof(struct bt_ancsc));

	return 0;
}
