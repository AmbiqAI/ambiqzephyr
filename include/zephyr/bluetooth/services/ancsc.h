/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 */

#ifndef BT_ANCSC_H_
#define BT_ANCSC_H_

/**
 * @brief Apple Notification Center Service Client (ANCSC)
 * @defgroup bt_ancsc Apple Notification Center Service Client (ANCSC)
 * @ingroup bluetooth
 * @{
 */

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Apple Notification Center Service UUID. */
#define BT_UUID_ANCS_VAL BT_UUID_128_ENCODE(0x7905f431, 0xb5ce, 0x4e99, 0xa40f, 0x4b1e122d00d0)

/** @brief Notification Source Characteristic UUID. */
#define BT_UUID_ANCS_NOTIFICATION_SOURCE_VAL                                                       \
	BT_UUID_128_ENCODE(0x9fbf120d, 0x6301, 0x42d9, 0x8c58, 0x25e699a21dbd)

/** @brief Control Point Characteristic UUID. */
#define BT_UUID_ANCS_CONTROL_POINT_VAL                                                             \
	BT_UUID_128_ENCODE(0x69d1d8f3, 0x45e1, 0x49a8, 0x9821, 0x9bbdfdaad9d9)

/** @brief Data Source Characteristic UUID. */
#define BT_UUID_ANCS_DATA_SOURCE_VAL                                                               \
	BT_UUID_128_ENCODE(0x22eac6e9, 0x24d6, 0x4bb5, 0xbe44, 0xb36ace7c7bfb)

#define BT_UUID_ANCS                     BT_UUID_DECLARE_128(BT_UUID_ANCS_VAL)
#define BT_UUID_ANCS_NOTIFICATION_SOURCE BT_UUID_DECLARE_128(BT_UUID_ANCS_NOTIFICATION_SOURCE_VAL)
#define BT_UUID_ANCS_CONTROL_POINT       BT_UUID_DECLARE_128(BT_UUID_ANCS_CONTROL_POINT_VAL)
#define BT_UUID_ANCS_DATA_SOURCE         BT_UUID_DECLARE_128(BT_UUID_ANCS_DATA_SOURCE_VAL)

/** Maximum data length of an iOS notification attribute. */
#define BT_ANCS_ATTR_DATA_MAX 32

/** @brief Length of the iOS notification data. */
#define BT_ANCS_NOTIF_DATA_LENGTH 8

/** @brief Size of the control point buffer. */
#define BT_ANCS_CP_BUF_SIZE 256

/** @brief Atomic opertionn bit. */
enum {
	ANCS_NS_NOTIF_ENABLED,
	ANCS_DS_NOTIF_ENABLED,
	ANCS_CP_WRITE_PENDING
};

/**@brief Event flag bitmask for iOS event flag. All flags can be active at the
 * same time.
 */
enum bt_ancs_evt_flag_bitmask {
	/** Silent: First (LSB) bit is set. */
	BT_ANCS_EVENT_FLAG_SILENT,
	/** Important: Second (LSB) bit is set. */
	BT_ANCS_EVENT_FLAG_IMPORTANT,
	/** Pre-existing: Third (LSB) bit is set. */
	BT_ANCS_EVENT_FLAG_PREEXISTING,
	/** Positive action: Fourth (LSB) bit is set. */
	BT_ANCS_EVENT_FLAG_POSITIVE_ACTION,
	/** Negative action: Fifth (LSB) bit is set. */
	BT_ANCS_EVENT_FLAG_NEGATIVE_ACTION,
	/** Number of event flags. */
	BT_ANCS_EVENT_FLAG_COUNT
};

/**@brief ANCS-specific error codes from Notification Provider. */
enum bt_ancs_np_error {
	/** The command ID was not recognized by the NP. */
	BT_ANCS_NP_ERR_UNKNOWN_CMD = 0xA0,
	/** The command was improperly formatted. */
	BT_ANCS_NP_ERR_INVALID_CMD,
	/** One of the parameters does not refer to an existing object on the NP. */
	BT_ANCS_NP_ERR_INVALID_PARA,
	/** The action was not performed.*/
	BT_ANCS_NP_ERR_ACTION_FAILED
};

/**@brief Category IDs for iOS notifications. */
enum bt_ancs_category_id {
	/** The iOS notification belongs to the "Other" category.  */
	BT_ANCS_CATEGORY_ID_OTHER,
	/** The iOS notification belongs to the "Incoming Call" category. */
	BT_ANCS_CATEGORY_ID_INCOMING_CALL,
	/** The iOS notification belongs to the "Missed Call" category. */
	BT_ANCS_CATEGORY_ID_MISSED_CALL,
	/** The iOS notification belongs to the "Voice Mail" category. */
	BT_ANCS_CATEGORY_ID_VOICE_MAIL,
	/** The iOS notification belongs to the "Social" category. */
	BT_ANCS_CATEGORY_ID_SOCIAL,
	/** The iOS notification belongs to the "Schedule" category. */
	BT_ANCS_CATEGORY_ID_SCHEDULE,
	/** The iOS notification belongs to the "Email" category. */
	BT_ANCS_CATEGORY_ID_EMAIL,
	/** The iOS notification belongs to the "News" category. */
	BT_ANCS_CATEGORY_ID_NEWS,
	/** The iOS notification belongs to the "Health and Fitness" category. */
	BT_ANCS_CATEGORY_ID_HEALTH_AND_FITNESS,
	/** The iOS notification belongs to the "Business and Finance" category. */
	BT_ANCS_CATEGORY_ID_BUSINESS_AND_FINANCE,
	/** The iOS notification belongs to the "Location" category. */
	BT_ANCS_CATEGORY_ID_LOCATION,
	/** The iOS notification belongs to the "Entertainment" category. */
	BT_ANCS_CATEGORY_ID_ENTERTAINMENT,
	/** Number of the iOS notification categories. */
	BT_ANCS_CATEGORY_ID_COUNT
};

/**@brief Event IDs for iOS notifications. */
enum bt_ancs_evt_id {
	/** The iOS notification was added. */
	BT_ANCS_EVENT_ID_NOTIFICATION_ADDED,
	/** The iOS notification was modified. */
	BT_ANCS_EVENT_ID_NOTIFICATION_MODIFIED,
	/** The iOS notification was removed. */
	BT_ANCS_EVENT_ID_NOTIFICATION_REMOVED,
	/** Number of iOS notification events. */
	BT_ANCS_EVT_ID_COUNT
};

/**@brief Control point command IDs that the Notification Consumer can send
 *        to the Notification Provider.
 */
enum bt_ancsc_cmd_id {
	/** Requests attributes to be sent from the NP to the NC for a given
	 *  notification.
	 */
	BT_ANCS_COMMAND_ID_GET_NOTIF_ATTRIBUTES,
	/** Requests attributes to be sent from the NP to the NC for a given
	 *  iOS app.
	 */
	BT_ANCS_COMMAND_ID_GET_APP_ATTRIBUTES,
	/** Requests an action to be performed on a given notification.
	 *  For example, dismiss an alarm.
	 */
	BT_ANCS_COMMAND_ID_PERFORM_NOTIF_ACTION
};

/**@brief IDs for actions that can be performed for iOS notifications. */
enum bt_ancs_action_id_values {
	/** Positive action. */
	BT_ANCS_ACTION_ID_POSITIVE,
	/** Negative action. */
	BT_ANCS_ACTION_ID_NEGATIVE
};

/**@brief App attribute ID values.
 * @details Currently, only one value is defined. However, the number of app
 * attributes might increase. For this reason, they are stored in an enumeration.
 */
enum bt_ancs_app_attr_id_val {
	/** Command used to get the display name for an app identifier. */
	BT_ANCS_APP_ATTR_ID_DISPLAY_NAME,
	/** Number of iOS application attributes. */
	BT_ANCS_APP_ATTR_COUNT
};

/**@brief IDs for iOS notification attributes. */
enum bt_ancs_notif_attr_id_val {
	/** Identifies that the attribute data is of an "App Identifier" type. */
	BT_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER,
	/** Identifies that the attribute data is a "Title". */
	BT_ANCS_NOTIF_ATTR_ID_TITLE,
	/** Identifies that the attribute data is a "Subtitle". */
	BT_ANCS_NOTIF_ATTR_ID_SUBTITLE,
	/** Identifies that the attribute data is a "Message". */
	BT_ANCS_NOTIF_ATTR_ID_MESSAGE,
	/** Identifies that the attribute data is a "Message Size". */
	BT_ANCS_NOTIF_ATTR_ID_MESSAGE_SIZE,
	/** Identifies that the attribute data is a "Date". */
	BT_ANCS_NOTIF_ATTR_ID_DATE,
	/** The notification has a "Positive action" that can be executed associated with it. */
	BT_ANCS_NOTIF_ATTR_ID_POSITIVE_ACTION_LABEL,
	/** The notification has a "Negative action" that can be executed associated with it. */
	BT_ANCS_NOTIF_ATTR_ID_NEGATIVE_ACTION_LABEL,
	/** Number of iOS notification attributes. */
	BT_ANCS_NOTIF_ATTR_COUNT
};

/**@brief Handle index of ANCS characteristics. */
enum bt_ancs_handle {
	/** Notification Source Characteristic. */
	BT_ANCS_NOTIFICATION_SOURCE_HDL_IDX,
	/** Notification Source CCCD. */
	BT_ANCS_NOTIFICATION_SOURCE_CCC_HDL_IDX,
	/** Control Point Characteristic. */
	BT_ANCS_CONTROL_POINT_HDL_IDX,
	/** Data Source Characteristic. */
	BT_ANCS_DATA_SOURCE_HDL_IDX,
	/** Data Source CCCD. */
	BT_ANCS_DATA_SOURCE_CCC_HDL_IDX,
	/* Length of Hanle List. */
	BT_ANCS_HDL_LIST_LEN
};

/**@brief Index of each part in notification data */
enum bt_ancs_notif_data_index {
	/** Index of the Event ID */
	BT_ANCS_NOTIF_EVT_ID_INDEX,
	/** Index of the Flags field */
	BT_ANCS_NOTIF_FLAGS_INDEX,
	/** Index of the Category ID field */
	BT_ANCS_NOTIF_CATEGORY_ID_INDEX,
	/** Index of the Category Count field */
	BT_ANCS_NOTIF_CATEGORY_CNT_INDEX,
	/** Index of the Notification UID field */
	BT_ANCS_NOTIF_NOTIF_UID
};

/**@brief Processing states for received attributes. */
enum bt_ancs_attrs_proc_state {
	/** Noitification UID. */
	BT_ANCS_ATTRS_PROC_STATE_NOTIF_UID,
	/** App Identifier. */
	BT_ANCS_ATTRS_PROC_STATE_APP_ID,
	/** Attribute ID. */
	BT_ANCS_ATTRS_PROC_STATE_ATTR_ID,
	/** LSB of the Attribute Length. */
	BT_ANCS_ATTRS_PROC_STATE_ATTR_LEN1,
	/** MSB of the Attribute length. */
	BT_ANCS_ATTRS_PROC_STATE_ATTR_LEN2,
	/** Attribute Data. */
	BT_ANCS_ATTRS_PROC_STATE_ATTR_DATA,
	/* Finish the processing. */
	BT_ANCS_ATTRS_PROC_STATE_FINISH
};

/**@brief iOS notification structure. */
struct bt_ancs_evt_notif {
	/** Notification UID. */
	uint32_t notif_uid;
	/** Whether the notification was added, removed, or modified. */
	enum bt_ancs_evt_id evt_id;
	/** Bitmask to signal whether a special condition applies to the
	 *  notification. For example, "Silent" or "Important".
	 */
	uint8_t evt_flags;
	/** Classification of the notification type. For example, email or
	 *  location.
	 */
	enum bt_ancs_category_id category_id;
	/** Current number of active notifications for this category ID. */
	uint8_t category_count;
};

/**@brief iOS attribute structure. This type is used for both notification
 *        attributes and app attributes.
 */
struct bt_ancs_attr {
	/** Length of the received attribute data. */
	uint16_t attr_len;
	/** Classification of the attribute type. For example, "Title" or "Date". */
	uint32_t attr_id;
	/** Pointer to where the memory is allocated for storing incoming attributes. */
	uint8_t *attr_data;
};

/**@brief iOS notification attribute content requested by the application. */
struct bt_ancs_attr_list {
	/** Attribute ID (see enum bt_ancs_notif_attr_id).
	 */
	uint8_t attr_id;
	/** Length of the attribute. If more data is received from the
	 *  Notification Provider, all the data beyond this length is discarded.
	 */
	uint16_t attr_len;
	/** Pointer to where the memory is allocated for storing incoming attributes. */
	uint8_t attr_data[BT_ANCS_ATTR_DATA_MAX];
};

struct bt_ancsc;

/**@brief Attribute response structure. */
struct bt_ancs_attr_response {
	/** Command ID. */
	enum bt_ancsc_cmd_id command_id;
	/** iOS notification attribute or app attribute, depending on the
	 *  Command ID.
	 */
	struct bt_ancs_attr attr;
	/** Notification UID. */
	uint32_t notif_uid;
	/** App identifier. */
	uint8_t app_id[BT_ANCS_ATTR_DATA_MAX];
};

/**@brief Write response callback function.
 *
 * @param[in] ancsc   ANCS client instance.
 * @param[in] err     ATT error code.
 */
typedef void (*bt_ancs_write_cb)(struct bt_ancsc *ancsc);

/**@brief ANCS client instance, which contains various status information. */
struct bt_ancsc {
	/** Connection object. */
	struct bt_conn *conn;

	/** Internal state. */
	atomic_t state;

	/** Handle List. */
	uint16_t handle[BT_ANCS_HDL_LIST_LEN];

	/** GATT write parameters for Control Point Characteristic. */
	struct bt_gatt_write_params cp_write_params;

	/** Callback function for Control Point GATT write. */
	bt_ancs_write_cb cp_write_cb;

	/** Data buffer for Control Point GATT write. */
	uint8_t cp_data[BT_ANCS_CP_BUF_SIZE];

	/** GATT subscribe parameters for Notification Source Characteristic. */
	struct bt_gatt_subscribe_params ns_notif_params;

	/** GATT subscribe parameters for Data Source Characteristic. */
	struct bt_gatt_subscribe_params ds_notif_params;

	/** For all attributes: contains information about whether the
	 *  attributes are to be requested upon attribute request, and
	 *  the length and buffer of where to store attribute data.
	 */
	struct bt_ancs_attr_list ancs_notif_attr_list[BT_ANCS_NOTIF_ATTR_COUNT];

	/** For all app attributes: contains information about whether the
	 *  attributes are to be requested upon attribute request, and the
	 *  length and buffer of where to store attribute data.
	 */
	struct bt_ancs_attr_list ancs_app_attr_list[BT_ANCS_APP_ATTR_COUNT];

	/** Allocate memory for the attribute response here.
	 */
	struct bt_ancs_attr_response attr_response;
};

/**@brief Function for initializing the ANCS client.
 *
 * @param[out] ancsc   ANCS client instance.
 *
 * @retval 0 If the client was initialized successfully.
 *           Otherwise, a (negative) error code is returned.
 */
int bt_ancsc_init(struct bt_ancsc *ancsc);

/**@brief Function for writing to the CCCD to enable notifications.
 *
 * @param[in] ancsc   ANCS client instance.
 *
 * @retval 0 If writing to the CCCD was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int bt_ancs_subscribe(struct bt_ancsc *ancsc);

/**@brief Function for writing to the CCCD to disable notifications from the ANCS.
 *
 * @param[in] ancsc   ANCS client instance.
 *
 * @retval 0 If writing to the CCCD was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int bt_ancs_unsubscribe(struct bt_ancsc *ancsc);

/**@brief Function for performing a notification action.
 *
 * @param[in] ancsc     ANCS client instance.
 * @param[in] uuid      The UUID of the notification for which to perform the action.
 * @param[in] action_id Perform a positive or negative action.
 * @param[in] func      Callback function for handling NP write response.
 *
 * @retval 0 If the operation is successful.
 *           Otherwise, a (negative) error code is returned.
 */
int bt_ancs_notification_action(struct bt_ancsc *ancsc, uint32_t uuid,
				enum bt_ancs_action_id_values action_id, bt_ancs_write_cb func);

/**@brief Function for writing to ANCS Control Point.
 *
 * The caller is expected to set the ANCS_CP_WRITE_PENDING and to set up
 * the Control Point data buffer before calling this function.
 *
 * @param[in] ancs_c  ANCS client instance.
 * @param[in] len     Length of data in Control Point data buffer.
 * @param[in] func    Callback function for handling NP write response.
 *
 * @retval 0 If the operation is successful.
 *           Otherwise, a (negative) error code is returned.
 */
int bt_ancs_write_cp(struct bt_ancsc *ancsc, uint16_t len, bt_ancs_write_cb func);

/**@brief Function for getting App attributes.
 *
 * @param[in] ancs_c  ANCS client instance.
 * @param[in] app_id  Data pointer of App identifier.
 * @param[in] len     Length of App identifier.
 *
 * @retval 0 If the operation is successful.
 *           Otherwise, a (negative) error code is returned.
 */
int bt_ancs_get_app_attrs(struct bt_ancsc *ancsc, const uint8_t *app_id, uint16_t len);

/**@brief Function for getting notification attributes.
 *
 * @param[in] ancs_c  ANCS client instance.
 * @param[in] uid     Notification UID.
 *
 * @retval 0 If the operation is successful.
 *           Otherwise, a (negative) error code is returned.
 */
int bt_ancs_get_notif_attrs(struct bt_ancsc *ancsc, const uint32_t uid);

/**@brief Function for processing the Data Source attributes.
 *
 * @param[in] ancs_c  ANCS client instance.
 * @param[in] data    Pointer of data to be processed.
 * @param[in] len     Length of data to be processed.
 *
 * @retval 0 If the operation is successful.
 *           Otherwise, a (negative) error code is returned.
 */
int bt_ancs_process_ds_attrs(struct bt_ancsc *ancsc, const uint8_t *data, const uint16_t len);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* BT_ANCSC_H_ */
