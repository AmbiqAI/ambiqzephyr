/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_THROUGHPUT_H_
#define ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_THROUGHPUT_H_

/**
 * @brief Throughput Service (THROUGHPUT)
 * @defgroup bt_throughput Throughput Service (THROUGHPUT)
 * @ingroup bluetooth
 * @{
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	THROUGHPUT_TRANS_MODE_NONE,
	THROUGHPUT_TRANS_MODE_TX,
	THROUGHPUT_TRANS_MODE_RX,
	THROUGHPUT_TRANS_MODE_TRX
} throughput_trans_mode_t;

/** @brief Throughput metrics. */
struct bt_throughput_data {

	/** Index of current TX packet. */
	uint16_t tx_index;

	/** Index of current RX packet. */
	uint16_t rx_index;

	/** Index of last previous RX packet. */
	uint16_t rx_last_index;

	/** Current MTU. */
	uint16_t mtu;

	/** Maximum transmission packet number. */
	uint16_t max_trans_packet;

	/** Transmission mode. */
	uint8_t trans_mode;

	/** The payload of TX packet. */
	uint8_t tx_payload[CONFIG_BT_L2CAP_TX_MTU];
};

/** @brief Throughput structure. */
struct bt_throughput {

	/** Throughput test data. */
	struct bt_throughput_data data;

	/** Connection object. */
	struct bt_conn *conn;
};

#define BT_UUID_THROUGHPUT_SVC_VAL                                                                 \
	BT_UUID_128_ENCODE(0x00002760, 0x08C2, 0x11E1, 0x9073, 0x0E8AC72E5450)

#define BT_UUID_THROUGHPUT_RX_CHAR_VAL                                                             \
	BT_UUID_128_ENCODE(0x00002760, 0x08C2, 0x11E1, 0x9073, 0x0E8AC72E5401)

#define BT_UUID_THROUGHPUT_TX_CHAR_VAL                                                             \
	BT_UUID_128_ENCODE(0x00002760, 0x08C2, 0x11E1, 0x9073, 0x0E8AC72E5402)

/** @brief Throughput Service UUID. */
#define BT_UUID_THROUGHPUT BT_UUID_DECLARE_128(BT_UUID_THROUGHPUT_SVC_VAL)

/** @brief Throughput RX Characteristic UUID. */
#define BT_UUID_THROUGHPUT_RX_CHAR BT_UUID_DECLARE_128(BT_UUID_THROUGHPUT_RX_CHAR_VAL)

/** @brief Throughput TX Characteristic UUID. */
#define BT_UUID_THROUGHPUT_TX_CHAR BT_UUID_DECLARE_128(BT_UUID_THROUGHPUT_TX_CHAR_VAL)

/** @brief Send throughput data.
 *
 * The data will be sent via a GATT notification to the subscriber.
 *
 *  @return Zero in case of success and error code in case of error.
 */
int bt_throughput_notify(void);

/** @brief Initialize the throughput data after connection.
 *
 *  @return Zero in case of success and error code in case of error.
 */
int bt_throughput_conn_init(struct bt_conn *conn);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_THROUGHPUT_H_ */
