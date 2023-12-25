/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief A sample to demostrate the BLE controller firmware/patch update
 * in Ambiq Apollo4x EVB with Bluetooth Low Engery supported.
 */

#include <zephyr/logging/log.h>
#include <zephyr/drivers/bluetooth/hci_driver.h>

#include "am_devices_cooper.h"

#define LOG_MODULE_NAME ble_firmware_update
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* Uncomment one of UPDATE_BLE_FW, UPDATE_BLE_INFO0 and UPDATE_BLE_INFO1
 * macros to enable the updating of the BLE controller firmware, controller
 * info0 or info1 patch.
 */
#define UPDATE_BLE_FW
/* #define UPDATE_BLE_INFO0 */
/* #define UPDATE_BLE_INFO1 */
#if defined(UPDATE_BLE_FW)
#include "ble_fw_image.h"
#define image_bin       ble_fw_image_bin
#define image_type      AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW
#define ble_get_binary  am_devices_cooper_get_FwImage
#define BLE_UPDATE_SIGN COOPER_FW_UPDATE_SIGN
#elif defined(UPDATE_BLE_INFO0)
#include "info0_patch.h"
#define image_bin       info0_patch_bin
#define image_type      AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_0
#define ble_get_binary  am_devices_cooper_get_info0_patch
#define BLE_UPDATE_SIGN COOPER_INFO0_UPDATE_SIGN
#elif defined(UPDATE_BLE_INFO1)
#include "info1_patch.h"
#define image_bin       info1_patch_bin
#define image_type      AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_1
#define ble_get_binary  am_devices_cooper_get_info1_patch
#define BLE_UPDATE_SIGN COOPER_INFO1_UPDATE_SIGN
#else
#warning No update image available
#endif

static am_devices_cooper_sbl_update_data_t g_sUpdateImage = {(uint8_t *)&image_bin,
							     sizeof(image_bin), image_type, 0};

static int set_update_sign(uint32_t ui32Sign)
{
	struct net_buf *buf;
	uint8_t *p;
	uint8_t sign_cmd[4] = {UINT32_TO_BYTE0(ui32Sign), UINT32_TO_BYTE1(ui32Sign),
			       UINT32_TO_BYTE2(ui32Sign), UINT32_TO_BYTE3(ui32Sign)};

	buf = bt_hci_cmd_create(HCI_VSC_UPDATE_FW_CFG_CMD_OPCODE, HCI_VSC_UPDATE_FW_CFG_CMD_LENGTH);
	if (!buf) {
		return -ENOBUFS;
	}

	p = net_buf_add(buf, HCI_VSC_UPDATE_FW_CFG_CMD_LENGTH);
	memcpy(p, sign_cmd, HCI_VSC_UPDATE_FW_CFG_CMD_LENGTH);
	return bt_hci_cmd_send_sync(HCI_VSC_UPDATE_FW_CFG_CMD_OPCODE, buf, NULL);
}

int main()
{
	int err;
	LOG_INF("BLE Firmware Update Application");
	ble_get_binary(&g_sUpdateImage);
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return err;
	}

	err = set_update_sign(BLE_UPDATE_SIGN);
	if (err) {
		LOG_ERR("Set signaure failed (err %d)\n", err);
		return err;
	}

	LOG_INF("Reset BLE controller to do a forcing upgrade");
	err = am_devices_cooper_reset_with_sbl_check();
	if (err) {
		LOG_ERR("BLE Firmware Update Failed (err %d)", err);
		return err;
	} else {
		LOG_INF("BLE Firmware Update Application Done!");
	}

	return 0;
}
