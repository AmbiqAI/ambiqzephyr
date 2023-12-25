/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief A sample to demostrate the BLE controller firmware/patch update
 * in Ambiq Apollo4x EVB with Bluetooth Low Engery supported.
 */

#include <zephyr/drivers/bluetooth/hci_driver.h>

#include "am_devices_cooper.h"

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
	printk("BLE Firmware Update Application\n");
	ble_get_binary(&g_sUpdateImage);

	/* Calling bt_enable instead of bt_enable_raw to use the designed HCI command
	 * synchronization semaphore and response status check in host stack, since the setting
	 * signature is the most important command in the upgrading process and we need to make
	 * sure it is sent to the BLE controller successfully.
	 */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return err;
	}

	/* Set the corresponding updating signature to BLE controller. */
	err = set_update_sign(BLE_UPDATE_SIGN);
	if (err) {
		printk("Set signature failed (err %d)\n", err);
		return err;
	}

	printk("Reset BLE controller to do a forcing upgrade\n");
	err = am_devices_cooper_reset_with_sbl_check();
	if (err) {
		printk("BLE Firmware Update Failed (err %d)\n", err);
		return err;
	} else {
		printk("BLE Firmware Update Application Done!\n");
	}

	return 0;
}
