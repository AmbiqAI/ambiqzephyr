/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>

#include "am_mcu_apollo.h"

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL))
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
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

static void bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();

	battery_level--;

	if (!battery_level) {
		battery_level = 100U;
	}

	bt_bas_set_battery_level(battery_level);
}

static void hrs_notify(void)
{
	static uint8_t heartrate = 90U;

	/* Heartrate measurements simulation */
	heartrate++;
	if (heartrate == 160U) {
		heartrate = 90U;
	}

	bt_hrs_notify(heartrate);
}

static int lp_init(void)
{
	am_hal_pwrctrl_mcu_memory_config_t McuMemCfg =
	{
		.eCacheCfg    = AM_HAL_PWRCTRL_CACHE_ALL,
		.bRetainCache = true,
		.eDTCMCfg     = AM_HAL_PWRCTRL_DTCM_128K,
		.eRetainDTCM  = AM_HAL_PWRCTRL_DTCM_128K,
		.bEnableNVM0  = true,
		.bRetainNVM0  = false
	};

	am_hal_pwrctrl_sram_memcfg_t SRAMMemCfg =
	{
		.eSRAMCfg           = AM_HAL_PWRCTRL_SRAM_NONE,
		.eActiveWithMCU     = AM_HAL_PWRCTRL_SRAM_NONE,
		.eActiveWithGFX     = AM_HAL_PWRCTRL_SRAM_NONE,
		.eActiveWithDISP    = AM_HAL_PWRCTRL_SRAM_NONE,
		.eActiveWithDSP     = AM_HAL_PWRCTRL_SRAM_NONE,
		.eSRAMRetain        = AM_HAL_PWRCTRL_SRAM_NONE
	};

    am_hal_pwrctrl_dsp_memory_config_t    DSPMemCfg =
    {
        .bEnableICache      = false,
        .bRetainCache       = false,
        .bEnableRAM         = false,
        .bActiveRAM         = false,
        .bRetainRAM         = false
    };

	am_hal_sysctrl_fpu_disable();
    am_hal_pwrctrl_low_power_init();
    am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_SIMOBUCK_INIT, 0);
    am_hal_rtc_osc_disable();
	am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
	am_hal_pwrctrl_mcu_memory_config(&McuMemCfg);
	am_hal_pwrctrl_sram_config(&SRAMMemCfg);
	am_hal_pwrctrl_dsp_memory_config(AM_HAL_DSP0, &DSPMemCfg);
	
	return 0;
}

SYS_INIT(lp_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

int main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	bt_ready();

	bt_conn_auth_cb_register(&auth_cb_display);

	/* Implement notification. At the moment there is no suitable way
	 * of starting delayed work so we do it here
	 */
	while (1) {
		k_sleep(K_SECONDS(1));

		/* Heartrate measurements simulation */
		hrs_notify();

		/* Battery level simulation */
		bas_notify();

        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
	}
	return 0;
}
