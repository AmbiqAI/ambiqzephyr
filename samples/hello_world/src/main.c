/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/sys/printk.h>

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

int main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD_TARGET);

	LOG_ERR("This is a error log");
	LOG_WRN("This is a warning log");
	LOG_INF("This is a information log");
	LOG_DBG("This is a debug log");

	printk("Done with prints\n");

	return 0;
}
