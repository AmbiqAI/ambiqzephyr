/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/spi.h>

#define TESE_MSPI DT_NODELABEL(mspi0)

#if DT_NODE_HAS_STATUS(TESE_MSPI, okay)
static const struct device *spi_dev = DEVICE_DT_GET(TESE_MSPI);
#else
#error "Node is disabled"
#endif

struct spi_config config;
int main(void)
{
	if (!device_is_ready(spi_dev)) {
		printf("SPI 0 device not ready!\n");
		return -1;
	}
	struct spi_cs_control cs_ctrl = (struct spi_cs_control){
		// .gpio = GPIO_DT_SPEC_GET(SPIBB_NODE, cs_gpios),
		.delay = 0u,
	};

	config.frequency = 48000000;
	config.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8);
	config.slave = 0;
	config.cs = cs_ctrl;

	enum { datacount = 5 };
	uint8_t buff[datacount] = { 0x01, 0x02, 0x03, 0x04, 0x05};
	uint8_t rxdata[datacount];

	struct spi_buf tx_buf[1] = {
		{.buf = buff, .len = datacount},
	};
	struct spi_buf rx_buf[1] = {
		{.buf = rxdata, .len = datacount},
	};

	struct spi_buf_set tx_set = { .buffers = tx_buf, .count = 5 };
	struct spi_buf_set rx_set = { .buffers = rx_buf, .count = 5 };

	int ret = spi_transceive(spi_dev, &config, &tx_set, &rx_set);

	printf("8bit_loopback_partial; ret: %d\n", ret);
	printf(" tx (i)  : %02x %02x %02x %02x %02x\n",
	       buff[0], buff[1], buff[2], buff[3], buff[4]);
	printf(" rx (i)  : %02x %02x %02x %02x %02x\n",
	       rxdata[0], rxdata[1], rxdata[2], rxdata[3], rxdata[4]);
	return 0;
}
