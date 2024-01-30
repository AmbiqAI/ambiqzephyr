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

#define TESE_SPI DT_NODELABEL(spi0)

#if DT_NODE_HAS_STATUS(TESE_SPI, okay)
static const struct device *spi_dev = DEVICE_DT_GET(TESE_SPI);
#else
#error "Node is disabled"
#endif

#define TEST_FREQ_HZ 8000000U	//48000000U


static struct spi_config spi_cfg_single = {
	.frequency = TEST_FREQ_HZ,
	.operation = (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8)
		      | SPI_LINES_SINGLE),
};

struct spi_buf_set tx_set;
struct spi_buf_set rx_set;

int main(void)
{
	int ret;

	if (!device_is_ready(spi_dev)) {
		printf("SPI 0 device not ready!\n");
		return -1;
	}

	#define DATA_SIZE	1024
	uint8_t buff[DATA_SIZE] = {0};
	memset(buff, 0x5A, DATA_SIZE);
	uint8_t rxdata[DATA_SIZE];

	struct spi_buf tx_buf[1] = {
		{.buf = buff, .len = DATA_SIZE},
	};
	struct spi_buf rx_buf[1] = {
		{.buf = rxdata, .len = 0},
	};

	tx_set.buffers = tx_buf;
	tx_set.count = 1;

	ret = spi_transceive(spi_dev, &spi_cfg_single, &tx_set, NULL);

	tx_buf[0].len = 1;
	rx_set.buffers = rx_buf;
	rx_set.count = 1;
	spi_cfg_single.operation |= SPI_HOLD_ON_CS;
	ret = spi_transceive(spi_dev, &spi_cfg_single, &tx_set, &rx_set);

	tx_buf[0].len = 0;
	rx_buf[0].len = DATA_SIZE;
	spi_cfg_single.operation &= ~SPI_HOLD_ON_CS;
	ret = spi_transceive(spi_dev, &spi_cfg_single, &tx_set, &rx_set);

	return 0;
}
