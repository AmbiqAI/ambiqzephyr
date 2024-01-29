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


static const struct spi_config spi_cfg_single = {
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

	#define DATA_SIZE	8
	uint8_t buff[DATA_SIZE] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
	uint8_t rxdata[DATA_SIZE];

	struct spi_buf tx_buf[1] = {
		{.buf = buff, .len = DATA_SIZE},
	};
	struct spi_buf rx_buf[1] = {
		{.buf = rxdata, .len = DATA_SIZE},
	};

	tx_set.buffers = tx_buf;
	tx_set.count = 1;

	ret = spi_transceive(spi_dev, &spi_cfg_single, &tx_set, NULL);

	printf("8bit_loopback_partial; ret: %d\n", ret);
	printf(" tx (i)  : %02x %02x %02x %02x %02x\n",
	       buff[0], buff[1], buff[2], buff[3], buff[4]);
	// printf(" rx (i)  : %02x %02x %02x %02x %02x\n",
	//        rxdata[0], rxdata[1], rxdata[2], rxdata[3], rxdata[4]);

	tx_buf[0].len = 1;
	rx_set.buffers = rx_buf;
	rx_set.count = 1;
	ret = spi_transceive(spi_dev, &spi_cfg_single, &tx_set, &rx_set);

	tx_buf[0].len = 0;
	ret = spi_transceive(spi_dev, &spi_cfg_single, &tx_set, &rx_set);

	return 0;
}
