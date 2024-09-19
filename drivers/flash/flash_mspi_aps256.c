/*
 * Copyright (c) 2024, Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mspi_aps256

#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mspi.h>
#if CONFIG_SOC_FAMILY_AMBIQ
#include "mspi_ambiq.h"
typedef struct mspi_ambiq_timing_cfg mspi_timing_cfg;
typedef enum mspi_ambiq_timing_param mspi_timing_param;

#else
typedef struct mspi_timing_cfg mspi_timing_cfg;
typedef enum mspi_timing_param mspi_timing_param;
#define TIMING_CFG_GET_RX_DUMMY(cfg)
#define TIMING_CFG_SET_RX_DUMMY(cfg, num)
#endif

#include <zephyr/drivers/flash.h>
#include "spi_nor.h"
LOG_MODULE_REGISTER(flash_mspi_aps256, CONFIG_FLASH_LOG_LEVEL);

#define NOR_WRITE_SIZE                      1
#define NOR_ERASE_VALUE                     0xff

#define APS256_VENDOR_ID                   0x0D
#define APS256_DEVICE_ID                   0x03

#define MSPI_PSRAM_DDR_GLOBAL_RESET   0xFFFF
#define MSPI_PSRAM_DDR_READ           0x2020
#define MSPI_PSRAM_DDR_WRITE          0xA0A0
#define MSPI_PSRAM_DDR_READ_REGISTER  0x4040
#define MSPI_PSRAM_DDR_WRITE_REGISTER 0xC0C0

enum aps256_dummy_clock {
	APS256_DC_8,
	APS256_DC_10,
	APS256_DC_12,
	APS256_DC_14,
	APS256_DC_16,
	APS256_DC_18,
	APS256_DC_20,
	APS256_DC_22,
};

struct flash_mspi_aps256_config {
	uint32_t                            port;
	uint32_t                            mem_size;
	struct flash_parameters             flash_param;
	struct flash_pages_layout           page_layout;

	const struct device                 *bus;
	struct mspi_dev_id                  dev_id;
	struct mspi_dev_cfg                 tar_dev_cfg;
	struct mspi_xip_cfg                 tar_xip_cfg;
	struct mspi_scramble_cfg            tar_scramble_cfg;

	mspi_timing_cfg                     tar_timing_cfg;
	mspi_timing_param                   timing_cfg_mask;

	bool                                sw_multi_periph;
};

struct flash_mspi_aps256_data {
	struct mspi_dev_cfg                 dev_cfg;
	struct mspi_xip_cfg                 xip_cfg;
	struct mspi_scramble_cfg            scramble_cfg;
	mspi_timing_cfg                     timing_cfg;
	struct mspi_xfer                    trans;
	struct mspi_xfer_packet             packet;

	struct k_sem                        lock;
	uint32_t                            jedec_id;
	uint16_t                            vendor_device_id;
	uint32_t                            device_size_kb;
};

static int flash_mspi_aps256_command_write(const struct device *flash, uint16_t cmd, uint32_t addr,
					    uint16_t addr_len, uint32_t rx_dummy, uint32_t tx_dummy, uint8_t *wdata,
					    uint32_t length)
{
	const struct flash_mspi_aps256_config *cfg = flash->config;
	struct flash_mspi_aps256_data *data = flash->data;
	int ret;

	data->packet.dir              = MSPI_TX;
	data->packet.cmd              = cmd;
	data->packet.address          = addr;
	data->packet.data_buf         = wdata;
	data->packet.num_bytes        = length;

	data->trans.async             = false;
	data->trans.xfer_mode         = MSPI_PIO;
	data->trans.rx_dummy          = rx_dummy;
	data->trans.tx_dummy          = tx_dummy;
	data->trans.cmd_length        = 1;
	data->trans.addr_length       = addr_len;
	data->trans.hold_ce           = false;
	data->trans.packets           = &data->packet;
	data->trans.num_packet        = 1;
	data->trans.timeout           = CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE;

	ret = mspi_transceive(cfg->bus, &cfg->dev_id, (const struct mspi_xfer *)&data->trans);
	if (ret) {
		LOG_ERR("MSPI write transaction failed with code: %d/%u", ret, __LINE__);
		return -EIO;
	}
	return ret;
}

static int flash_mspi_aps256_command_read(const struct device *flash, uint16_t cmd, uint32_t addr,
					  uint16_t addr_len, uint32_t rx_dummy, uint32_t tx_dummy,
					  uint8_t *rdata, uint32_t length)
{
	const struct flash_mspi_aps256_config *cfg = flash->config;
	struct flash_mspi_aps256_data *data = flash->data;
	int ret;

	data->packet.dir              = MSPI_RX;
	data->packet.cmd              = cmd;
	data->packet.address          = addr;
	data->packet.data_buf         = rdata;
	data->packet.num_bytes        = length;

	data->trans.async             = false;
	data->trans.xfer_mode         = MSPI_PIO;
	data->trans.rx_dummy          = rx_dummy;
	data->trans.tx_dummy          = tx_dummy;
	data->trans.cmd_length        = 1;
	data->trans.addr_length       = addr_len;
	data->trans.hold_ce           = false;
	data->trans.packets           = &data->packet;
	data->trans.num_packet        = 1;
	data->trans.timeout           = CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE;

	ret = mspi_transceive(cfg->bus, &cfg->dev_id, (const struct mspi_xfer *)&data->trans);
	if (ret) {
		LOG_ERR("MSPI read transaction failed with code: %d/%u", ret, __LINE__);
		return -EIO;
	}
	return ret;
}

static void acquire(const struct device *flash)
{
	const struct flash_mspi_aps256_config *cfg = flash->config;
	struct flash_mspi_aps256_data *data = flash->data;

	k_sem_take(&data->lock, K_FOREVER);

	if (cfg->sw_multi_periph) {
		while (mspi_dev_config(cfg->bus, &cfg->dev_id,
				       MSPI_DEVICE_CONFIG_ALL, &data->dev_cfg))
			;
	} else {
		while (mspi_dev_config(cfg->bus, &cfg->dev_id,
				       MSPI_DEVICE_CONFIG_NONE, NULL))
			;

	}
}

static void release(const struct device *flash)
{
	const struct flash_mspi_aps256_config *cfg = flash->config;
	struct flash_mspi_aps256_data *data = flash->data;

	while (mspi_get_channel_status(cfg->bus, cfg->port))
		;

	k_sem_give(&data->lock);
}

static int flash_mspi_aps256_reset(const struct device *flash)
{
	int ret;
	uint32_t pio_buffer = 0;

	LOG_DBG("Return to SPI mode");
	ret = flash_mspi_aps256_command_write(flash, MSPI_PSRAM_DDR_GLOBAL_RESET, 0, 4, 0, 0, (uint8_t *)&pio_buffer, 2);

	return ret;
}

static int flash_mspi_aps256_enter_hex_mode(const struct device *flash)
{
	uint8_t io_mode_reg = 0;
	uint32_t raw_data;
	int ret;

	ret = flash_mspi_aps256_command_read(flash, MSPI_PSRAM_DDR_READ_REGISTER, 8, 4, 6,
							6, (uint8_t *)&raw_data, 4);
	if (0 != ret) {
		LOG_ERR("Failed to read PSRAM MR8 register!\n");
		return ret;
	} else {
		io_mode_reg = (uint8_t)raw_data;
		LOG_DBG("PSRAM Register MR8 = 0x%X\n", io_mode_reg);
		LOG_DBG("PSRAM IO mode = 0x%X\n\n", (io_mode_reg & 0x40) >> 6);
	}

	raw_data = io_mode_reg | 0x40;

	ret = flash_mspi_aps256_command_write(flash, MSPI_PSRAM_DDR_WRITE_REGISTER, 8, 4, 6,
							0, (uint8_t *)&raw_data, 4);
	if (0 != ret) {
		LOG_ERR("Failed to set PSRAM into HEX mode!\n");
		return ret;
	} else {
		LOG_DBG("Set PSRAM into HEX mode\n\n");
	}

	return ret;
}

static int flash_mspi_aps256_device_init(const struct device *flash)
{
	int ret;
	uint32_t raw_data;
	uint8_t vendor_id_reg = 0;
	uint8_t device_id_reg = 0;
	uint8_t device_id = 0;
	uint8_t rlc_reg = 0;
	struct flash_mspi_aps256_data *data = flash->data;

	/* Read and set PSRAM Register MR0 */
	LOG_DBG("Read PSRAM Register MR0\n");
	ret = flash_mspi_aps256_command_read(flash, MSPI_PSRAM_DDR_READ_REGISTER, 0, 4, 6, 6,
					     (uint8_t *)&raw_data, 4);
	if (0 != ret)
	{
		LOG_ERR("Failed to read PSRAM Register MR0!\n");
		return ret;
	}
	else
	{
		rlc_reg = (uint8_t)raw_data;
		LOG_DBG("PSRAM Register MR0 = 0x%02X\n", rlc_reg);
		LOG_DBG("PSRAM Read Latency Code = 0x%02X\n", ((rlc_reg & 0x1C)>>2) + 3 );
	}

	rlc_reg &= 0xC0;                        /* set latency to 3 (0b000) */
	rlc_reg |= 0x01;                        /* set PSRAM drive strength (0:Full 1:Half(default) 2:Quarter 3: Eighth) */
	if (data->dev_cfg.dqs_enable == false) {
		rlc_reg |= 0x20; /* set LT to fixed */
		LOG_DBG("Set read latency into fixed type in NON-DQS mode!\n");
	}

	raw_data = rlc_reg;

	ret = flash_mspi_aps256_command_write(flash, MSPI_PSRAM_DDR_WRITE_REGISTER, 0, 4, 0, 0,
					      (uint8_t *)&raw_data, 4);
	if (0 != ret)
	{
		LOG_ERR("Failed to write PSRAM Register MR0!\n");
		return ret;
	}
	else
	{
		LOG_DBG("Set PSRAM Register MR0 into 0x%02X\n", rlc_reg);
	}

	LOG_DBG("Read PSRAM Read Latency Code\n");
	ret = flash_mspi_aps256_command_read(flash, MSPI_PSRAM_DDR_READ_REGISTER, 0, 4, 6, 6,
					     (uint8_t *)&raw_data, 4);
	if (0 != ret)
	{
		LOG_ERR("Failed to read PSRAM Read Latency Code!\n");
		return ret;
	}
	else
	{
		rlc_reg = (uint8_t)raw_data;
		LOG_DBG("PSRAM Register MR0 = 0x%02X\n", rlc_reg);
		LOG_DBG("PSRAM Read Latency Code = 0x%02X\n\n", ((rlc_reg & 0x1C)>>2) + 3 );
	}

	/* Read and set PSRAM Write Latency Code */
	LOG_DBG("Read PSRAM Write Latency Code\n");
	ret = flash_mspi_aps256_command_read(flash, MSPI_PSRAM_DDR_READ_REGISTER, 4, 4, 6, 6,
					     (uint8_t *)&raw_data, 4);
	if (0 != ret)
	{
		LOG_ERR("Failed to read PSRAM Write Latency Code!\n");
		return ret;
	}
	else
	{
		rlc_reg = (uint8_t)raw_data;
		LOG_DBG("PSRAM Register MR4 = 0x%02X\n", rlc_reg);
		LOG_DBG("PSRAM Write Latency Code = 0x%02X\n", ((rlc_reg & 0xE0)>>5) + 3 );
	}

	raw_data = rlc_reg & 0x1F;

	ret = flash_mspi_aps256_command_write(flash, MSPI_PSRAM_DDR_WRITE_REGISTER, 4, 4, 0, 0,
					      (uint8_t *)&raw_data, 4);
	if (0 != ret)
	{
		LOG_ERR("Failed to write PSRAM Write Latency Code!\n");
		return ret;
	}
	else
	{
		LOG_DBG("Set PSRAM Write Latency Code into 3\n");
	}

	LOG_DBG("Read PSRAM Write Latency Code\n");
	ret = flash_mspi_aps256_command_read(flash, MSPI_PSRAM_DDR_READ_REGISTER, 4, 4, 6, 6,
					     (uint8_t *)&raw_data, 4);
	if (0 != ret)
	{
		LOG_ERR("Failed to read PSRAM Write Latency Code!\n");
		return ret;
	}
	else
	{
		rlc_reg = (uint8_t)raw_data;
		LOG_DBG("PSRAM Register MR4 = 0x%02X\n", rlc_reg);
		LOG_DBG("PSRAM Write Latency Code = 0x%02X\n\n", ((rlc_reg & 0xE0)>>5) + 3 );
	}

	/* Read PSRAM Vendor ID and Device ID and return status. */
	LOG_DBG("Read PSRAM Vendor ID\n");
	ret = flash_mspi_aps256_command_read(flash, MSPI_PSRAM_DDR_READ_REGISTER, 1, 4, 6, 6,
					     (uint8_t *)&raw_data, 4);
	if (0 != ret)
	{
		LOG_ERR("Failed to read PSRAM Vendor ID!\n");
		return ret;
	}
	else
	{
		vendor_id_reg = (uint8_t)raw_data;
		LOG_DBG("PSRAM Register MR1 = 0x%X\n", vendor_id_reg);
		if ( (vendor_id_reg & 0x1F) == 0xD )
		{
			LOG_DBG("PSRAM Vendor ID =  01101\n\n");
			data->vendor_device_id = ((vendor_id_reg & 0x1F) << 8) & 0xFF00;
		} else {
			LOG_DBG("Fail to get correct PSRAM Vendor ID!\n\n");
			return ret;
		}
	}

	LOG_DBG("Read PSRAM Device ID\n");
	ret = flash_mspi_aps256_command_read(flash, MSPI_PSRAM_DDR_READ_REGISTER, 2, 4, 6, 6,
					     (uint8_t *)&raw_data, 4);
	if (0 != ret)
	{
		LOG_ERR("Failed to read PSRAM Device ID!\n");
		return ret;
	}
	else
	{
		device_id_reg = (uint8_t)raw_data;
		device_id = (device_id_reg & 0x18) >> 3;
		data->vendor_device_id |= device_id & 0xFF;
		LOG_DBG("PSRAM Register MR2 = 0x%X\n", device_id_reg);
		LOG_DBG("PSRAM Device ID =  Generation %d\n", device_id + 1);
		if ( (device_id_reg & 0x7) == 0x1 )
		{
			data->device_size_kb = 32 / 8 * 1024U;
			LOG_DBG("PSRAM Density =  32Mb\n\n");
		}
		else if ( (device_id_reg & 0x7) == 0x3 )
		{
			data->device_size_kb = 64 / 8 * 1024U;
			LOG_DBG("PSRAM Density =  64Mb\n\n");
		}
		else if ( (device_id_reg & 0x7) == 0x5 )
		{
			data->device_size_kb = 128 / 8 * 1024U;
			LOG_DBG("PSRAM Density =  128Mb\n\n");
		}
		else if ( (device_id_reg & 0x7) == 0x7 )
		{
			data->device_size_kb = 256 / 8 * 1024U;
			LOG_DBG("PSRAM Density =  256Mb\n\n");
		}
	}

	return ret;
}

static int flash_mspi_aps256_page_program(const struct device *flash, off_t offset, void *wdata,
					   size_t len)
{
	const struct flash_mspi_aps256_config *cfg = flash->config;
	struct flash_mspi_aps256_data *data = flash->data;
	int ret;

	data->packet.dir              = MSPI_TX;
	data->packet.cmd              = data->dev_cfg.write_cmd;
	data->packet.address          = offset;
	data->packet.data_buf         = wdata;
	data->packet.num_bytes        = len;

	data->trans.async             = false;
	data->trans.xfer_mode         = MSPI_DMA;
	data->trans.tx_dummy          = data->dev_cfg.tx_dummy;
	data->trans.cmd_length        = data->dev_cfg.cmd_length;
	data->trans.addr_length       = data->dev_cfg.addr_length;
	data->trans.hold_ce           = false;
	data->trans.priority          = 1;
	data->trans.packets           = &data->packet;
	data->trans.num_packet        = 1;
	data->trans.timeout           = CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE;

	LOG_DBG("Page programming %d bytes to 0x%08zx", len, (ssize_t)offset);

	ret = mspi_transceive(cfg->bus, &cfg->dev_id, (const struct mspi_xfer *)&data->trans);
	if (ret) {
		LOG_ERR("MSPI write transaction failed with code: %d/%u", ret, __LINE__);
		return -EIO;
	}
	return ret;
}

static int flash_mspi_aps256_erase_area(const struct device *flash, off_t addr, int size)
{
	int ret;
	uint8_t buf[SPI_NOR_PAGE_SIZE];

	memset(buf, 0xFF, SPI_NOR_PAGE_SIZE);

	while ( size > 0 ) {
		int len = size;
		if (len >= SPI_NOR_PAGE_SIZE) {
			len = SPI_NOR_PAGE_SIZE;
		} else {
			len %= SPI_NOR_PAGE_SIZE;
		}

		LOG_DBG("\nErasing sector at 0x%08zx", (ssize_t)addr);
		ret = flash_mspi_aps256_page_program(flash, addr, &buf, len);
		if (ret) {
			return ret;
		}

		size -= SPI_NOR_PAGE_SIZE;
		addr += SPI_NOR_PAGE_SIZE;
	}

	return ret;
}

static int flash_mspi_aps256_read(const struct device *flash, off_t offset, void *rdata,
				   size_t len)
{
	const struct flash_mspi_aps256_config *cfg = flash->config;
	struct flash_mspi_aps256_data *data = flash->data;

	int ret;

	acquire(flash);

	data->packet.dir              = MSPI_RX;
	data->packet.cmd              = data->dev_cfg.read_cmd;
	data->packet.address          = offset;
	data->packet.data_buf         = rdata;
	data->packet.num_bytes        = len;

	data->trans.async             = false;
	data->trans.xfer_mode         = MSPI_DMA;
	data->trans.rx_dummy          = data->dev_cfg.rx_dummy;
	data->trans.cmd_length        = data->dev_cfg.cmd_length;
	data->trans.addr_length       = data->dev_cfg.addr_length;
	data->trans.hold_ce           = false;
	data->trans.priority          = 1;
	data->trans.packets           = &data->packet;
	data->trans.num_packet        = 1;
	data->trans.timeout           = CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE;

	LOG_DBG("Read %d bytes from 0x%08zx", len, (ssize_t)offset);

	ret = mspi_transceive(cfg->bus, &cfg->dev_id, (const struct mspi_xfer *)&data->trans);
	if (ret) {
		LOG_ERR("MSPI read transaction failed with code: %d/%u", ret, __LINE__);
		return -EIO;
	}

	release(flash);

	return ret;
}

static int flash_mspi_aps256_write(const struct device *flash, off_t offset, const void *wdata,
				    size_t len)
{
	int ret;
	uint8_t *src = (uint8_t *)wdata;
	int i;

	acquire(flash);

	while (len) {
		/* If the offset isn't a multiple of the NOR page size, we first need
		 * to write the remaining part that fits, otherwise the write could
		 * be wrapped around within the same page
		 */
		i = MIN(SPI_NOR_PAGE_SIZE - (offset % SPI_NOR_PAGE_SIZE), len);

		ret = flash_mspi_aps256_page_program(flash, offset, src, i);
		if (ret) {
			return ret;
		}

		src += i;
		offset += i;
		len -= i;
	}

	release(flash);

	return ret;
}

static int flash_mspi_aps256_erase(const struct device *flash, off_t offset, size_t size)
{
	int ret = 0;
	const size_t num_sectors = size / SPI_NOR_SECTOR_SIZE;

	int i;

	acquire(flash);

	if (offset % SPI_NOR_SECTOR_SIZE) {
		LOG_ERR("Invalid offset");
		return -EINVAL;
	}

	if (size % SPI_NOR_SECTOR_SIZE) {
		LOG_ERR("Invalid size");
		return -EINVAL;
	}

	for (i = 0; i < num_sectors; i++) {
		ret = flash_mspi_aps256_erase_area(flash, offset, size);
		if (ret) {
			return ret;
		}

		offset += SPI_NOR_SECTOR_SIZE;
	}

	release(flash);

	return ret;
}

static const struct flash_parameters *flash_mspi_aps256_get_parameters(const struct device *flash)
{
	const struct flash_mspi_aps256_config *cfg = flash->config;

	return &cfg->flash_param;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_mspi_aps256_pages_layout(const struct device *flash,
					    const struct flash_pages_layout **layout,
					    size_t *layout_size)
{
	const struct flash_mspi_aps256_config *cfg = flash->config;

	*layout = &cfg->page_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static int flash_mspi_aps256_init(const struct device *flash)
{
	const struct flash_mspi_aps256_config *cfg = flash->config;
	struct flash_mspi_aps256_data *data = flash->data;
	struct mspi_dev_cfg lcl_dev_cfg = cfg->tar_dev_cfg;

#define AM_BSP_GPIO_MSPI1_MUX_SEL 34
#define AM_BSP_GPIO_MSPI1_MUX_OE 35
    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_MUX_SEL, am_hal_gpio_pincfg_output);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_MUX_OE, am_hal_gpio_pincfg_output);
    am_hal_gpio_state_write(AM_BSP_GPIO_MSPI1_MUX_SEL, AM_HAL_GPIO_OUTPUT_SET);
    am_hal_gpio_state_write(AM_BSP_GPIO_MSPI1_MUX_OE, AM_HAL_GPIO_OUTPUT_CLEAR);

	if (!device_is_ready(cfg->bus))
	{
		LOG_ERR("Controller device is not ready");
		return -ENODEV;
	}

	switch (cfg->tar_dev_cfg.io_mode) {
	case MSPI_IO_MODE_SINGLE:
	case MSPI_IO_MODE_QUAD:
	case MSPI_IO_MODE_OCTAL:
	case MSPI_IO_MODE_HEX:
		break;
	default:
		LOG_ERR("bus mode %d not supported/%u", cfg->tar_dev_cfg.io_mode, __LINE__);
		return -EIO;
	}

	if (cfg->tar_dev_cfg.io_mode == MSPI_IO_MODE_HEX) {
		lcl_dev_cfg.io_mode = MSPI_IO_MODE_OCTAL;
	}

	data->dev_cfg = cfg->tar_dev_cfg;

	// if (mspi_timing_config(cfg->bus, &cfg->dev_id, cfg->timing_cfg_mask,
	// 		       (void *)&cfg->tar_timing_cfg)) {
	// 	LOG_ERR("Failed to config mspi timing/%u", __LINE__);
	// 	return -EIO;
	// }

	data->timing_cfg = cfg->tar_timing_cfg;

	if (mspi_dev_config(cfg->bus, &cfg->dev_id, MSPI_DEVICE_CONFIG_ALL, &lcl_dev_cfg)) {
		LOG_ERR("Failed to config mspi controller/%u", __LINE__);
		return -EIO;
	}

	if (cfg->tar_scramble_cfg.enable) {
		if (mspi_scramble_config(cfg->bus, &cfg->dev_id, &cfg->tar_scramble_cfg)) {
			LOG_ERR("Failed to enable scrambling/%u", __LINE__);
			return -EIO;
		}
		data->scramble_cfg = cfg->tar_scramble_cfg;
	}

	if (mspi_xip_config(cfg->bus, &cfg->dev_id, &cfg->tar_xip_cfg)) {
		LOG_ERR("Failed to enable XIP/%u", __LINE__);
		return -EIO;
	}

	data->xip_cfg = cfg->tar_xip_cfg;

	if (flash_mspi_aps256_reset(flash)) {
		LOG_ERR("Could not reset Flash/%u", __LINE__);
		return -EIO;
	}

	if (flash_mspi_aps256_device_init(flash)) {
		LOG_ERR("Could not perform device init/%u", __LINE__);
		return -EIO;
	}

	if (cfg->tar_dev_cfg.io_mode == MSPI_IO_MODE_HEX) {
		if (flash_mspi_aps256_enter_hex_mode(flash)) {
			LOG_ERR("Could not enter hex mode/%u", __LINE__);
			return -EIO;
		}

		lcl_dev_cfg.io_mode = MSPI_IO_MODE_HEX;
	}

	if (mspi_dev_config(cfg->bus, &cfg->dev_id, MSPI_DEVICE_CONFIG_IO_MODE, &lcl_dev_cfg)) {
		LOG_ERR("Failed to config mspi controller to HEX mode/%u", __LINE__);
		return -EIO;
	}

	release(flash);

	return 0;
}

#if defined(CONFIG_FLASH_JESD216_API)
static int flash_mspi_aps256_read_sfdp(const struct device *flash, off_t addr, void *rdata,
					size_t size)
{
	const struct flash_mspi_aps256_config *cfg = flash->config;
	struct flash_mspi_aps256_data *data = flash->data;
	int ret;

	acquire(flash);

	data->packet.dir              = MSPI_RX;
	data->packet.cmd              = 0x5A;
	data->packet.address          = addr;
	data->packet.data_buf         = rdata;
	data->packet.num_bytes        = size;

	data->trans.async             = false;
	data->trans.xfer_mode         = MSPI_DMA;
	data->trans.rx_dummy          = 8;
	data->trans.cmd_length        = 1;
	data->trans.addr_length       = 3;
	data->trans.hold_ce           = false;
	data->trans.priority          = 1;
	data->trans.packets           = &data->packet;
	data->trans.num_packet        = 1;
	data->trans.timeout           = CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE;

	LOG_DBG("Read %d bytes from 0x%08zx", size, (ssize_t)addr);

	ret = mspi_transceive(cfg->bus, &cfg->dev_id, (const struct mspi_xfer *)&data->trans);

	if (ret) {
		LOG_ERR("MSPI read transaction failed with code: %d/%u", ret, __LINE__);
		return -EIO;
	}

	release(flash);
	return 0;
}
static int flash_mspi_aps256_read_jedec_id(const struct device *flash, uint8_t *id)
{
	struct flash_mspi_aps256_data *data = flash->data;

	id = &data->jedec_id;
	return 0;
}
#endif /* CONFIG_FLASH_JESD216_API */

#if defined(CONFIG_PM_DEVICE)
static int flash_mspi_aps256_pm_action(const struct device *flash, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		acquire(flash);

		release(flash);
		break;

	case PM_DEVICE_ACTION_SUSPEND:
		acquire(flash);

		release(flash);
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /** IS_ENABLED(CONFIG_PM_DEVICE) */

static const struct flash_driver_api flash_mspi_aps256_api = {
	.erase = flash_mspi_aps256_erase,
	.write = flash_mspi_aps256_write,
	.read = flash_mspi_aps256_read,
	.get_parameters = flash_mspi_aps256_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_mspi_aps256_pages_layout,
#endif
#if defined(CONFIG_FLASH_JESD216_API)
	.sfdp_read = flash_mspi_aps256_read_sfdp,
	.read_jedec_id = flash_mspi_aps256_read_jedec_id,
#endif
};

#if CONFIG_SOC_FAMILY_AMBIQ
#define MSPI_TIMING_CONFIG(n)                                                                     \
	{                                                                                         \
		.ui8WriteLatency    = DT_INST_PROP_BY_IDX(n, ambiq_timing_config, 0),             \
		.ui8TurnAround      = DT_INST_PROP_BY_IDX(n, ambiq_timing_config, 1),             \
		.bTxNeg             = DT_INST_PROP_BY_IDX(n, ambiq_timing_config, 2),             \
		.bRxNeg             = DT_INST_PROP_BY_IDX(n, ambiq_timing_config, 3),             \
		.bRxCap             = DT_INST_PROP_BY_IDX(n, ambiq_timing_config, 4),             \
		.ui32TxDQSDelay     = DT_INST_PROP_BY_IDX(n, ambiq_timing_config, 5),             \
		.ui32RxDQSDelay     = DT_INST_PROP_BY_IDX(n, ambiq_timing_config, 6),             \
		.ui32RXDQSDelayEXT  = DT_INST_PROP_BY_IDX(n, ambiq_timing_config, 7),             \
	}
#define MSPI_TIMING_CONFIG_MASK(n) DT_INST_PROP(n, ambiq_timing_config_mask)
#else
#define MSPI_TIMING_CONFIG(n)
#define MSPI_TIMING_CONFIG_MASK(n)
#endif

#define FLASH_MSPI_APS256(n)                                                                     \
	static const struct flash_mspi_aps256_config flash_mspi_aps256_config_##n = {           \
		.port = (DT_REG_ADDR(DT_INST_BUS(n)) - MSPI0_BASE) /                       \
			(DT_REG_SIZE(DT_INST_BUS(n)) * 4),                                        \
		.mem_size = DT_INST_PROP(n, size) / 8,                                            \
		.flash_param =                                                                    \
			{                                                                         \
				.write_block_size = NOR_WRITE_SIZE,                               \
				.erase_value = NOR_ERASE_VALUE,                                   \
			},                                                                        \
		.page_layout =                                                                    \
			{                                                                         \
				.pages_count = DT_INST_PROP(n, size) / 8 / SPI_NOR_PAGE_SIZE,     \
				.pages_size = SPI_NOR_PAGE_SIZE,                                  \
			},                                                                        \
		.bus                = DEVICE_DT_GET(DT_INST_BUS(n)),                              \
		.dev_id             = MSPI_DEVICE_ID_DT_INST(n),                                  \
		.tar_dev_cfg        = MSPI_DEVICE_CONFIG_DT_INST(n),                              \
		.tar_xip_cfg        = MSPI_XIP_CONFIG_DT_INST(n),                                 \
		.tar_scramble_cfg   = MSPI_SCRAMBLE_CONFIG_DT_INST(n),                            \
		.tar_timing_cfg     = MSPI_TIMING_CONFIG(n),                                      \
		.timing_cfg_mask    = MSPI_TIMING_CONFIG_MASK(n),                                 \
		.sw_multi_periph     = DT_PROP(DT_INST_BUS(n), software_multiperipheral)                \
	};                                                                                        \
	static struct flash_mspi_aps256_data flash_mspi_aps256_data_##n = {                     \
		.lock = Z_SEM_INITIALIZER(flash_mspi_aps256_data_##n.lock, 0, 1),                \
	};                                                                                        \
	PM_DEVICE_DT_INST_DEFINE(n, flash_mspi_aps256_pm_action);                                \
	DEVICE_DT_INST_DEFINE(n,                                                                  \
			      flash_mspi_aps256_init,                                            \
			      PM_DEVICE_DT_INST_GET(n),                                           \
			      &flash_mspi_aps256_data_##n,                                       \
			      &flash_mspi_aps256_config_##n,                                     \
			      POST_KERNEL,                                                        \
			      CONFIG_FLASH_INIT_PRIORITY,                                         \
			      &flash_mspi_aps256_api);

DT_INST_FOREACH_STATUS_OKAY(FLASH_MSPI_APS256)
