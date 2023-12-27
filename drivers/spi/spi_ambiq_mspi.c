/*
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_ap3_mspi

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ambiq_ap3_mspi);

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>

#include "spi_context.h"
#include <am_mcu_apollo.h>

#define SPI_WORD_SIZE        8
#define MSPI_MAX_FREQ        96000000
#define MSPI_TIMEOUT_US      1000000
#define PWRCTRL_MAX_WAIT_US  5
#define MSPI_BUSY            BIT(2)

typedef int (*ambiq_mspi_pwr_func_t)(void);

struct mspi_ambiq_config {
	uint32_t base;
	int size;
	// uint32_t clock_freq;
	am_hal_mspi_dev_config_t mspicfg;
	const struct pinctrl_dev_config *pincfg;
	ambiq_mspi_pwr_func_t pwr_func;
};

struct mspi_ambiq_data {
	struct spi_context ctx;
	void *mspiHandle;
};

static int mspi_set_freq(uint32_t freq)
{
	uint32_t d = MSPI_MAX_FREQ / freq;

	switch (d) {
	case AM_HAL_MSPI_CLK_48MHZ:
	case AM_HAL_MSPI_CLK_24MHZ:
	case AM_HAL_MSPI_CLK_16MHZ:
	case AM_HAL_MSPI_CLK_12MHZ:
	case AM_HAL_MSPI_CLK_8MHZ:
	case AM_HAL_MSPI_CLK_6MHZ:
	case AM_HAL_MSPI_CLK_4MHZ:
	case AM_HAL_MSPI_CLK_3MHZ:
		break;
	default:
		d = AM_HAL_MSPI_CLK_INVALID;
		LOG_ERR("Frequency not supported!");
		break;
	}

	return d;
}

static int mspi_config(const struct device *dev, const struct spi_config *config)
{
	struct mspi_ambiq_data *data = dev->data;
	int ret;
	am_hal_mspi_dev_config_t mspicfg = {0};

	if (config->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	// if (SPI_WORD_SIZE_GET(config->operation) != 8) {
	// 	LOG_ERR("Word size must be %d", SPI_WORD_SIZE);
	// 	return -ENOTSUP;
	// }

	if ((config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("Only single mode is currently supported");
		return -ENOTSUP;
	}

	if (config->operation & SPI_LOCK_ON) {
		LOG_ERR("Lock On not supported");
		return -ENOTSUP;
	}

	if (config->operation & SPI_TRANSFER_LSB) {
		LOG_ERR("LSB first not supported");
		return -ENOTSUP;
	}

	if (config->operation & SPI_MODE_CPOL) {
		if (config->operation & SPI_MODE_CPHA) {
			return AM_HAL_MSPI_SPI_MODE_3;
		} else {
			return AM_HAL_MSPI_SPI_MODE_2;
		}
	} else {
		if (config->operation & SPI_MODE_CPHA) {
			return AM_HAL_MSPI_SPI_MODE_1;
		} else {
			return AM_HAL_MSPI_SPI_MODE_0;
		}
	}


	mspicfg.eClockFreq = mspi_set_freq(config->frequency);
	if (mspicfg.eClockFreq == AM_HAL_MSPI_CLK_INVALID) {
		return -ENOTSUP;
	}

	mspicfg.eDeviceConfig = AM_HAL_MSPI_FLASH_SERIAL_CE0;

	ret = am_hal_mspi_disable(data->mspiHandle);
	if (ret) {
		return ret;
	}

	ret = am_hal_mspi_device_configure(data->mspiHandle, &mspicfg);
	if (ret) {
		return ret;
	}

	ret = am_hal_mspi_enable(data->mspiHandle);


	return ret;
}

static int mspi_ambiq_xfer(const struct device *dev, const struct spi_config *config)
{
	struct mspi_ambiq_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret;

	am_hal_mspi_pio_transfer_t Transaction;
    //
    // Create the individual write transaction.
    //
    // Transaction.ui32NumBytes       = ui32NumBytes;
    Transaction.bScrambling        = false;
    // Transaction.eDirection         = AM_HAL_MSPI_TX;
    Transaction.bSendAddr          = true;
    Transaction.ui32DeviceAddr     = 0xA5 << 8;	//ui32Instr << 8;
    Transaction.bSendInstr         = true;
    Transaction.ui16DeviceInstr    = 0x02;	//AM_DEVICES_MSPI_RM69330_CMD_WRITE;
    Transaction.bTurnaround        = false;
    Transaction.bDCX                    = false;
    Transaction.bEnWRLatency            = false;
    Transaction.bQuadCmd                = false;
    Transaction.bContinue               = false;    // MSPI CONT is deprecated for Apollo3


    // Transaction.pui32Buffer        = (uint32_t*)pData;
	// trans.bSendAddr = true;
	// trans.bSendInstr = true;
	// trans.ui16DeviceInstr = *ctx->tx_buf;
	spi_context_update_tx(ctx, 1, 1);
	// trans.ui32DeviceAddr = *ctx->tx_buf;

	Transaction.eDirection = AM_HAL_MSPI_TX;
	Transaction.pui32Buffer = (uint32_t *)ctx->tx_buf;
	Transaction.ui32NumBytes = ctx->tx_len;

	// if (ctx->rx_buf != NULL) {
	// 	spi_context_update_rx(ctx, 1, ctx->rx_len);
	// 	Transaction.eDirection = AM_HAL_MSPI_RX;
	// 	Transaction.pui32Buffer = (uint32_t *)ctx->rx_buf;
	// 	Transaction.ui32NumBytes = ctx->rx_len;

	// } else if (ctx->tx_buf != NULL) {
	// 	spi_context_update_tx(ctx, 1, 1);
	// 	Transaction.eDirection = AM_HAL_MSPI_TX;
	// 	Transaction.pui32Buffer = (uint32_t *)ctx->tx_buf;
	// 	Transaction.ui32NumBytes = ctx->tx_len;
	// }

	ret = am_hal_mspi_blocking_transfer(data->mspiHandle, &Transaction, MSPI_TIMEOUT_US);

	spi_context_complete(ctx, dev, 0);

	return ret;
}

static int mspi_ambiq_transceive(const struct device *dev, const struct spi_config *config,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	struct mspi_ambiq_data *data = dev->data;

	int ret = mspi_config(dev, config);

	if (ret) {
		return ret;
	}

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	ret = mspi_ambiq_xfer(dev, config);

	return ret;
}

static int mspi_ambiq_release(const struct device *dev, const struct spi_config *config)
{
	const struct mspi_ambiq_config *cfg = dev->config;

	if (sys_read32(cfg->base) & MSPI_BUSY) {
		return -EBUSY;
	}

	return 0;
}

static struct spi_driver_api mspi_ambiq_driver_api = {
	.transceive = mspi_ambiq_transceive,
	.release = mspi_ambiq_release,
};

static int mspi_ambiq_init(const struct device *dev)
{
	int ret;
	struct mspi_ambiq_data *data = dev->data;
	struct mspi_ambiq_config *cfg = dev->config;
	void            *pMspiHandle;

	// am_hal_mspi_config_t mspiCfg = {0};

	// mspiCfg.pTCB = NULL;

	if(AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize((cfg->base - REG_MSPI_BASEADDR) / (cfg->size * 4),
					 &pMspiHandle) )
	{
		// #TODO return value
		return -1;
	}

	ret = cfg->pwr_func();



	// #TODO am_hal_mspi_power_control

	if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pMspiHandle, &cfg->mspicfg) )
	{
		// #TODO return value
		return -1 ;
	}

	if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pMspiHandle))
    {
        // am_util_stdio_printf("Error - Failed to enable MSPI.\n");
		// #TODO return value
        return -1;
    }

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0 && ret != -ENOENT) {
		return ret;
	}

	data->mspiHandle = pMspiHandle;

	return 0;

}

// #error "test"

#define AMBIQ_MSPI_DEFINE(n)                                                                       \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static int pwr_on_ambiq_mspi_##n(void)                                                     \
	{                                                                                          \
		uint32_t addr = DT_REG_ADDR(DT_INST_PHANDLE(n, ambiq_pwrcfg)) +                    \
				DT_INST_PHA(n, ambiq_pwrcfg, offset);                              \
		sys_write32((sys_read32(addr) | DT_INST_PHA(n, ambiq_pwrcfg, mask)), addr);        \
		k_busy_wait(PWRCTRL_MAX_WAIT_US);                                                  \
		return 0;                                                                          \
	}                                                                                          \
	static struct mspi_ambiq_data mspi_ambiq_data##n = {                                       \
		SPI_CONTEXT_INIT_SYNC(mspi_ambiq_data##n, ctx)};                                   \
	static const struct mspi_ambiq_config mspi_ambiq_config##n = {                             \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.size = DT_INST_REG_SIZE(n),	\
		.mspicfg  = { \
			.eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,	\
			.eClockFreq           = AM_HAL_MSPI_CLK_8MHZ,	\
			.ui8TurnAround        = 1,						\
			.eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,		\
			.eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,		\
			.eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE0,	\
			.bSendInstr           = true,		\
			.bSendAddr            = false,		\
			.bTurnaround          = false,		\
			.ui8WriteLatency      = 0,			\
			.bEnWriteLatency      = false,		\
			.bEmulateDDR          = false,		\
			.ui16DMATimeLimit     = 0,			\
			.eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,	\
			.ui32TCBSize          = 0,			\
			.pTCB                 = NULL,		\
			.scramblingStartAddr  = 0,			\
			.scramblingEndAddr    = 0,			\
			.ui8ReadInstr         = 0x0B,	\
			.ui8WriteInstr        = 0x12,	\
		},                                             \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.pwr_func = pwr_on_ambiq_mspi_##n,                                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, mspi_ambiq_init, NULL, &mspi_ambiq_data##n,                       \
			      &mspi_ambiq_config##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,        \
			      &mspi_ambiq_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_MSPI_DEFINE)
