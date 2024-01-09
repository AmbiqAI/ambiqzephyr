/*
 * Copyright (c) 2023 Ambiq <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_mspi

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(mspi_ambiq);

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>

#include "spi_context.h"
#include <am_mcu_apollo.h>

#define SPI_WORD_SIZE        8
#define MSPI_BASE_CLOCK      48000000
#define MSPI_TIMEOUT_US      1000000
#define PWRCTRL_MAX_WAIT_US  5
#define MSPI_BUSY            BIT(2)

typedef int (*ambiq_mspi_pwr_func_t)(void);

#if (CONFIG_DMA_BUFFER_SIZE <= 0)
#define CONFIG_DMA_BUFFER_SIZE 4096
#endif /* CONFIG_DMA_BUFFER_SIZE */

struct mspi_ambiq_config {
	uint32_t base;
	int size;
	uint8_t chip_sel;
	struct gpio_dt_spec cs_gpio;
	const struct pinctrl_dev_config *pincfg;
    ambiq_mspi_pwr_func_t           pwr_func;
	void (*irq_config_func)(void);
    LOG_INSTANCE_PTR_DECLARE(log);
};

struct mspi_ambiq_data {
	struct spi_context 		ctx;
	void 					*mspiHandle;
	uint32_t 				pDMATCBBuffer[4096];

};

#define AM_PART_APOLLO3P
// MSPI configuration
static am_hal_mspi_dev_config_t  MspiCfgDefault =
{
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_48MHZ,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE0,
    .pTCB                 = 0,

    .bSendInstr           = false,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .bSendAddr            = false,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
    .bTurnaround          = false,
    .ui8TurnAround        = 0,
    .bEnWriteLatency      = false,
    .ui8WriteLatency      = 0,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
    .ui32TCBSize          = 0,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
    .ui8ReadInstr         = 0,
    .ui8WriteInstr        = 0,
};

static void
pfnMSPI_Callback(void *pCallbackCtxt, uint32_t status)
{
    // Set the DMA complete flag.
	const struct device *dev = pCallbackCtxt;
	struct mspi_ambiq_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	spi_context_complete(ctx, dev, 0);
}

static void mspi_ambiq_isr(const struct device *dev)
{
    uint32_t      ui32Status;
	struct mspi_ambiq_data *data = dev->data;

    am_hal_mspi_interrupt_status_get(data->mspiHandle, &ui32Status, false);
    am_hal_mspi_interrupt_clear(data->mspiHandle, ui32Status);
    am_hal_mspi_interrupt_service(data->mspiHandle, ui32Status);
}


static am_hal_mspi_clock_e mspi_set_freq(const struct mspi_ambiq_config *cfg, uint32_t freq)
{
	am_hal_mspi_clock_e clkdiv = MSPI_BASE_CLOCK / freq;

	switch (clkdiv) {
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
		clkdiv = 0;
		LOG_INST_ERR(cfg->log, "[%s:%d] Frequency not supported!",__func__,__LINE__);
		break;
	}

	return clkdiv;
}

static am_hal_mspi_device_e mspi_set_line(const struct mspi_ambiq_config *cfg, spi_operation_t operation)
{
	am_hal_mspi_device_e line_mode;

	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES))
	{
		switch (operation & SPI_LINES_MASK)
		{
		case SPI_LINES_SINGLE:
			line_mode = AM_HAL_MSPI_FLASH_SERIAL_CE0;
			break;
		case SPI_LINES_DUAL:
			line_mode = AM_HAL_MSPI_FLASH_DUAL_CE0;
			break;
		case SPI_LINES_QUAD:
			line_mode = AM_HAL_MSPI_FLASH_QUAD_CE0;
			break;
		case SPI_LINES_OCTAL:
			line_mode = AM_HAL_MSPI_FLASH_OCTAL_CE0;
			break;
		default:
			LOG_ERR("SPI Lines On not supported");
			break;
		}
	}
	if(cfg->chip_sel)
	{
		line_mode += 1;
	}
	return line_mode;
}

static int mspi_config(const struct device *dev, const struct spi_config *config)
{
	const struct mspi_ambiq_config *cfg = dev->config;
	struct mspi_ambiq_data *data = dev->data;
	am_hal_mspi_dev_config_t mspicfg_tmp;
	int ret;

	if (config->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

    if (SPI_WORD_SIZE_GET(config->operation) != 8) {
        LOG_INST_ERR(cfg->log, "%u,Word reg_size must be %d", SPI_WORD_SIZE, __LINE__);
        return -ENOTSUP;
    }

	if (config->operation & SPI_TRANSFER_LSB) {
		LOG_ERR("LSB first not supported");
		return -ENOTSUP;
	}
	if (config->operation & SPI_LOCK_ON) {
		LOG_ERR("Lock On not supported");
		return -ENOTSUP;
	}

	mspicfg_tmp = MspiCfgDefault;

	if (config->operation & SPI_MODE_CPOL) {
		if (config->operation & SPI_MODE_CPHA) {
			mspicfg_tmp.eSpiMode = AM_HAL_MSPI_SPI_MODE_3;
		} else {
			mspicfg_tmp.eSpiMode =  AM_HAL_MSPI_SPI_MODE_2;
		}
	} else {
		if (config->operation & SPI_MODE_CPHA) {
			mspicfg_tmp.eSpiMode = AM_HAL_MSPI_SPI_MODE_1;
		} else {
			mspicfg_tmp.eSpiMode =  AM_HAL_MSPI_SPI_MODE_0;
		}
	}


	mspicfg_tmp.eClockFreq = mspi_set_freq(cfg, config->frequency);
	if (mspicfg_tmp.eClockFreq == 0) {
		return -ENOTSUP;
	}

	mspicfg_tmp.eDeviceConfig = mspi_set_line(cfg, config->operation);
	if (mspicfg_tmp.eDeviceConfig == AM_HAL_MSPI_FLASH_MAX) {
		return -ENOTSUP;
	}

	// mspicfg_tmp.pTCB = data->pDMATCBBuffer;
	// mspicfg_tmp.ui32TCBSize = CONFIG_DMA_BUFFER_SIZE / 4;
	mspicfg_tmp.pTCB = data->pDMATCBBuffer;
	mspicfg_tmp.ui32TCBSize = (sizeof(data->pDMATCBBuffer)/sizeof(uint32_t));

	ret = am_hal_mspi_disable(data->mspiHandle);
	if (ret) {
		return ret;
	}

	ret = am_hal_mspi_device_configure(data->mspiHandle, &mspicfg_tmp);
	if (ret) {
		return ret;
	}

	ret = am_hal_mspi_enable(data->mspiHandle);

	return ret;
}



static int mspi_ambiq_xfer(const struct device *dev, const struct spi_config *config)
{
	const struct mspi_ambiq_config *cfg = dev->config;
	struct mspi_ambiq_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret;

    am_hal_mspi_dma_transfer_t    Transaction;

	Transaction.ui8Priority = 1;
	Transaction.ui32PauseCondition = 0;
	Transaction.ui32StatusSetClr = 0;

	if (ctx->tx_len == 0) {
		/* rx only, nothing to tx */
		/* Set the transfer direction to RX (Read) */
		Transaction.eDirection = AM_HAL_MSPI_RX;
		Transaction.ui32SRAMAddress = (uint32_t)ctx->rx_buf;
		Transaction.ui32TransferCount = ctx->rx_len;
		ret = am_hal_mspi_nonblocking_transfer(data->mspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA,  pfnMSPI_Callback,(void *)dev);
	} else if (ctx->rx_len == 0) {
		/* tx only, nothing to rx */
		/* Set the transfer direction to TX (Write) */
		Transaction.eDirection = AM_HAL_MSPI_TX;
		Transaction.ui32SRAMAddress = (uint32_t)ctx->tx_buf;
		Transaction.ui32TransferCount = ctx->tx_len;
		ret = am_hal_mspi_nonblocking_transfer(data->mspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_Callback,(void *)dev);
	}
	else {
		/* Breaks the data into two transfers, keeps the chip select active between the two transfers,
		 * and reconfigures the pin to the CS function when the transfer is complete.
		 */
		gpio_pin_configure_dt(&cfg->cs_gpio, GPIO_OUTPUT_INACTIVE);

		/* Set the transfer direction to TX (Write) */
		Transaction.eDirection = AM_HAL_MSPI_TX;
		Transaction.ui32SRAMAddress = (uint32_t)ctx->tx_buf;
		Transaction.ui32TransferCount = ctx->tx_len;
		ret = am_hal_mspi_nonblocking_transfer(data->mspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, NULL,NULL);

		/* Set the transfer direction to RX (Read) */
		Transaction.eDirection = AM_HAL_MSPI_RX;
		Transaction.ui32SRAMAddress = (uint32_t)ctx->rx_buf;
		Transaction.ui32TransferCount = ctx->rx_len;
		ret = am_hal_mspi_nonblocking_transfer(data->mspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, NULL,NULL);
		gpio_pin_configure_dt(&cfg->cs_gpio, GPIO_OUTPUT_ACTIVE);
		ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
		spi_context_complete(ctx, dev, 0);
	}
	return ret;
}

static int mspi_ambiq_transceive(const struct device *dev, const struct spi_config *config,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	struct mspi_ambiq_data *data = dev->data;
	int ret;

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}

	/* context setup */
	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	ret = mspi_config(dev, config);
	if (ret) {
		goto done;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	ret = mspi_ambiq_xfer(dev, config);

done:
	spi_context_release(&data->ctx, ret);

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
	struct mspi_ambiq_data *data = dev->data;
	const struct mspi_ambiq_config *cfg = dev->config;
	void *pMspiHandle;
	void *buf;

	int ret;

	if(AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize((cfg->base - REG_MSPI_BASEADDR) / (cfg->size * 4),
					 &pMspiHandle) )
	{
        LOG_INST_ERR(cfg->log, "[%s:%d] Fail to initialize MSPI, code:%d",__func__,__LINE__, ret);
        return -EIO;
	}

	ret = cfg->pwr_func();

	data->mspiHandle = pMspiHandle;

	/* default SPI Mode 0 signalling */
	struct spi_config spi_cfg = {
		.frequency = MSPI_BASE_CLOCK,
		.operation = SPI_WORD_SET(8),
	};

	ret = mspi_config(dev, &spi_cfg);
	if (ret) {
		LOG_ERR("Ambiq MSPi init configure failed (%d)", ret);
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0 && ret != -ENOENT) {
		return ret;
	}

	cfg->irq_config_func();
#if 0
	buf = k_malloc(CONFIG_DMA_BUFFER_SIZE);
	if (buf == NULL) {
		return -ENOMEM;
	}
	data->pDMATCBBuffer = (uint32_t*)buf;
#endif

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_interrupt_clear(data->mspiHandle,	\
											 AM_HAL_MSPI_INT_CQUPD |AM_HAL_MSPI_INT_ERR)) {
		ret = -ENODEV;
        LOG_INST_ERR(cfg->log, "%u, Fail to clear interrupt, code:%d",
                     __LINE__, ret);
        return ret;
    }

    if(AM_HAL_STATUS_SUCCESS != am_hal_mspi_interrupt_enable(data->mspiHandle, AM_HAL_MSPI_INT_CQUPD |
                                                         AM_HAL_MSPI_INT_ERR) ) {
		ret = -ENODEV;
        LOG_INST_ERR(cfg->log, "%u, Fail to turn on interrupt, code:%d",
                     __LINE__, ret);
        return ret;
    }

    spi_context_unlock_unconditionally(&data->ctx);
	return 0;

}
#define AMBIQ_MSPI_DEFINE(n)																\
	PINCTRL_DT_INST_DEFINE(n);																\
	static int pwr_on_ambiq_mspi_##n(void)													\
	{																						\
		uint32_t addr = DT_REG_ADDR(DT_INST_PHANDLE(n, ambiq_pwrcfg)) +                    	\
				DT_INST_PHA(n, ambiq_pwrcfg, offset);                              			\
		sys_write32((sys_read32(addr) | DT_INST_PHA(n, ambiq_pwrcfg, mask)), addr);        	\
		k_busy_wait(PWRCTRL_MAX_WAIT_US);                                                 	\
		return 0;                                                                          	\
	} ;																						\
	static void mspi_irq_config_func_##n(void)												\
	{																						\
		IRQ_CONNECT(DT_INST_IRQN(n),														\
				DT_INST_IRQ(n, priority),													\
				mspi_ambiq_isr,																	\
				DEVICE_DT_INST_GET(n), 0);													\
		irq_enable(DT_INST_IRQN(n));														\
	};																						\
	static struct mspi_ambiq_data mspi_ambiq_data##n = {									\
		SPI_CONTEXT_INIT_LOCK(mspi_ambiq_data##n, ctx),										\
		SPI_CONTEXT_INIT_SYNC(mspi_ambiq_data##n, ctx)										\
	};																						\
	static const struct mspi_ambiq_config mspi_ambiq_config##n = {							\
		.base = DT_INST_REG_ADDR(n),														\
		.size = DT_INST_REG_SIZE(n),														\
		.chip_sel = DT_INST_PROP_OR(n, chip_select, 0),										\
		.cs_gpio = GPIO_DT_SPEC_INST_GET_BY_IDX(n, cs_gpios, 0),							\
		.irq_config_func = mspi_irq_config_func_##n,											\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),										\
		.pwr_func = pwr_on_ambiq_mspi_##n,													\
	};																						\
	DEVICE_DT_INST_DEFINE(n, mspi_ambiq_init, NULL, &mspi_ambiq_data##n,					\
			      &mspi_ambiq_config##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,        		\
			      &mspi_ambiq_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_MSPI_DEFINE)

