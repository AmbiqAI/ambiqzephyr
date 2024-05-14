/*
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_spi

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_ambiq);

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/device_runtime.h>

#include <stdlib.h>
#include <errno.h>
#include "spi_context.h"
#include <am_mcu_apollo.h>

#define PWRCTRL_MAX_WAIT_US 5

typedef int (*ambiq_spi_pwr_func_t)(void);

struct spi_ambiq_config {
	uint32_t base;
	int size;
	uint32_t clock_freq;
	const struct pinctrl_dev_config *pcfg;
	ambiq_spi_pwr_func_t pwr_func;
	void (*irq_config_func)(void);
};

struct spi_ambiq_data {
	struct spi_context ctx;
	am_hal_iom_config_t iom_cfg;
	void *IOMHandle;
	uint32_t *pDMATCBBuffer;
};

#define SPI_BASE (((const struct spi_ambiq_config *)(dev)->config)->base)
#if defined(CONFIG_SOC_SERIES_APOLLO3X)
#define REG_STAT 0x2B4
#else
#define REG_STAT 0x248
#endif
#define IDLE_STAT     0x4
#define SPI_STAT(dev) (SPI_BASE + REG_STAT)
#define SPI_WORD_SIZE 8

#define SPI_CS_INDEX 3

#ifdef CONFIG_SPI_AMBIQ_DMA
static void pfnSPI_Callback(void *pCallbackCtxt, uint32_t status)
{
	const struct device *dev = pCallbackCtxt;
	struct spi_ambiq_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	spi_context_complete(ctx, dev, 0);
}
#endif

static void spi_ambiq_isr(const struct device *dev)
{
	uint32_t ui32Status;
	struct spi_ambiq_data *data = dev->data;

	am_hal_iom_interrupt_status_get(data->IOMHandle, false, &ui32Status);
	am_hal_iom_interrupt_clear(data->IOMHandle, ui32Status);
	am_hal_iom_interrupt_service(data->IOMHandle, ui32Status);
}

static int spi_config(const struct device *dev, const struct spi_config *config)
{
	struct spi_ambiq_data *data = dev->data;
	const struct spi_ambiq_config *cfg = dev->config;
	struct spi_context *ctx = &(data->ctx);

	data->iom_cfg.eInterfaceMode = AM_HAL_IOM_SPI_MODE;

	int ret = 0;

	if (spi_context_configured(ctx, config)) {
		/* Already configured. No need to do it again. */
		return 0;
	}

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		LOG_ERR("Word size must be %d", SPI_WORD_SIZE);
		return -ENOTSUP;
	}

	if ((config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("Only supports single mode");
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
			data->iom_cfg.eSpiMode = AM_HAL_IOM_SPI_MODE_3;
		} else {
			data->iom_cfg.eSpiMode = AM_HAL_IOM_SPI_MODE_2;
		}
	} else {
		if (config->operation & SPI_MODE_CPHA) {
			data->iom_cfg.eSpiMode = AM_HAL_IOM_SPI_MODE_1;
		} else {
			data->iom_cfg.eSpiMode = AM_HAL_IOM_SPI_MODE_0;
		}
	}

	if (config->operation & SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode not supported");
		return -ENOTSUP;
	}
	if (config->operation & SPI_MODE_LOOP) {
		LOG_ERR("Loopback mode not supported");
		return -ENOTSUP;
	}

	if (cfg->clock_freq > AM_HAL_IOM_MAX_FREQ) {
		LOG_ERR("Clock frequency too high");
		return -ENOTSUP;
	}

	data->iom_cfg.ui32ClockFreq = cfg->clock_freq;
	ctx->config = config;

#ifdef CONFIG_SPI_AMBIQ_DMA
	data->iom_cfg.pNBTxnBuf = data->pDMATCBBuffer;
	data->iom_cfg.ui32NBTxnBufLength = CONFIG_SPI_DMA_TCB_BUFFER_SIZE;
#endif

	/* Disable IOM instance as it cannot be configured when enabled*/
	ret = am_hal_iom_disable(data->IOMHandle);

	ret = am_hal_iom_configure(data->IOMHandle, &data->iom_cfg);

	ret = am_hal_iom_enable(data->IOMHandle);

	return ret;
}

static int spi_ambiq_xfer(const struct device *dev, const struct spi_config *config,
			  const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	struct spi_ambiq_data *data = dev->data;
	const struct spi_ambiq_config *cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	int ret = 0;
	static bool bPreContinue;
	bool bContinue = (config->operation & SPI_HOLD_ON_CS) ? true : false;

	am_hal_iom_transfer_t trans = {0};
#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	uint32_t iom_nce = cfg->pcfg->states->pins[SPI_CS_INDEX].iom_nce;
#else
	uint32_t iom_nce = cfg->pcfg->states->pins[SPI_CS_INDEX].iom_nce % 4;
#endif

	/* There's data to send */
	if (spi_context_tx_on(ctx)) {
		/* There's data to Receive */
		if (spi_context_rx_on(ctx)) {
			/* Only need instruction in the first packet */
			if (!bPreContinue) {
				/*
				 * The instruction length can only be:
				 *  0~AM_HAL_IOM_MAX_OFFSETSIZE.
				 */
				trans.ui32InstrLen = (ctx->tx_len <= AM_HAL_IOM_MAX_OFFSETSIZE)
							     ? ctx->tx_len
							     : AM_HAL_IOM_MAX_OFFSETSIZE;
				for (int i = 0; i < trans.ui32InstrLen; i++) {
#if defined(CONFIG_SOC_SERIES_APOLLO3X)
					trans.ui32Instr = (trans.ui32Instr << 8) | (*ctx->tx_buf);
#else
					trans.ui64Instr = (trans.ui64Instr << 8) | (*ctx->tx_buf);
#endif
					spi_context_update_tx(ctx, 1, 1);
				}
				/* Skip the cmd buffer for rx. */
				if (rx_bufs->count > 1) {
					spi_context_update_rx(ctx, 1, rx_bufs->buffers[0].len);
				}
			}
			if (!(config->operation & SPI_HALF_DUPLEX)) {
				uint8_t *tx_dummy = NULL;
				tx_dummy = malloc(ctx->rx_len);
				if (tx_dummy == NULL) {
					spi_context_complete(ctx, dev, 0);
					return -ENOMEM;
				}
				memset(tx_dummy, 0, ctx->rx_len);
				trans.eDirection = AM_HAL_IOM_FULLDUPLEX;
				trans.bContinue = true;
				trans.pui32RxBuffer = (uint32_t *)ctx->rx_buf;
				trans.pui32TxBuffer = (uint32_t *)tx_dummy;
				trans.ui32NumBytes = ctx->rx_len;
				trans.uPeerInfo.ui32SpiChipSelect = iom_nce;
				ret = am_hal_iom_spi_blocking_fullduplex(data->IOMHandle, &trans);
				free((void *)tx_dummy);
				spi_context_complete(ctx, dev, 0);
			} else {
				/* Set RX direction and receive data. */
				trans.eDirection = AM_HAL_IOM_RX;
				trans.bContinue = bContinue;
				trans.pui32RxBuffer = (uint32_t *)ctx->rx_buf;
				trans.ui32NumBytes = ctx->rx_len;
				trans.uPeerInfo.ui32SpiChipSelect = iom_nce;
#ifdef CONFIG_SPI_AMBIQ_DMA
				if (AM_HAL_STATUS_SUCCESS !=
				    am_hal_iom_nonblocking_transfer(data->IOMHandle, &trans,
								    pfnSPI_Callback, (void *)dev)) {
					spi_context_complete(ctx, dev, 0);
					return -EIO;
				}

				ret = spi_context_wait_for_completion(ctx);
#else
				ret = am_hal_iom_blocking_transfer(data->IOMHandle, &trans);
#endif
			}
		} else { /* There's no data to Receive */
			bool bLast = false;
			uint32_t ui32RemNum, ui32CurNum = 0;
			for (size_t i = 0; i < tx_bufs->count; i++) {
				trans.eDirection = AM_HAL_IOM_TX;
				trans.uPeerInfo.ui32SpiChipSelect = iom_nce;
				ui32RemNum = ctx->tx_len;
				while (ui32RemNum) {
					ui32CurNum = (ui32RemNum > AM_HAL_IOM_MAX_TXNSIZE_SPI)
							     ? AM_HAL_IOM_MAX_TXNSIZE_SPI
							     : ui32RemNum;
					if ((i == (tx_bufs->count - 1)) &&
					    (ui32CurNum == ui32RemNum)) {
						bLast = true;
					}
					trans.bContinue = (bLast == true) ? bContinue : true;
					trans.ui32NumBytes = ui32CurNum;
					trans.pui32TxBuffer = (uint32_t *)ctx->tx_buf;
#ifdef CONFIG_SPI_AMBIQ_DMA
					if (AM_HAL_STATUS_SUCCESS !=
					    am_hal_iom_nonblocking_transfer(
						    data->IOMHandle, &trans,
						    ((bLast == true) ? pfnSPI_Callback : NULL),
						    (void *)dev)) {
						spi_context_complete(ctx, dev, 0);
						return -EIO;
					}
					if (bLast) {
						ret = spi_context_wait_for_completion(ctx);
					}
#else
					ret = am_hal_iom_blocking_transfer(data->IOMHandle, &trans);
					if (bLast) {
						spi_context_complete(ctx, dev, 0);
					}
#endif
					ui32RemNum -= ui32CurNum;
					spi_context_update_tx(ctx, 1, ui32CurNum);
				}
			}
		}
	} else { /* There's no data to send */
		/* Skip the cmd buffer for rx. */
		if (rx_bufs->count > 1) {
			spi_context_update_rx(ctx, 1, rx_bufs->buffers[0].len);
		}
		bool bLast = false;
		uint32_t ui32CurNum = 0;
		uint32_t ui32RemNum = ctx->rx_len;
		trans.eDirection = AM_HAL_IOM_RX;
		trans.uPeerInfo.ui32SpiChipSelect = iom_nce;
		while (ui32RemNum) {
			ui32CurNum = (ui32RemNum > AM_HAL_IOM_MAX_TXNSIZE_SPI)
					     ? AM_HAL_IOM_MAX_TXNSIZE_SPI
					     : ui32RemNum;
			if (ui32CurNum == ui32RemNum) {
				bLast = true;
			}
			/* Set RX direction to receive data and release CS after transmission. */
			trans.bContinue = (bLast == true) ? bContinue : true;
			trans.pui32RxBuffer = (uint32_t *)ctx->rx_buf;
			trans.ui32NumBytes = ui32CurNum;
#ifdef CONFIG_SPI_AMBIQ_DMA
			if (AM_HAL_STATUS_SUCCESS !=
			    am_hal_iom_nonblocking_transfer(
				    data->IOMHandle, &trans,
				    ((bLast == true) ? pfnSPI_Callback : NULL), (void *)dev)) {
				spi_context_complete(ctx, dev, 0);
				return -EIO;
			}
			if (bLast) {
				ret = spi_context_wait_for_completion(ctx);
			}
#else
			ret = am_hal_iom_blocking_transfer(data->IOMHandle, &trans);
			if (bLast) {
				spi_context_complete(ctx, dev, 0);
			}
#endif
			ui32RemNum -= ui32CurNum;
			spi_context_update_rx(ctx, 1, ui32CurNum);
		}
	}

	bPreContinue = bContinue;
	return ret;
}

static int spi_ambiq_transceive(const struct device *dev, const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	struct spi_ambiq_data *data = dev->data;
	int ret;

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}

#if defined(CONFIG_PM_DEVICE_RUNTIME)
	ret = pm_device_runtime_get(dev);

	if (ret < 0) {
		LOG_ERR("pm_device_runtime_get failed: %d", ret);
	}
#endif

	/* context setup */
	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	ret = spi_config(dev, config);

	if (ret) {
		goto end;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	ret = spi_ambiq_xfer(dev, config, tx_bufs, rx_bufs);

end:
	spi_context_release(&data->ctx, ret);

#if defined(CONFIG_PM_DEVICE_RUNTIME)
	/* Do not power off if need to hold the CS */
	if (!(config->operation & SPI_HOLD_ON_CS)) {
		/* Use async put to avoid useless device suspension/resumption
		 * when doing consecutive transmission.
		 */
		ret = pm_device_runtime_put_async(dev, K_MSEC(2));

		if (ret < 0) {
			LOG_ERR("pm_device_runtime_put failed: %d", ret);
		}
	}
#endif

	return ret;
}

static int spi_ambiq_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_ambiq_data *data = dev->data;

#ifdef CONFIG_SPI_AMBIQ_DMA
	free((void *)data->pDMATCBBuffer);
#endif
	if (!sys_read32(SPI_STAT(dev))) {
		return -EBUSY;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct spi_driver_api spi_ambiq_driver_api = {
	.transceive = spi_ambiq_transceive,
	.release = spi_ambiq_release,
};

static int spi_ambiq_init(const struct device *dev)
{
	struct spi_ambiq_data *data = dev->data;
	const struct spi_ambiq_config *cfg = dev->config;
	int ret = 0;
	void *buf = NULL;

	if (AM_HAL_STATUS_SUCCESS !=
	    am_hal_iom_initialize((cfg->base - REG_IOM_BASEADDR) / cfg->size, &data->IOMHandle)) {
		LOG_ERR("Fail to initialize SPI\n");
		return -ENXIO;
	}

	cfg->pwr_func();

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Fail to config SPI pins\n");
		goto end;
	}

#ifdef CONFIG_SPI_AMBIQ_DMA
	am_hal_iom_interrupt_clear(data->IOMHandle, AM_HAL_IOM_INT_CQUPD | AM_HAL_IOM_INT_ERR);
	am_hal_iom_interrupt_enable(data->IOMHandle, AM_HAL_IOM_INT_CQUPD | AM_HAL_IOM_INT_ERR);
	cfg->irq_config_func();

	buf = malloc(CONFIG_SPI_DMA_TCB_BUFFER_SIZE * 4);
	if (buf == NULL) {
		ret = -ENOMEM;
		goto end;
	}
	data->pDMATCBBuffer = (uint32_t *)buf;
#endif
end:
	if (ret < 0) {
		am_hal_iom_uninitialize(data->IOMHandle);
		if (buf != NULL) {
			free((void *)data->pDMATCBBuffer);
		}
	} else {
		spi_context_unlock_unconditionally(&data->ctx);
	}
	return ret;
}

#ifdef CONFIG_PM_DEVICE
static int spi_ambiq_pm_action(const struct device *dev, enum pm_device_action action)
{
	struct spi_ambiq_data *data = dev->data;
	uint32_t ret;
	am_hal_sysctrl_power_state_e status;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		status = AM_HAL_SYSCTRL_WAKE;
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		status = AM_HAL_SYSCTRL_DEEPSLEEP;
		break;
	default:
		return -ENOTSUP;
	}

	ret = am_hal_iom_power_ctrl(data->IOMHandle, status, true);

	if (ret != AM_HAL_STATUS_SUCCESS) {
		LOG_ERR("am_hal_iom_power_ctrl failed: %d", ret);
		return -EPERM;
	} else {
		return 0;
	}
}
#endif /* CONFIG_PM_DEVICE */

#define AMBIQ_SPI_INIT(n)                                                                          \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static int pwr_on_ambiq_spi_##n(void)                                                      \
	{                                                                                          \
		uint32_t addr = DT_REG_ADDR(DT_INST_PHANDLE(n, ambiq_pwrcfg)) +                    \
				DT_INST_PHA(n, ambiq_pwrcfg, offset);                              \
		sys_write32((sys_read32(addr) | DT_INST_PHA(n, ambiq_pwrcfg, mask)), addr);        \
		k_busy_wait(PWRCTRL_MAX_WAIT_US);                                                  \
		return 0;                                                                          \
	}                                                                                          \
	static void spi_irq_config_func_##n(void)                                                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), spi_ambiq_isr,              \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	};                                                                                         \
	static struct spi_ambiq_data spi_ambiq_data##n = {                                         \
		SPI_CONTEXT_INIT_LOCK(spi_ambiq_data##n, ctx),                                     \
		SPI_CONTEXT_INIT_SYNC(spi_ambiq_data##n, ctx)};                                    \
	static const struct spi_ambiq_config spi_ambiq_config##n = {                               \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.size = DT_INST_REG_SIZE(n),                                                       \
		.clock_freq = DT_INST_PROP(n, clock_frequency),                                    \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.irq_config_func = spi_irq_config_func_##n,                                        \
		.pwr_func = pwr_on_ambiq_spi_##n};                                                 \
	PM_DEVICE_DT_INST_DEFINE(n, spi_ambiq_pm_action);                                          \
	DEVICE_DT_INST_DEFINE(n, spi_ambiq_init, PM_DEVICE_DT_INST_GET(n), &spi_ambiq_data##n,     \
			      &spi_ambiq_config##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,         \
			      &spi_ambiq_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_SPI_INIT)
