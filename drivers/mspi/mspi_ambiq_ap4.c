/*
 * Copyright (c) 2024, Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_mspi_controller

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_instance.h>
LOG_MODULE_REGISTER(mspi_ambiq_ap4);
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys_clock.h>
#include <zephyr/irq.h>

#include "mspi_ambiq.h"

#define MSPI_MAX_FREQ        96000000
#define MSPI_MAX_DEVICE      2
#define MSPI_TIMEOUT_US      1000000
#define PWRCTRL_MAX_WAIT_US  5
#define MSPI_BUSY            BIT(2)

typedef int (*mspi_ambiq_pwr_func_t)(void);
typedef void (*irq_config_func_t)(void);

struct mspi_context {
	const struct mspi_dev_id      *owner;

	struct mspi_xfer              xfer;

	int                           packets_left;
	int                           packets_done;

	mspi_callback_handler_t       callback;
	struct mspi_callback_context  *callback_ctx;
	bool asynchronous;

	struct k_sem lock;
};

struct mspi_ambiq_config {
	uint32_t                        reg_base;
	uint32_t                        reg_size;

	struct mspi_cfg                 mspicfg;

	const struct pinctrl_dev_config *pcfg;
	irq_config_func_t               irq_cfg_func;

	LOG_INSTANCE_PTR_DECLARE(log);
};

struct mspi_ambiq_data {
	void                            *mspiHandle;
	am_hal_mspi_dev_config_t        hal_dev_cfg;
	am_hal_mspi_config_t            hal_config_cfg;
	am_hal_mspi_xip_config_t        hal_xip_cfg;
	am_hal_mspi_xip_misc_t hal_xip_misc_cfg;

	struct mspi_dev_id              *dev_id;
	struct k_mutex                  lock;

	struct mspi_dev_cfg             dev_cfg;
	struct mspi_xip_cfg             xip_cfg;
	struct mspi_scramble_cfg        scramble_cfg;

	mspi_callback_handler_t         cbs[MSPI_BUS_EVENT_MAX];
	struct mspi_callback_context    *cb_ctxs[MSPI_BUS_EVENT_MAX];

	struct mspi_context             ctx;
};

static int mspi_set_freq(const struct mspi_ambiq_config *cfg, uint32_t freq)
{
	uint32_t d = MSPI_MAX_FREQ / freq;

	switch (d) {
	case AM_HAL_MSPI_CLK_96MHZ:
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
		LOG_INST_ERR(cfg->log, "%u,Frequency not supported!", __LINE__);
		d = 0;
		break;
	}

	return d;
}

static am_hal_mspi_device_e mspi_set_line(const struct mspi_ambiq_config *cfg,
					  enum mspi_io_mode eIOMode,
					  enum mspi_data_rate eDataRate,
					  uint32_t ui32CENum)
{
	if (eDataRate != MSPI_SINGLE_DATA_RATE) {
		LOG_INST_ERR(cfg->log, "%u, incorrect data rate, only SDR is supported.", __LINE__);
		return AM_HAL_MSPI_FLASH_MAX;
	}

	if (ui32CENum == 0) {
		switch (eIOMode) {
		case MSPI_IO_MODE_SINGLE:
			return AM_HAL_MSPI_FLASH_SERIAL_CE0;
		case MSPI_IO_MODE_DUAL:
			return AM_HAL_MSPI_FLASH_DUAL_CE0;
		case MSPI_IO_MODE_DUAL_1_1_2:
			return AM_HAL_MSPI_FLASH_DUAL_CE0_1_1_2;
		case MSPI_IO_MODE_DUAL_1_2_2:
			return AM_HAL_MSPI_FLASH_DUAL_CE0_1_2_2;
		case MSPI_IO_MODE_QUAD:
			return AM_HAL_MSPI_FLASH_QUAD_CE0;
		case MSPI_IO_MODE_QUAD_1_1_4:
			return AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4;
		case MSPI_IO_MODE_QUAD_1_4_4:
			return AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4;
		case MSPI_IO_MODE_OCTAL:
			return AM_HAL_MSPI_FLASH_OCTAL_CE0;
		default:
			return AM_HAL_MSPI_FLASH_MAX;
		}
	} else if (ui32CENum == 1) {
		switch (eIOMode) {
		case MSPI_IO_MODE_SINGLE:
			return AM_HAL_MSPI_FLASH_SERIAL_CE1;
		case MSPI_IO_MODE_DUAL:
			return AM_HAL_MSPI_FLASH_DUAL_CE1;
		case MSPI_IO_MODE_DUAL_1_1_2:
			return AM_HAL_MSPI_FLASH_DUAL_CE1_1_1_2;
		case MSPI_IO_MODE_DUAL_1_2_2:
			return AM_HAL_MSPI_FLASH_DUAL_CE1_1_2_2;
		case MSPI_IO_MODE_QUAD:
			return AM_HAL_MSPI_FLASH_QUAD_CE1;
		case MSPI_IO_MODE_QUAD_1_1_4:
			return AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4;
		case MSPI_IO_MODE_QUAD_1_4_4:
			return AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4;
		case MSPI_IO_MODE_OCTAL:
			return AM_HAL_MSPI_FLASH_OCTAL_CE1;
		default:
			return AM_HAL_MSPI_FLASH_MAX;
		}
	} else {
		return AM_HAL_MSPI_FLASH_MAX;
	}
}

static inline void mspi_context_ce_control(struct mspi_context *ctx, bool on)
{
	if (ctx->owner) {
		if (ctx->xfer.bHoldCE &&
		    ctx->xfer.sCE.gpio.port != NULL) {
			if (on) {
				gpio_pin_set_dt(&ctx->xfer.sCE.gpio, 1);
				k_busy_wait(ctx->xfer.sCE.delay);
			} else {
				k_busy_wait(ctx->xfer.sCE.delay);
				gpio_pin_set_dt(&ctx->xfer.sCE.gpio, 0);
			}
		}
	}
}

static inline void mspi_context_release(struct mspi_context *ctx)
{
	ctx->owner = NULL;
	k_sem_give(&ctx->lock);
}

static inline void mspi_context_unlock_unconditionally(struct mspi_context *ctx)
{
	mspi_context_ce_control(ctx, false);

	if (!k_sem_count_get(&ctx->lock)) {
		ctx->owner = NULL;
		k_sem_give(&ctx->lock);
	}
}

static inline int mspi_context_lock(struct mspi_context *ctx,
				    const struct mspi_dev_id *req,
				    const struct mspi_xfer *xfer,
				    mspi_callback_handler_t callback,
				    struct mspi_callback_context *callback_ctx,
				    bool lockon)
{
	int ret = 1;

	if ((k_sem_count_get(&ctx->lock) == 0) && !lockon &&
	    (ctx->owner == req)) {
		return 0;
	}

	k_sem_take(&ctx->lock, K_FOREVER);
	if (ctx->xfer.bAsync) {
		if ((xfer->ui32TXDummy == ctx->xfer.ui32TXDummy) &&
		    (xfer->ui32RXDummy == ctx->xfer.ui32RXDummy) &&
		    (xfer->ui16InstrLength == ctx->xfer.ui16InstrLength) &&
		    (xfer->ui16AddrLength == ctx->xfer.ui16AddrLength)) {
			ret = 0;
		} else if (ctx->packets_left == 0) {
			if (ctx->callback_ctx) {
				struct mspi_event_data *evt_data;
				evt_data = &ctx->callback_ctx->mspi_evt.evt_data;
				while (evt_data->status != 0) {
				}
				ret = 1;
			} else {
				ret = 0;
			}
		} else {
			return -EIO;
		}
	}
	ctx->owner           = req;
	ctx->xfer            = *xfer;
	ctx->packets_done    = 0;
	ctx->packets_left    = ctx->xfer.ui32NumPacket;
	ctx->callback        = callback;
	ctx->callback_ctx    = callback_ctx;
	return ret;
}

static inline bool mspi_is_inp(const struct device *controller)
{
	struct mspi_ambiq_data *data = controller->data;

	return (k_sem_count_get(&data->ctx.lock) == 0);
}

static inline int mspi_verify_device(const struct device *controller,
				     const struct mspi_dev_id *dev_id)
{
	const struct mspi_ambiq_config *cfg = controller->config;
	int device_index = cfg->mspicfg.ui32SlaveNum;
	int ret = 0;

	for (int i = 0; i < cfg->mspicfg.ui32SlaveNum; i++) {
		if (dev_id->ce.port == cfg->mspicfg.pCE[i].port &&
		    dev_id->ce.pin == cfg->mspicfg.pCE[i].pin &&
		    dev_id->ce.dt_flags == cfg->mspicfg.pCE[i].dt_flags) {
			device_index = i;
		}
	}

	if (device_index >= cfg->mspicfg.ui32SlaveNum ||
	    device_index != dev_id->dev_idx) {
		LOG_INST_ERR(cfg->log, "%u, invalid device ID.", __LINE__);
		return -ENODEV;
	}

	return ret;
}

static int mspi_ambiq_deinit(const struct device *controller)
{
	const struct mspi_ambiq_config *cfg = controller->config;
	struct mspi_ambiq_data *data = controller->data;
	int ret = 0;

	if (!data->mspiHandle) {
		LOG_INST_ERR(cfg->log, "%u, the mspi not yet initialized.", __LINE__);
		return -ENODEV;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = am_hal_mspi_interrupt_disable(data->mspiHandle, 0xFFFFFFFF);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to disable interrupt, code:%d.",
			     __LINE__, ret);
		ret = -EHOSTDOWN;
		goto e_deinit_return;
	}

	ret = am_hal_mspi_interrupt_clear(data->mspiHandle, 0xFFFFFFFF);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to clear interrupt, code:%d.",
			     __LINE__, ret);
		ret = -EHOSTDOWN;
		goto e_deinit_return;
	}

	ret = am_hal_mspi_disable(data->mspiHandle);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to disable MSPI, code:%d.",
			     __LINE__, ret);
		ret = -EHOSTDOWN;
		goto e_deinit_return;
	}

	ret = am_hal_mspi_power_control(data->mspiHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to power off MSPI, code:%d.",
			     __LINE__, ret);
		ret = -EHOSTDOWN;
		goto e_deinit_return;
	}

	ret = am_hal_mspi_deinitialize(&data->mspiHandle);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to deinit MSPI.", __LINE__);
		ret = -ENODEV;
		goto e_deinit_return;
	}
	return ret;

e_deinit_return:
	k_mutex_unlock(&data->lock);
	return ret;
}

/** DMA specific config */
static int mspi_xfer_config(const struct device *controller,
			    const struct mspi_xfer *xfer)
{
	const struct mspi_ambiq_config *cfg = controller->config;
	struct mspi_ambiq_data *data = controller->data;
	am_hal_mspi_dev_config_t hal_dev_cfg = data->hal_dev_cfg;
	am_hal_mspi_request_e eRequest;
	int ret = 0;

	if (data->scramble_cfg.bEnable) {
		eRequest = AM_HAL_MSPI_REQ_SCRAMB_EN;
	} else {
		eRequest = AM_HAL_MSPI_REQ_SCRAMB_DIS;
	}

	ret = am_hal_mspi_disable(data->mspiHandle);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to disable MSPI, code:%d.",
			     __LINE__, ret);
		return -EHOSTDOWN;
	}

	ret = am_hal_mspi_control(data->mspiHandle, eRequest, NULL);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u,Unable to complete scramble config:%d.",
			     __LINE__, data->scramble_cfg.bEnable);
		return -EHOSTDOWN;
	}

	if (xfer->ui16InstrLength > AM_HAL_MSPI_INSTR_2_BYTE + 1) {
		LOG_INST_ERR(cfg->log, "%u, ui16InstrLength is too large.", __LINE__);
		return -ENOTSUP;
	}
	if (xfer->ui16InstrLength == 0) {
		hal_dev_cfg.bSendInstr = false;
	} else {
		hal_dev_cfg.bSendInstr = true;
		hal_dev_cfg.eInstrCfg = xfer->ui16InstrLength - 1;
	}

	if (xfer->ui16AddrLength > AM_HAL_MSPI_ADDR_4_BYTE + 1) {
		LOG_INST_ERR(cfg->log, "%u, ui16AddrLength is too large.", __LINE__);
		return -ENOTSUP;
	}
	if (xfer->ui16AddrLength == 0) {
		hal_dev_cfg.bSendAddr = false;
	} else {
		hal_dev_cfg.bSendAddr = true;
		hal_dev_cfg.eAddrCfg = xfer->ui16AddrLength - 1;
	}

	hal_dev_cfg.bTurnaround     = (xfer->ui32RXDummy != 0);
	hal_dev_cfg.ui8TurnAround   = (uint8_t)xfer->ui32RXDummy;
	hal_dev_cfg.bEnWriteLatency = (xfer->ui32TXDummy != 0);
	hal_dev_cfg.ui8WriteLatency = (uint8_t)xfer->ui32TXDummy;

	ret = am_hal_mspi_device_configure(data->mspiHandle, &hal_dev_cfg);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to configure MSPI, code:%d.",
			     __LINE__, ret);
		return -EHOSTDOWN;
	}

	ret = am_hal_mspi_enable(data->mspiHandle);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to enable MSPI, code:%d.",
			     __LINE__, ret);
		return -EHOSTDOWN;
	}

	data->hal_dev_cfg = hal_dev_cfg;
	return ret;
}

static int mspi_ambiq_config(const struct mspi_dt_spec *spec)
{
	const struct mspi_cfg *config = &spec->config;
	const struct mspi_ambiq_config *cfg = spec->bus->config;
	struct mspi_ambiq_data *data = spec->bus->data;
	am_hal_mspi_config_t hal_config_cfg = data->hal_config_cfg;

	int ret = 0;

	if (config->eOPMode != MSPI_OP_MODE_MASTER) {
		LOG_INST_ERR(cfg->log, "%u, only support MSPI master.", __LINE__);
		return -ENOTSUP;
	}

	if (config->ui32MaxFreq > MSPI_MAX_FREQ) {
		LOG_INST_ERR(cfg->log, "%u, ui32MaxFreq too large.", __LINE__);
		return -ENOTSUP;
	}

	if (config->eDuplex != MSPI_HALF_DUPLEX) {
		LOG_INST_ERR(cfg->log, "%u, only support half duplex mode.", __LINE__);
		return -ENOTSUP;
	}

	if (config->bDQS) {
		LOG_INST_ERR(cfg->log, "%u, only support non-DQS mode.", __LINE__);
		return -ENOTSUP;
	}

	if (config->bReinit) {
		ret = mspi_ambiq_deinit(spec->bus);
		if (ret) {
			return ret;
		}
	}

	ret = am_hal_mspi_initialize(config->ui8MSPIChannel, &data->mspiHandle);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to initialize MSPI, code:%d.",
			     __LINE__, ret);
		return -EPERM;
	}

	ret = am_hal_mspi_power_control(data->mspiHandle, AM_HAL_SYSCTRL_WAKE, false);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to power on MSPI, code:%d.",
			     __LINE__, ret);
		return -EHOSTDOWN;
	}

	ret = am_hal_mspi_configure(data->mspiHandle, &hal_config_cfg);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to Enable MSPI, code:%d.",
			     __LINE__, ret);
		return -EHOSTDOWN;
	}

	ret = am_hal_mspi_enable(data->mspiHandle);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to Enable MSPI, code:%d.",
			     __LINE__, ret);
		return -EHOSTDOWN;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

	ret = am_hal_mspi_interrupt_clear(data->mspiHandle, AM_HAL_MSPI_INT_CQUPD |
							    AM_HAL_MSPI_INT_ERR);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to clear interrupt, code:%d.",
			__LINE__, ret);
		return -EHOSTDOWN;
	}

	ret = am_hal_mspi_interrupt_enable(data->mspiHandle, AM_HAL_MSPI_INT_CQUPD |
							     AM_HAL_MSPI_INT_ERR);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to turn on interrupt, code:%d.",
			     __LINE__, ret);
		return -EHOSTDOWN;
	}

	cfg->irq_cfg_func();

	mspi_context_unlock_unconditionally(&data->ctx);

	if (config->bReinit) {
		k_mutex_unlock(&data->lock);
	}

	return ret;
}

static int mspi_ambiq_dev_config(const struct device *controller,
				 const struct mspi_dev_id *dev_id,
				 const enum mspi_dev_cfg_mask param_mask,
				 const struct mspi_dev_cfg *dev_cfg)
{
	const struct mspi_ambiq_config *cfg = controller->config;
	struct mspi_ambiq_data *data = controller->data;
	am_hal_mspi_dev_config_t hal_dev_cfg = data->hal_dev_cfg;
	int ret = 0;

	if (data->dev_id != dev_id) {
		k_mutex_lock(&data->lock, K_FOREVER);

		ret = mspi_verify_device(controller, dev_id);
		if (ret) {
			goto e_return;
		}
	}

	if (mspi_is_inp(controller)) {
		ret = -EBUSY;
		goto e_return;
	}

	if (param_mask != MSPI_DEVICE_CONFIG_ALL) {
		if (data->dev_id != dev_id) {
			LOG_INST_ERR(cfg->log, "%u, config failed, must be the same device.",
				     __LINE__);
			ret = -ENOTSUP;
			goto e_return;
		}

		if ((param_mask & (~(MSPI_DEVICE_CONFIG_FREQUENCY |
				     MSPI_DEVICE_CONFIG_IO_MODE |
				     MSPI_DEVICE_CONFIG_CE_NUM |
				     MSPI_DEVICE_CONFIG_DATA_RATE |
				     MSPI_DEVICE_CONFIG_INSTR_LEN |
				     MSPI_DEVICE_CONFIG_ADDR_LEN)))) {
			LOG_INST_ERR(cfg->log, "%u, config type not supported.", __LINE__);
			ret = -ENOTSUP;
			goto e_return;
		}

		if (param_mask & MSPI_DEVICE_CONFIG_FREQUENCY) {
			hal_dev_cfg.eClockFreq = mspi_set_freq(cfg, dev_cfg->ui32Freq);
			if (hal_dev_cfg.eClockFreq == 0) {
				ret = -ENOTSUP;
				goto e_return;
			}
			ret = am_hal_mspi_control(data->mspiHandle,
						  AM_HAL_MSPI_REQ_CLOCK_CONFIG,
						  &hal_dev_cfg.eClockFreq);
			if (ret) {
				LOG_INST_ERR(cfg->log, "%u, failed to configure eClockFreq.",
					     __LINE__);
				ret = -EHOSTDOWN;
				goto e_return;
			}
			data->dev_cfg.ui32Freq = dev_cfg->ui32Freq;
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_IO_MODE) ||
		    (param_mask & MSPI_DEVICE_CONFIG_CE_NUM) ||
		    (param_mask & MSPI_DEVICE_CONFIG_DATA_RATE)) {
			hal_dev_cfg.eDeviceConfig = mspi_set_line(cfg, dev_cfg->eIOMode,
								  dev_cfg->eDataRate,
								  dev_cfg->ui32CENum);
			if (hal_dev_cfg.eDeviceConfig == AM_HAL_MSPI_FLASH_MAX) {
				ret = -ENOTSUP;
				goto e_return;
			}
			ret = am_hal_mspi_control(data->mspiHandle,
						  AM_HAL_MSPI_REQ_DEVICE_CONFIG,
						  &hal_dev_cfg.eDeviceConfig);
			if (ret) {
				LOG_INST_ERR(cfg->log, "%u, failed to configure device.", __LINE__);
				ret = -EHOSTDOWN;
				goto e_return;
			}
			data->dev_cfg.ui32Freq = dev_cfg->eIOMode;
			data->dev_cfg.eDataRate = dev_cfg->eDataRate;
			data->dev_cfg.ui32CENum = dev_cfg->ui32CENum;
		}

		if (param_mask & MSPI_DEVICE_CONFIG_INSTR_LEN) {
			if (dev_cfg->ui16InstrLength > AM_HAL_MSPI_INSTR_2_BYTE + 1 ||
			    dev_cfg->ui16InstrLength == 0) {
				LOG_INST_ERR(cfg->log, "%u, invalid ui16InstrLength.", __LINE__);
				ret = -ENOTSUP;
				goto e_return;
			}
			hal_dev_cfg.eInstrCfg = dev_cfg->ui16InstrLength - 1;
			// ret = am_hal_mspi_control(data->mspiHandle,
			// 			  AM_HAL_MSPI_REQ_ISIZE_SET,
			// 			  &hal_dev_cfg.eInstrCfg);
			if (ret) {
				LOG_INST_ERR(cfg->log, "%u, failed to configure ui16InstrLength.",
					     __LINE__);
				ret = -EHOSTDOWN;
				goto e_return;
			}
			data->dev_cfg.ui16InstrLength = dev_cfg->ui16InstrLength;
		}

		if (param_mask & MSPI_DEVICE_CONFIG_ADDR_LEN) {
			if (dev_cfg->ui16AddrLength > AM_HAL_MSPI_ADDR_4_BYTE + 1 ||
			    dev_cfg->ui16AddrLength == 0) {
				LOG_INST_ERR(cfg->log, "%u, invalid ui16AddrLength.", __LINE__);
				ret = -ENOTSUP;
				goto e_return;
			}
			hal_dev_cfg.eAddrCfg = dev_cfg->ui16AddrLength - 1;
			// ret = am_hal_mspi_control(data->mspiHandle,
			// 			AM_HAL_MSPI_REQ_ASIZE_SET,
			// 			&hal_dev_cfg.eAddrCfg);
			if (ret) {
				LOG_INST_ERR(cfg->log, "%u, failed to configure ui16AddrLength.",
					     __LINE__);
				ret = -EHOSTDOWN;
				goto e_return;
			}
			data->dev_cfg.ui16AddrLength = dev_cfg->ui16AddrLength;
		}

	} else {
		if (data->dev_id != dev_id) {
			ret = pinctrl_apply_state(cfg->pcfg,
						  PINCTRL_STATE_PRIV_START + dev_id->dev_idx);
			if (ret) {
				goto e_return;
			}
		}

		if (dev_cfg->eEndian != MSPI_XFER_LITTLE_ENDIAN) {
			LOG_INST_ERR(cfg->log, "%u, only support MSB first.", __LINE__);
			ret = -ENOTSUP;
			goto e_return;
		}

		if (dev_cfg->bDQSEnable && !cfg->mspicfg.bDQS) {
			LOG_INST_ERR(cfg->log, "%u, only support non-DQS mode.", __LINE__);
			ret = -ENOTSUP;
			goto e_return;
		}

		hal_dev_cfg.eSpiMode = dev_cfg->eCPP;

		hal_dev_cfg.bEnWriteLatency = (dev_cfg->ui32TXDummy != 0);
		hal_dev_cfg.ui8WriteLatency = dev_cfg->ui32TXDummy;
		hal_dev_cfg.bTurnaround = (dev_cfg->ui32RXDummy != 0);
		hal_dev_cfg.ui8TurnAround = dev_cfg->ui32RXDummy;

		hal_dev_cfg.eClockFreq = mspi_set_freq(cfg, dev_cfg->ui32Freq);
		if (hal_dev_cfg.eClockFreq == 0) {
			ret = -ENOTSUP;
			goto e_return;
		}

		hal_dev_cfg.eDeviceConfig = mspi_set_line(cfg, dev_cfg->eIOMode, dev_cfg->eDataRate,
							  dev_cfg->ui32CENum);
		if (hal_dev_cfg.eDeviceConfig == AM_HAL_MSPI_FLASH_MAX) {
			ret = -ENOTSUP;
			goto e_return;
		}

		if (dev_cfg->ui16InstrLength > AM_HAL_MSPI_INSTR_2_BYTE + 1) {
			LOG_INST_ERR(cfg->log, "%u, ui16InstrLength too large.", __LINE__);
			ret = -ENOTSUP;
			goto e_return;
		}
		if (dev_cfg->ui16InstrLength == 0) {
			hal_dev_cfg.bSendInstr = false;
		} else {
			hal_dev_cfg.bSendInstr = true;
			hal_dev_cfg.eInstrCfg = dev_cfg->ui16InstrLength - 1;
		}

		if (dev_cfg->ui16AddrLength > AM_HAL_MSPI_ADDR_4_BYTE + 1) {
			LOG_INST_ERR(cfg->log, "%u, ui16AddrLength too large.", __LINE__);
			ret = -ENOTSUP;
			goto e_return;
		}
		if (dev_cfg->ui16AddrLength == 0) {
			hal_dev_cfg.bSendAddr = false;
		} else {
			hal_dev_cfg.bSendAddr = true;
			hal_dev_cfg.eAddrCfg = dev_cfg->ui16AddrLength - 1;
		}

		hal_dev_cfg.ui16ReadInstr = (uint16_t)dev_cfg->ui32ReadInstr;
		hal_dev_cfg.ui16WriteInstr = (uint16_t)dev_cfg->ui32WriteInstr;

		if (dev_cfg->ui32MemBoundary > AM_HAL_MSPI_BOUNDARY_MAX) {
			LOG_INST_ERR(cfg->log, "%u, ui32MemBoundary too large.", __LINE__);
			ret = -ENOTSUP;
			goto e_return;
		}
		hal_dev_cfg.eDMABoundary = dev_cfg->ui32MemBoundary;

		hal_dev_cfg.ui16DMATimeLimit = dev_cfg->ui32BreakTimeLimit;

		ret = am_hal_mspi_disable(data->mspiHandle);
		if (ret) {
			LOG_INST_ERR(cfg->log, "%u, fail to disable MSPI, code:%d.", __LINE__, ret);
			ret = -EHOSTDOWN;
			goto e_return;
		}

		ret = am_hal_mspi_device_configure(data->mspiHandle, &hal_dev_cfg);
		if (ret) {
			LOG_INST_ERR(cfg->log, "%u, fail to configure MSPI, code:%d.", __LINE__,
				     ret);
			ret = -EHOSTDOWN;
			goto e_return;
		}

		ret = am_hal_mspi_enable(data->mspiHandle);
		if (ret) {
			LOG_INST_ERR(cfg->log, "%u, fail to enable MSPI, code:%d.", __LINE__, ret);
			ret = -EHOSTDOWN;
			goto e_return;
		}
		data->dev_cfg = *dev_cfg;
		data->dev_id = (struct mspi_dev_id *)dev_id;
	}
	data->hal_dev_cfg = hal_dev_cfg;

	return ret;

e_return:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int mspi_ambiq_xip_config(const struct device *controller,
				 const struct mspi_dev_id *dev_id,
				 const struct mspi_xip_cfg *xip_cfg)
{
	const struct mspi_ambiq_config *cfg = controller->config;
	struct mspi_ambiq_data *data = controller->data;
	am_hal_mspi_request_e eRequest;
	int ret = 0;

	if (dev_id != data->dev_id) {
		LOG_INST_ERR(cfg->log, "%u, dev_id don't match.", __LINE__);
		return -ESTALE;
	}

	if (xip_cfg->bEnable) {
		eRequest = AM_HAL_MSPI_REQ_XIP_EN;
	} else {
		eRequest = AM_HAL_MSPI_REQ_XIP_DIS;
	}

	ret = am_hal_mspi_control(data->mspiHandle, eRequest, NULL);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u,Unable to complete xip config:%d.", __LINE__,
			     xip_cfg->bEnable);
		return -EHOSTDOWN;
	}

	data->xip_cfg = *xip_cfg;
	return ret;
}

static int mspi_ambiq_scramble_config(const struct device *controller,
				      const struct mspi_dev_id *dev_id,
				      const struct mspi_scramble_cfg *scramble_cfg)
{
	const struct mspi_ambiq_config *cfg = controller->config;
	struct mspi_ambiq_data *data = controller->data;
	am_hal_mspi_xip_config_t hal_xip_cfg = data->hal_xip_cfg;
	am_hal_mspi_request_e eRequest;
	int ret = 0;

	if (mspi_is_inp(controller)) {
		return -EBUSY;
	}

	if (dev_id != data->dev_id) {
		LOG_INST_ERR(cfg->log, "%u, dev_id don't match.", __LINE__);
		return -ESTALE;
	}

	if (scramble_cfg->bEnable) {
		eRequest = AM_HAL_MSPI_REQ_SCRAMB_EN;
	} else {
		eRequest = AM_HAL_MSPI_REQ_SCRAMB_DIS;
	}

	ret = am_hal_mspi_disable(data->mspiHandle);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to disable MSPI, code:%d.", __LINE__, ret);
		return -EHOSTDOWN;
	}

	ret = am_hal_mspi_control(data->mspiHandle, eRequest, NULL);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u,Unable to complete scramble config:%d.", __LINE__,
			     scramble_cfg->bEnable);
		return -EHOSTDOWN;
	}

	hal_xip_cfg.scramblingStartAddr = 0 + scramble_cfg->ui32AddrOffset;
	hal_xip_cfg.scramblingEndAddr = hal_xip_cfg.scramblingStartAddr + scramble_cfg->ui32Size;

	ret = am_hal_mspi_control(data->mspiHandle, AM_HAL_MSPI_REQ_XIP_CONFIG, &hal_xip_cfg);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to configure MSPI, code:%d.", __LINE__, ret);
		return -EHOSTDOWN;
	}

	ret = am_hal_mspi_enable(data->mspiHandle);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to enable MSPI, code:%d.", __LINE__, ret);
		return -EHOSTDOWN;
	}

	data->scramble_cfg = *scramble_cfg;
	return ret;
}

static int mspi_ambiq_timing_config(const struct device *controller,
				    const struct mspi_dev_id *dev_id,
				    const uint32_t param_mask,
				    void *timing_cfg)
{
	const struct mspi_ambiq_config *cfg = controller->config;
	struct mspi_ambiq_data *data = controller->data;
	am_hal_mspi_dev_config_t hal_dev_cfg = data->hal_dev_cfg;
	struct mspi_ambiq_timing_cfg *time_cfg = timing_cfg;
	am_hal_mspi_timing_scan_t timing;
	am_hal_mspi_dev_config_t config;
	int ret = 0;

	if (mspi_is_inp(controller)) {
		return -EBUSY;
	}

	if (dev_id != data->dev_id) {
		LOG_INST_ERR(cfg->log, "%u, dev_id don't match.", __LINE__);
		return -ESTALE;
	}

	if (param_mask & (~(MSPI_AMBIQ_SET_WLC | MSPI_AMBIQ_SET_RLC))) {
		LOG_INST_ERR(cfg->log, "%u, config type not supported.", __LINE__);
		return -ENOTSUP;
	}

	if (param_mask & MSPI_AMBIQ_SET_WLC) {
		if (time_cfg->ui8WriteLatency) {
			config.bEnWriteLatency = true;
		} else {
			config.bEnWriteLatency = false;
		}
		config.ui8WriteLatency = time_cfg->ui8WriteLatency;
	}

	ret = am_hal_mspi_control(data->mspiHandle, AM_HAL_MSPI_REQ_NAND_FLASH_SET_WLAT, &config);

	if (param_mask & MSPI_AMBIQ_SET_RLC) {
		if (time_cfg->ui8TurnAround) {
			hal_dev_cfg.bTurnaround = true;
		} else {
			hal_dev_cfg.bTurnaround = false;
		}
		hal_dev_cfg.ui8TurnAround = time_cfg->ui8TurnAround;
	}

	timing.ui8Turnaround = hal_dev_cfg.ui8TurnAround;

	ret = am_hal_mspi_control(data->mspiHandle, AM_HAL_MSPI_REQ_TIMING_SCAN, &timing);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, fail to configure timing.", __LINE__);
		return -EHOSTDOWN;
	}

	data->hal_dev_cfg = hal_dev_cfg;
	return ret;
}

static int mspi_ambiq_get_channel_status(const struct device *controller, uint8_t ch)
{
	const struct mspi_ambiq_config *cfg = controller->config;
	struct mspi_ambiq_data *data = controller->data;
	int ret = 0;

	if (sys_read32(cfg->reg_base) & MSPI_BUSY) {
		ret = -EBUSY;
	}

	if (mspi_is_inp(controller)) {
		return -EBUSY;
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static void mspi_ambiq_isr(const struct device *dev)
{
	struct mspi_ambiq_data *data = dev->data;
	uint32_t ui32Status;
	am_hal_mspi_interrupt_status_get(data->mspiHandle, &ui32Status, false);
	am_hal_mspi_interrupt_clear(data->mspiHandle, ui32Status);
	am_hal_mspi_interrupt_service(data->mspiHandle, ui32Status);
}

/** Manage sync dma transceive */
static void hal_mspi_callback(void *pCallbackCtxt, uint32_t status)
{
	const struct device *controller = pCallbackCtxt;
	struct mspi_ambiq_data *data = controller->data;
	data->ctx.packets_done++;
}

static int mspi_pio_prepare(const struct device *controller,
			    am_hal_mspi_pio_transfer_t *trans)
{
	const struct mspi_ambiq_config *cfg = controller->config;
	struct mspi_ambiq_data *data = controller->data;
	const struct mspi_xfer *xfer = &data->ctx.xfer;
	int ret = 0;

	trans->bScrambling    = false;
	trans->bSendAddr      = (xfer->ui16AddrLength != 0);
	trans->bSendInstr     = (xfer->ui16InstrLength != 0);
	trans->bTurnaround    = (xfer->ui32RXDummy != 0);
	trans->bEnWRLatency   = (xfer->ui32TXDummy != 0);
	trans->bDCX           = false;
	//trans->bQuadCmd       = false;
	trans->bContinue      = false;

	if (xfer->ui16InstrLength > AM_HAL_MSPI_INSTR_2_BYTE + 1) {
		LOG_INST_ERR(cfg->log, "%u, invalid ui16InstrLength.", __LINE__);
		return -ENOTSUP;
	}
	if (xfer->ui16InstrLength != 0) {
		am_hal_mspi_instr_e eInstrCfg = xfer->ui16InstrLength - 1;
		//ret = am_hal_mspi_control(data->mspiHandle, AM_HAL_MSPI_REQ_ISIZE_SET, &eInstrCfg);
		if (ret) {
			LOG_INST_ERR(cfg->log, "%u, failed to configure ui16InstrLength.",
				     __LINE__);
			return -EHOSTDOWN;
		}
		data->hal_dev_cfg.eInstrCfg = eInstrCfg;
	}
	data->dev_cfg.ui16InstrLength = xfer->ui16InstrLength;

	if (xfer->ui16AddrLength > AM_HAL_MSPI_ADDR_4_BYTE + 1) {
		LOG_INST_ERR(cfg->log, "%u, invalid ui16AddrLength.", __LINE__);
		return -ENOTSUP;
	}
	if (xfer->ui16AddrLength != 0) {
		am_hal_mspi_addr_e eAddrCfg = xfer->ui16AddrLength - 1;
		//ret = am_hal_mspi_control(data->mspiHandle, AM_HAL_MSPI_REQ_ASIZE_SET, &eAddrCfg);
		if (ret) {
			LOG_INST_ERR(cfg->log, "%u, failed to configure ui16AddrLength.", __LINE__);
			return -EHOSTDOWN;
		}
		data->hal_dev_cfg.eAddrCfg = eAddrCfg;
	}
	data->dev_cfg.ui16AddrLength = xfer->ui16AddrLength;

	return ret;
}

static int mspi_pio_transceive(const struct device *controller,
			       const struct mspi_xfer *xfer,
			       mspi_callback_handler_t cb,
			       struct mspi_callback_context *cb_ctx)
{
	const struct mspi_ambiq_config *cfg = controller->config;
	struct mspi_ambiq_data *data = controller->data;
	struct mspi_context *ctx = &data->ctx;
	const struct mspi_xfer_packet *packet;
	uint32_t packet_idx;
	am_hal_mspi_pio_transfer_t trans;
	int ret = 0;
	int cfg_flag = 0;

	if (xfer->ui32NumPacket == 0 || !xfer->pPacket) {
		return -EFAULT;
	}

	cfg_flag = mspi_context_lock(ctx, data->dev_id, xfer, cb, cb_ctx, true);

	/** user must make sure when cfg_flag = 0 the dummy and instr addr lengh)
	 * in mspi_xfer of the two calls are the same if the first one has not finished yet.
	 */
	if (cfg_flag) {
		if (cfg_flag == 1) {
			ret = mspi_pio_prepare(controller, &trans);
			if (ret) {
				goto pio_err;
			}
		} else {
			ret = cfg_flag;
			goto pio_err;
		}
	}

	if (!ctx->xfer.bAsync) {

		while (ctx->packets_left > 0) {
			packet_idx = ctx->xfer.ui32NumPacket - ctx->packets_left;
			packet = &ctx->xfer.pPacket[packet_idx];
			trans.eDirection = packet->eDirection;
			trans.ui16DeviceInstr = packet->ui16DeviceInstr;
			trans.ui32DeviceAddr = packet->ui32DeviceAddr;
			trans.ui32NumBytes = packet->ui32NumBytes;
			trans.pui32Buffer = packet->pui32Buffer;

			ret = am_hal_mspi_blocking_transfer(data->mspiHandle, &trans,
							    MSPI_TIMEOUT_US);
			ctx->packets_left--;
			if (ret) {
				ret = -EIO;
				goto pio_err;
			}
		}

	} else {

		ret = am_hal_mspi_interrupt_enable(data->mspiHandle, AM_HAL_MSPI_INT_DMACMP);
		if (ret) {
			LOG_INST_ERR(cfg->log, "%u, failed to enable interrupt.", __LINE__);
			ret = -EHOSTDOWN;
			goto pio_err;
		}

		while (ctx->packets_left > 0) {
			packet_idx = ctx->xfer.ui32NumPacket - ctx->packets_left;
			packet = &ctx->xfer.pPacket[packet_idx];
			trans.eDirection = packet->eDirection;
			trans.ui16DeviceInstr = packet->ui16DeviceInstr;
			trans.ui32DeviceAddr = packet->ui32DeviceAddr;
			trans.ui32NumBytes = packet->ui32NumBytes;
			trans.pui32Buffer = packet->pui32Buffer;

			if (ctx->callback && packet->eCBMask == MSPI_BUS_XFER_COMPLETE_CB) {
				ctx->callback_ctx->mspi_evt.evt_type = MSPI_BUS_XFER_COMPLETE;
				ctx->callback_ctx->mspi_evt.evt_data.controller = controller;
				ctx->callback_ctx->mspi_evt.evt_data.dev_id = data->ctx.owner;
				ctx->callback_ctx->mspi_evt.evt_data.packet = packet;
				ctx->callback_ctx->mspi_evt.evt_data.packet_idx = packet_idx;
				ctx->callback_ctx->mspi_evt.evt_data.status = ~0;
			}

			am_hal_mspi_callback_t callback = NULL;
			if (packet->eCBMask == MSPI_BUS_XFER_COMPLETE_CB) {
				callback = (am_hal_mspi_callback_t)ctx->callback;
			}

			ret = am_hal_mspi_nonblocking_transfer(data->mspiHandle, &trans, MSPI_PIO,
							       callback, (void *)ctx->callback_ctx);
			ctx->packets_left--;
			if (ret) {
				if (ret == AM_HAL_STATUS_OUT_OF_RANGE) {
					ret = -ENOMEM;
				} else {
					ret = -EIO;
				}
				goto pio_err;
			}
		}
	}

pio_err:
	mspi_context_release(ctx);
	return ret;
}

static int mspi_dma_transceive(const struct device *controller,
			       const struct mspi_xfer *xfer,
			       mspi_callback_handler_t cb,
			       struct mspi_callback_context *cb_ctx)
{
	const struct mspi_ambiq_config *cfg = controller->config;
	struct mspi_ambiq_data *data = controller->data;
	struct mspi_context *ctx = &data->ctx;
	am_hal_mspi_dma_transfer_t trans;
	int ret = 0;
	int cfg_flag = 0;

	if (xfer->ui32NumPacket == 0 || !xfer->pPacket) {
		return -EFAULT;
	}

	cfg_flag = mspi_context_lock(ctx, data->dev_id, xfer, cb, cb_ctx, true);
	/** user must make sure when cfg_flag = 0 the dummy and instr addr lengh)
	 * in mspi_xfer of the two calls are the same if the first one has not finished yet.
	 */
	if (cfg_flag) {
		if (cfg_flag == 1) {
			ret = mspi_xfer_config(controller, xfer);
			if (ret) {
				goto dma_err;
			}
		} else {
			ret = cfg_flag;
			goto dma_err;
		}
	}

	ret = am_hal_mspi_interrupt_enable(data->mspiHandle, AM_HAL_MSPI_INT_DMACMP);
	if (ret) {
		LOG_INST_ERR(cfg->log, "%u, failed to enable interrupt.", __LINE__);
		ret = -EHOSTDOWN;
		goto dma_err;
	}

	while (ctx->packets_left > 0) {
		uint32_t packet_idx = ctx->xfer.ui32NumPacket - ctx->packets_left;
		const struct mspi_xfer_packet *packet;

		packet = &ctx->xfer.pPacket[packet_idx];
		trans.ui8Priority = ctx->xfer.ui8Priority;
		trans.eDirection = packet->eDirection;
		trans.ui32TransferCount = packet->ui32NumBytes;
		trans.ui32DeviceAddress = packet->ui32DeviceAddr;
		trans.ui32SRAMAddress = (uint32_t)packet->pui32Buffer;
		trans.ui32PauseCondition = 0;
		trans.ui32StatusSetClr = 0;

		if (ctx->xfer.bAsync) {

			if (ctx->callback && packet->eCBMask == MSPI_BUS_XFER_COMPLETE_CB) {
				ctx->callback_ctx->mspi_evt.evt_type = MSPI_BUS_XFER_COMPLETE;
				ctx->callback_ctx->mspi_evt.evt_data.controller = controller;
				ctx->callback_ctx->mspi_evt.evt_data.dev_id = data->ctx.owner;
				ctx->callback_ctx->mspi_evt.evt_data.packet = packet;
				ctx->callback_ctx->mspi_evt.evt_data.packet_idx = packet_idx;
				ctx->callback_ctx->mspi_evt.evt_data.status = ~0;
			}

			am_hal_mspi_callback_t callback = NULL;
			if (packet->eCBMask == MSPI_BUS_XFER_COMPLETE_CB) {
				callback = (am_hal_mspi_callback_t)ctx->callback;
			}

			ret = am_hal_mspi_nonblocking_transfer(data->mspiHandle, &trans, MSPI_DMA,
							       callback, (void *)ctx->callback_ctx);
		} else {
			ret = am_hal_mspi_nonblocking_transfer(data->mspiHandle, &trans, MSPI_DMA,
							       hal_mspi_callback,
							       (void *)controller);
		}
		ctx->packets_left--;
		if (ret) {
			if (ret == AM_HAL_STATUS_OUT_OF_RANGE) {
				ret = -ENOMEM;
			} else {
				ret = -EIO;
			}
			goto dma_err;
		}
	}

	if (!ctx->xfer.bAsync) {
		while (ctx->packets_done < ctx->xfer.ui32NumPacket) {
			k_busy_wait(10);
		}
	}

dma_err:
	mspi_context_release(ctx);
	return ret;
}

static int mspi_ambiq_transceive(const struct device *controller,
				 const struct mspi_dev_id *dev_id,
				 const struct mspi_xfer *xfer)
{
	const struct mspi_ambiq_config *cfg = controller->config;
	struct mspi_ambiq_data *data = controller->data;
	mspi_callback_handler_t cb = NULL;
	struct mspi_callback_context *cb_ctx = NULL;

	if (dev_id != data->dev_id) {
		LOG_INST_ERR(cfg->log, "%u, dev_id don't match.", __LINE__);
		return -ESTALE;
	}

	if (xfer->bAsync) {
		cb = data->cbs[MSPI_BUS_XFER_COMPLETE];
		cb_ctx = data->cb_ctxs[MSPI_BUS_XFER_COMPLETE];
	}

	if (xfer->eMode == MSPI_PIO) {
		return mspi_pio_transceive(controller, xfer, cb, cb_ctx);
	} else if (xfer->eMode == MSPI_DMA) {
		return mspi_dma_transceive(controller, xfer, cb, cb_ctx);
	} else {
		return -EIO;
	}
}

static int mspi_ambiq_register_callback(const struct device *controller,
					const struct mspi_dev_id *dev_id,
					const enum mspi_bus_event evt_type,
					mspi_callback_handler_t cb,
					struct mspi_callback_context *ctx)
{
	const struct mspi_ambiq_config *cfg = controller->config;
	struct mspi_ambiq_data *data = controller->data;

	if (mspi_is_inp(controller)) {
		return -EBUSY;
	}

	if (dev_id != data->dev_id) {
		LOG_INST_ERR(cfg->log, "%u, dev_id don't match.", __LINE__);
		return -ESTALE;
	}

	if (evt_type != MSPI_BUS_XFER_COMPLETE) {
		LOG_INST_ERR(cfg->log, "%u, callback types not supported.", __LINE__);
		return -ENOTSUP;
	}

	data->cbs[evt_type] = cb;
	data->cb_ctxs[evt_type] = ctx;
	return 0;
}

#if CONFIG_PM_DEVICE
static int mspi_ambiq_pm_action(const struct device *controller, enum pm_device_action action)
{
	const struct mspi_ambiq_config *cfg = controller->config;
	struct mspi_ambiq_data *data = controller->data;
	int ret = 0;

	if (mspi_is_inp(controller)) {
		return -EBUSY;
	}

	switch (action) {
	case PM_DEVICE_ACTION_TURN_ON:
		ret = am_hal_mspi_power_control(data->mspiHandle, AM_HAL_SYSCTRL_WAKE, true);
		if (ret) {
			LOG_INST_ERR(cfg->log, "%u, fail to power on MSPI, code:%d.", __LINE__,
				     ret);
			return -EHOSTDOWN;
		}
		break;

	case PM_DEVICE_ACTION_TURN_OFF:
		ret = am_hal_mspi_power_control(data->mspiHandle, AM_HAL_SYSCTRL_DEEPSLEEP, true);
		if (ret) {
			LOG_INST_ERR(cfg->log, "%u, fail to power off MSPI, code:%d.", __LINE__,
				     ret);
			return -EHOSTDOWN;
		}
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif

static int mspi_ambiq_init(const struct device *controller)
{
	const struct mspi_ambiq_config *cfg = controller->config;
	const struct mspi_dt_spec spec = {
		.bus = controller,
		.config = cfg->mspicfg,
	};

	return mspi_ambiq_config(&spec);
}

static struct mspi_driver_api mspi_ambiq_driver_api = {
	.config                = mspi_ambiq_config,
	.dev_config            = mspi_ambiq_dev_config,
	.xip_config            = mspi_ambiq_xip_config,
	.scramble_config       = mspi_ambiq_scramble_config,
	.timing_config         = mspi_ambiq_timing_config,
	.get_channel_status    = mspi_ambiq_get_channel_status,
	.register_callback     = mspi_ambiq_register_callback,
	.transceive            = mspi_ambiq_transceive,
};

#define _CNT(n)         1 +
#define DT_NUM_CHILD(n) (DT_FOREACH_CHILD(n, _CNT) 0)

#define MSPI_PINCTRL_STATE_INIT(state_idx, node_id)                                      \
	COND_CODE_1(Z_PINCTRL_SKIP_STATE(state_idx, node_id), (),                            \
		({                                                                               \
			.id = state_idx,                                                         \
			.pins = Z_PINCTRL_STATE_PINS_NAME(state_idx, node_id),                   \
			.pin_cnt = ARRAY_SIZE(Z_PINCTRL_STATE_PINS_NAME(state_idx, node_id))     \
		}))

#define MSPI_PINCTRL_STATES_DEFINE(node_id)                                              \
	static const struct pinctrl_state                                                    \
	Z_PINCTRL_STATES_NAME(node_id)[] = {                                                 \
		LISTIFY(DT_NUM_PINCTRL_STATES(node_id),                                          \
			MSPI_PINCTRL_STATE_INIT, (,), node_id)                                   \
	};

#define MSPI_PINCTRL_DT_DEFINE(node_id)                                                  \
		LISTIFY(DT_NUM_PINCTRL_STATES(node_id),                                          \
			Z_PINCTRL_STATE_PINS_DEFINE, (;), node_id);                              \
		MSPI_PINCTRL_STATES_DEFINE(node_id)                                              \
		Z_PINCTRL_DEV_CONFIG_STATIC Z_PINCTRL_DEV_CONFIG_CONST                           \
			struct pinctrl_dev_config Z_PINCTRL_DEV_CONFIG_NAME(node_id) =           \
				Z_PINCTRL_DEV_CONFIG_INIT(node_id)

#define MSPI_CONFIG(n)                                                                   \
	{                                                                                    \
		.ui8MSPIChannel      = (DT_INST_REG_ADDR(n) - REG_MSPI_BASEADDR) /               \
					(DT_INST_REG_SIZE(n) * 4),                               \
		.eOPMode             = MSPI_OP_MODE_MASTER,                                      \
		.eDuplex             = MSPI_HALF_DUPLEX,                                         \
		.ui32MaxFreq         = MSPI_MAX_FREQ,                                            \
		.bDQS                = false,                                                    \
	}

#define MSPI_HAL_DEVICE_CONFIG(n)                                                        \
	{                                                                                    \
		.ui8TurnAround         = 0,                                                      \
		.eAddrCfg              = 0,                                                      \
		.eInstrCfg             = 0,                                                      \
		.ui16ReadInstr         = 0,                                                      \
		.ui16WriteInstr        = 0,                                                      \
		.eDeviceConfig         = AM_HAL_MSPI_FLASH_SERIAL_CE0,                           \
		.ui8WriteLatency       = 0,                                                      \
		.eSpiMode              = AM_HAL_MSPI_SPI_MODE_0,                                 \
		.eClockFreq            = MSPI_MAX_FREQ / DT_INST_PROP_OR(n,                      \
									 clock_frequency,        \
									 MSPI_MAX_FREQ),         \
		.bEnWriteLatency       = false,                                                  \
		.bSendAddr             = false,                                                  \
		.bSendInstr            = false,                                                  \
		.bTurnaround           = false,                                                  \
		.bEmulateDDR           = false,                                                  \
		.ui16DMATimeLimit      = 0,                                                      \
		.eDMABoundary          = AM_HAL_MSPI_BOUNDARY_NONE,                              \
	}

#define MSPI_HAL_CFG_CONFIG(n, cmdq, cmdq_size)                                         \
	{                                                                                   \
		.ui32TCBSize = cmdq_size,                                                       \
		.pTCB        = cmdq,                                                            \
		.bClkonD4    = 0,                                                               \
	}

#define MSPI_HAL_XIP_CONFIG(n)                                                          \
	{                                                                                   \
		.scramblingStartAddr = 0,                                                       \
		.scramblingEndAddr   = 0,                                                       \
		.ui32APBaseAddr      = 0x14000000 + n*0x4000000,                                \
		.eAPMode             = AM_HAL_MSPI_AP_READ_WRITE,                               \
		.eAPSize             = AM_HAL_MSPI_AP_SIZE64M,                                  \
	}

#define MSPI_HAL_XIP_MISC_CONFIG(n)                                                     \
	{                                                                                   \
		.ui32CEBreak        = 10,                                                       \
		.bXIPBoundary       = true,                                                     \
		.bXIPOdd            = true,                                                     \
		.bAppndOdd          = false,                                                    \
		.bBEOn              = false,                                                    \
		.eBEPolarity        = AM_HAL_MSPI_BE_LOW_ENABLE,                                \
	}

#define AMBIQ_MSPI_DEFINE(n)                                                            \
	LOG_INSTANCE_REGISTER(DT_DRV_INST(n), mspi##n, CONFIG_MSPI_LOG_LEVEL);              \
	MSPI_PINCTRL_DT_DEFINE(DT_DRV_INST(n));                                             \
	static void mspi_ambiq_irq_cfg_func_##n(void)                                       \
	{                                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),                          \
		mspi_ambiq_isr, DEVICE_DT_INST_GET(n), 0);                                      \
		irq_enable(DT_INST_IRQN(n));                                                    \
	}                                                                                   \
	static uint32_t mspi_ambiq_cmdq##n[DT_INST_PROP_OR(n, cmdq_buffer_size, 1024) / 4]  \
	__attribute__((section(DT_INST_PROP_OR(n, cmdq_buffer_location, ".mspi_buff"))));   \
	static struct gpio_dt_spec ce_gpios##n[] = MSPI_CE_GPIOS_DT_SPEC_INST_GET(n);       \
	static struct mspi_ambiq_data mspi_ambiq_data##n = {                                \
		.mspiHandle            = NULL,                                                  \
		.hal_config_cfg        = MSPI_HAL_CFG_CONFIG(n, mspi_ambiq_cmdq##n,             \
					 DT_INST_PROP_OR(n, cmdq_buffer_size, 1024)), \
		.hal_dev_cfg           = MSPI_HAL_DEVICE_CONFIG(n),                             \
		.hal_xip_cfg           = MSPI_HAL_XIP_CONFIG(n),                                \
		.hal_xip_misc_cfg      = MSPI_HAL_XIP_MISC_CONFIG(n),                           \
		.dev_id                = 0,                                                     \
		.lock                  = Z_MUTEX_INITIALIZER(mspi_ambiq_data##n.lock),          \
		.dev_cfg               = {0},                                                   \
		.xip_cfg               = {0},                                                   \
		.scramble_cfg          = {0},                                                   \
		.cbs                   = {0},                                                   \
		.cb_ctxs               = {0},                                                   \
		.ctx.lock              = Z_SEM_INITIALIZER(mspi_ambiq_data##n.ctx.lock, 0, 1),  \
		.ctx.callback          = 0,                                                     \
		.ctx.callback_ctx      = 0,                                                     \
	};                                                                                  \
	static const struct mspi_ambiq_config mspi_ambiq_config##n = {                      \
		.reg_base              = DT_INST_REG_ADDR(n),                                   \
		.reg_size              = DT_INST_REG_SIZE(n),                                   \
		.mspicfg               = MSPI_CONFIG(n),                                        \
		.mspicfg.pCE           = (struct gpio_dt_spec *)ce_gpios##n,                    \
		.mspicfg.ui32CeGpioNum = ARRAY_SIZE(ce_gpios##n),                               \
		.mspicfg.ui32SlaveNum  = DT_NUM_CHILD(DT_DRV_INST(n)),                          \
		.mspicfg.bReinit       = false,                                                 \
		.pcfg                  = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                     \
		.irq_cfg_func          = mspi_ambiq_irq_cfg_func_##n,                           \
		LOG_INSTANCE_PTR_INIT(log, DT_DRV_INST(n), mspi##n)                             \
	};                                                                                  \
	PM_DEVICE_DT_INST_DEFINE(n, mspi_ambiq_pm_action);                                  \
	DEVICE_DT_INST_DEFINE(n,                                                            \
			      mspi_ambiq_init,                                                   \
			      PM_DEVICE_DT_INST_GET(n),                                          \
			      &mspi_ambiq_data##n,                                               \
			      &mspi_ambiq_config##n,                                             \
			      POST_KERNEL,                                                       \
			      CONFIG_MSPI_INIT_PRIORITY,                                         \
			      &mspi_ambiq_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_MSPI_DEFINE)
