/*
 * Copyright (c) 2024 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
// #include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"
#include <am_mcu_apollo.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_ambiq, CONFIG_ADC_LOG_LEVEL);

#define DT_DRV_COMPAT ambiq_adc

typedef int (*ambiq_adc_pwr_func_t)(void);
#define PWRCTRL_MAX_WAIT_US       5
#define AMBIQ_DEFAULT_SLOT_NUMBER 1

struct adc_ambiq_config {
	/* adc controller base address */
	uint32_t base;
	int size;
	// uint8_t instance;
	/** Number of supported channels */
	uint8_t num_channels;

	// uint8_t callback_select;
	// Adc_Sar_Ip_ConfigType *adc_cfg;
	// void (*irq_config_func)(const struct device *dev);
	/* routine for configuring ADC's ISR */
	void (*irq_config_func)(void);
	const struct pinctrl_dev_config *pin_cfg;
	ambiq_adc_pwr_func_t pwr_func;
};

struct adc_ambiq_data {
	/* ADC Device pointer used in api functions */
	// const struct device *dev;
	/* mutex of ADC channels */
	struct adc_context ctx;
	void *adcHandle;
	uint16_t *buffer;
	// uint16_t *buf_end;
	uint16_t *repeat_buffer;
	// uint32_t mask_channels;
	// uint8_t num_channels;
};

static int adc_ambiq_set_resolution(am_hal_adc_slot_prec_e *prec, uint8_t adc_resolution)
{
	switch (adc_resolution) {
	case 8:
		*prec = AM_HAL_ADC_SLOT_8BIT;
		break;
	case 10:
		*prec = AM_HAL_ADC_SLOT_10BIT;
		break;
	case 12:
		*prec = AM_HAL_ADC_SLOT_12BIT;
		break;
	case 14:
		*prec = AM_HAL_ADC_SLOT_14BIT;
		break;
	default:
		LOG_ERR("Unsupported resolution");
		return -ENOTSUP;
	}

	return 0;
}

/**
 * Interrupt handler
 */
static void adc_ambiq_isr(const struct device *dev)
{
	struct adc_ambiq_data *data = dev->data;
	uint32_t ui32IntStatus;
	uint32_t ui32IntMask;
	uint32_t ui32NumSamples;
	am_hal_adc_sample_t Sample;

	// Read the interrupt status.
	am_hal_adc_interrupt_status(data->adcHandle, &ui32IntMask, true);
	// Clear the ADC interrupt.
	am_hal_adc_interrupt_clear(data->adcHandle, ui32IntMask);
	//
	// If we got a conversion completion interrupt (which should be our only
	// ADC interrupt), go ahead and read the data.
	//
	if (ui32IntMask & AM_HAL_ADC_INT_CNVCMP) {
		//
		// Read the value from the FIFO.
		//
		ui32NumSamples = 1;
		am_hal_adc_samples_read(data->adcHandle, false, NULL, &ui32NumSamples, &Sample);
		*data->buffer++ = Sample.ui32Sample;
		adc_context_on_sampling_done(&data->ctx, dev);
	}
}

static int adc_ambiq_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_ambiq_data *data = dev->data;
	const struct adc_ambiq_config *cfg = dev->config;
	am_hal_adc_slot_config_t ADCSlotConfig;
	int error = 0;
	uint8_t select_channel = 0;
	if (sequence->channels & ~BIT_MASK(cfg->num_channels)) {
		LOG_ERR("Incorrect channels, bitmask 0x%x", sequence->channels);
		return -EINVAL;
	}

	if (sequence->channels == 0UL) {
		LOG_ERR("No channel selected");
		return -EINVAL;
	}

	select_channel = find_lsb_set(sequence->channels) - 1;

	if (adc_ambiq_set_resolution(&ADCSlotConfig.ePrecisionMode, sequence->resolution) != 0) {
		LOG_ERR("unsupported resolution %d", sequence->resolution);
		return -ENOTSUP;
	}
	/* Set up an ADC slot */
	ADCSlotConfig.eMeasToAvg = AM_HAL_ADC_SLOT_AVG_1;
	ADCSlotConfig.eChannel = (am_hal_adc_slot_chan_e)select_channel;
	ADCSlotConfig.bWindowCompare = false;
	ADCSlotConfig.bEnabled = true;
	// #TODO slotmumber = 0, the actual value should be according to the relationship between
	// the channel and pin numbers
	if (AM_HAL_STATUS_SUCCESS !=
	    am_hal_adc_configure_slot(data->adcHandle, AMBIQ_DEFAULT_SLOT_NUMBER, &ADCSlotConfig)) {
		LOG_ERR("configuring ADC Slot 0 failed.\n");
		return -ENOTSUP;
	}

	/* Enable the ADC. */
	am_hal_adc_enable(data->adcHandle);

	data->buffer = sequence->buffer;
	/* Start ADC conversion */
	adc_context_start_read(&data->ctx, sequence);
	error = adc_context_wait_for_completion(&data->ctx);

	return error;
}

static int adc_ambiq_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_ambiq_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, false, NULL);
	error = adc_ambiq_start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}

static int adc_ambiq_channel_setup(const struct device *dev, const struct adc_channel_cfg *chan_cfg)
{
	const struct adc_ambiq_config *cfg = dev->config;

	if (chan_cfg->channel_id >= cfg->num_channels) {
		LOG_ERR("unsupported channel id '%d'", chan_cfg->channel_id);
		return -ENOTSUP;
	}

	if (chan_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Gain is not valid");
		return -ENOTSUP;
	}

	// #TODO check if only support internal reference
	if (chan_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Reference is not valid");
		return -ENOTSUP;
	}

	if (chan_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("unsupported acquisition_time '%d'", chan_cfg->acquisition_time);
		return -ENOTSUP;
	}
	// #TODO AP3 support differential, ap4 do not support, is it necessary to support this
	// feature?
	if (chan_cfg->differential) {
		LOG_ERR("Differential sampling not supported");
		return -ENOTSUP;
	}

	return 0;
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_ambiq_data *data = CONTAINER_OF(ctx, struct adc_ambiq_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_ambiq_data *data = CONTAINER_OF(ctx, struct adc_ambiq_data, ctx);
	// const struct device *dev = data->dev;
	// const struct adc_mspi_config *cfg = dev->config;

	data->repeat_buffer = data->buffer;
	/*Trigger the ADC*/
	am_hal_adc_sw_trigger(data->adcHandle);
}

static int adc_ambiq_init(const struct device *dev)
{
	struct adc_ambiq_data *data = dev->data;
	const struct adc_ambiq_config *cfg = dev->config;
	static void *pAdcHandle;
	am_hal_adc_config_t ADCConfig;

	int ret;

	/* Initialize the ADC and get the handle*/
	if (AM_HAL_STATUS_SUCCESS !=
	    am_hal_adc_initialize((cfg->base - REG_ADC_BASEADDR) / (cfg->size * 4), &pAdcHandle)) {
		ret = -ENODEV;
		LOG_ERR("Faile to initialize ADC, code:%d", ret);
		return ret;
	}
	data->adcHandle = pAdcHandle;

	/* power on ADC*/
	ret = cfg->pwr_func();
	// am_hal_adc_power_control(data->adcHandle, AM_HAL_SYSCTRL_WAKE,false);

	/* Set up the ADC configuration parameters. These settings are reasonable
	 *  for accurate measurements at a low sample rate.
	 */
	ADCConfig.eClock = AM_HAL_ADC_CLKSEL_HFRC;
	ADCConfig.ePolarity = AM_HAL_ADC_TRIGPOL_RISING;
	ADCConfig.eTrigger = AM_HAL_ADC_TRIGSEL_SOFTWARE;
	ADCConfig.eReference = AM_HAL_ADC_REFSEL_INT_1P5;
	ADCConfig.eClockMode = AM_HAL_ADC_CLKMODE_LOW_POWER;
	ADCConfig.ePowerMode = AM_HAL_ADC_LPMODE0;
	ADCConfig.eRepeat = AM_HAL_ADC_SINGLE_SCAN; // #TODO need check the repeat mode
	if (AM_HAL_STATUS_SUCCESS != am_hal_adc_configure(data->adcHandle, &ADCConfig)) {
		ret = -ENODEV;
		LOG_ERR("Configuring ADC failed, code:%d", ret);
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* Enable the ADC interrupts in the ADC. */
	cfg->irq_config_func();
	am_hal_adc_interrupt_enable(data->adcHandle, AM_HAL_ADC_INT_CNVCMP);

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

/* reference voltage for the ADC */
#define ADC_AMBIQ_DRIVER_API(n)                                                                    \
	static const struct adc_driver_api adc_ambiq_driver_api_##n = {                            \
		.channel_setup = adc_ambiq_channel_setup,                                          \
		.read = adc_ambiq_read,                                                            \
		.ref_internal = DT_INST_PROP(n, vref_mv),                                          \
	};

#define ADC_AMBIQ_INIT(n)                                                                          \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	ADC_AMBIQ_DRIVER_API(n);                                                                   \
	static int pwr_on_ambiq_adc_##n(void)                                                      \
	{                                                                                          \
		uint32_t addr = DT_REG_ADDR(DT_INST_PHANDLE(n, ambiq_pwrcfg)) +                    \
				DT_INST_PHA(n, ambiq_pwrcfg, offset);                              \
		sys_write32((sys_read32(addr) | DT_INST_PHA(n, ambiq_pwrcfg, mask)), addr);        \
		k_busy_wait(PWRCTRL_MAX_WAIT_US);                                                  \
		return 0;                                                                          \
	}                                                                                          \
	static void adc_irq_config_func_##n(void)                                                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), adc_ambiq_isr,              \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	};                                                                                         \
	static struct adc_ambiq_data adc_ambiq_data_##n = {                                        \
		ADC_CONTEXT_INIT_TIMER(adc_ambiq_data_##n, ctx),                                   \
		ADC_CONTEXT_INIT_LOCK(adc_ambiq_data_##n, ctx),                                    \
		ADC_CONTEXT_INIT_SYNC(adc_ambiq_data_##n, ctx),                                    \
	};                                                                                         \
	const static struct adc_ambiq_config adc_ambiq_config_##n = {                              \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.size = DT_INST_REG_SIZE(n),                                                       \
		.num_channels = DT_PROP(DT_DRV_INST(n), channel_count),                            \
		.irq_config_func = adc_irq_config_func_##n,                                        \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                      \
		.pwr_func = pwr_on_ambiq_adc_##n,                                                  \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, &adc_ambiq_init, NULL, &adc_ambiq_data_##n,                       \
			      &adc_ambiq_config_##n, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,        \
			      &adc_ambiq_driver_api_##n);

DT_INST_FOREACH_STATUS_OKAY(ADC_AMBIQ_INIT)