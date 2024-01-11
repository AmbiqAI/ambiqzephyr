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

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_ambiq, CONFIG_ADC_LOG_LEVEL);

#define DT_DRV_COMPAT ambiq_adc

struct adc_ambiq_config {
	uint32_t base;
	// uint8_t instance;
	// uint8_t group_channel;
	// uint8_t callback_select;
	// Adc_Sar_Ip_ConfigType *adc_cfg;
	// void (*irq_config_func)(const struct device *dev);
    // void (*irq_config_func)(void);
	const struct pinctrl_dev_config *pin_cfg;
	// ambiq_mspi_pwr_func_t pwr_func;

};

struct adc_ambiq_data {
	const struct device *dev;
	struct adc_context ctx;
	// uint16_t *buffer;
	// uint16_t *buf_end;
	// uint16_t *repeat_buffer;
	// uint32_t mask_channels;
	// uint8_t num_channels;
};

// static void adc_ambiq_global_irq_cfg(void)
// {
// 	static bool global_irq_init = true;

// 	if (!global_irq_init) {
// 		return;
// 	}

// 	global_irq_init = false;

// }

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_ambiq_data *data = CONTAINER_OF(ctx, struct adc_ambiq_data, ctx);
	const struct device *dev = data->dev;
	const struct adc_ambiq_config *cfg = dev->config;

	// data->repeat_buffer = data->buffer;

	// /* Enable EOC interrupt */
	// ADC_CTL0(cfg->reg) |= ADC_CTL0_EOCIE;

	// /* Set ADC software conversion trigger. */
	// ADC_CTL1(cfg->reg) |= ADC_CTL1_SWRCST;
}


static int adc_ambiq_read(const struct device *dev,
			 const struct adc_sequence *sequence)
{
	struct adc_ambiq_data *data = dev->data;
	int error;

	// adc_context_lock(&data->ctx, false, NULL);
	// // error = adc_ambiq_start_read(dev, sequence);
	// adc_context_release(&data->ctx, error);

	return error;
}


static int adc_ambiq_channel_setup(const struct device *dev,
				  const struct adc_channel_cfg *chan_cfg)
{
	// const struct adc_ambiq_config *cfg = dev->config;

	if (chan_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Gain is not valid");
		return -ENOTSUP;
	}

	if (chan_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Reference is not valid");
		return -ENOTSUP;
	}

	if (chan_cfg->differential) {
		LOG_ERR("Differential sampling not supported");
		return -ENOTSUP;
	}

	// if (chan_cfg->channel_id >= cfg->channels) {
	// 	LOG_ERR("Invalid channel (%u)", chan_cfg->channel_id);
	// 	return -EINVAL;
	// }

    return 0;
	// return adc_ambiq_configure_sampt(cfg, chan_cfg->channel_id,
	// 				chan_cfg->acquisition_time);
}

static struct adc_driver_api adc_ambiq_driver_api = {
	.channel_setup = adc_ambiq_channel_setup,
	.read = adc_ambiq_read,
};


static int adc_ambiq_init(const struct device *dev){
    struct adc_ambiq_data *data = dev->data;
	const struct adc_ambiq_config *cfg = dev->config;
	int ret;

	data->dev = dev;

	ret = pinctrl_apply_state(cfg->pin_cfg, PINCTRL_STATE_DEFAULT);

	return ret;
}

#define ADC_AMBIQ_INIT(n)                                                                          \
    PINCTRL_DT_INST_DEFINE(n);                                                                 \
    static struct adc_ambiq_data adc_ambiq_data_##n = {                                          \
            ADC_CONTEXT_INIT_TIMER(adc_ambiq_data_##n, ctx),                                    \
            ADC_CONTEXT_INIT_LOCK(adc_ambiq_data_##n, ctx),                                     \
            ADC_CONTEXT_INIT_SYNC(adc_ambiq_data_##n, ctx),                                     \
    };                                                                                         \
    const static struct adc_ambiq_config adc_ambiq_config_##n = {                                \
            .base = DT_INST_REG_ADDR(n),                                                        \
            .pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
    };  \
    DEVICE_DT_INST_DEFINE(n, &adc_ambiq_init, NULL,     \
                            &adc_ambiq_data_##n, &adc_ambiq_config_##n,   \
                            POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,        \
                            &adc_ambiq_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ADC_AMBIQ_INIT)