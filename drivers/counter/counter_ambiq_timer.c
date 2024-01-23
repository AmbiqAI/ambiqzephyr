/*
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_counter

#include <zephyr/drivers/counter.h>
#include <zephyr/spinlock.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
/* ambiq-sdk includes */
#include <am_mcu_apollo.h>

LOG_MODULE_REGISTER(ambiq_counter, CONFIG_COUNTER_LOG_LEVEL);

static void counter_ambiq_isr(void *arg);

struct counter_ambiq_config {
	struct counter_config_info counter_info;
	uint32_t instance;
	void (*irq_config_func)(void);
};

struct counter_ambiq_data {
	counter_alarm_callback_t callback;
	void *user_data;
};

static struct k_spinlock lock;

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
/* Timer configuration */
static am_hal_ctimer_config_t g_sContTimer =
{
    // Create 32-bit timer
    1,

    // Set up TimerA.
    (AM_HAL_CTIMER_FN_REPEAT    |
     AM_HAL_CTIMER_INT_ENABLE   |
     AM_HAL_CTIMER_HFRC_3MHZ),

    // Set up TimerB.
    0
};
/* AM_HAL_CTIMER_HFRC_3MHZ */
#define COUNTER_FREQ 3000000
#define COUNTER_BASE REG_CTIMER_BASEADDR
#else
/* AM_HAL_TIMER_CLOCK_HFRC_DIV16 */
#define COUNTER_FREQ 6000000
#define COUNTER_BASE (REG_TIMER_BASEADDR + 0x200)
#endif
static int counter_ambiq_init(const struct device *dev)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	const struct counter_ambiq_config *cfg = dev->config;

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

	am_hal_ctimer_clear(cfg->instance, AM_HAL_CTIMER_BOTH);
	am_hal_ctimer_config(cfg->instance, &g_sContTimer);
#else
	am_hal_timer_config_t tc;

	am_hal_timer_default_config_set(&tc);
	tc.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV16;
	tc.eFunction = AM_HAL_TIMER_FN_UPCOUNT;
	tc.ui32PatternLimit = 0;

	am_hal_timer_config(cfg->instance, &tc);
#endif
	cfg->irq_config_func();

	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ambiq_start(const struct device *dev)
{
	const struct counter_ambiq_config *cfg = dev->config;
	k_spinlock_key_t key = k_spin_lock(&lock);

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	am_hal_ctimer_start(cfg->instance, AM_HAL_CTIMER_TIMERA);
#else
	am_hal_timer_start(cfg->instance);
#endif

	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ambiq_stop(const struct device *dev)
{
	const struct counter_ambiq_config *cfg = dev->config;

	k_spinlock_key_t key = k_spin_lock(&lock);

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	am_hal_ctimer_stop(cfg->instance, AM_HAL_CTIMER_BOTH);
#else
	am_hal_timer_stop(cfg->instance);
#endif

	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ambiq_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_ambiq_config *cfg = dev->config;

	k_spinlock_key_t key = k_spin_lock(&lock);

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	*ticks = (am_hal_ctimer_read(cfg->instance, AM_HAL_CTIMER_TIMERA)) | (am_hal_ctimer_read(cfg->instance, AM_HAL_CTIMER_TIMERB) << 16);
#else
	*ticks = am_hal_timer_read(cfg->instance);
#endif

	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ambiq_set_alarm(const struct device *dev, uint8_t chan_id,
				   const struct counter_alarm_cfg *alarm_cfg)
{
	ARG_UNUSED(chan_id);
	struct counter_ambiq_data *data = dev->data;
	const struct counter_ambiq_config *cfg = dev->config;
	uint32_t now;

	counter_ambiq_get_value(dev, &now);

	k_spinlock_key_t key = k_spin_lock(&lock);

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);
	am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);

	if ((alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) == 0) {
		am_hal_ctimer_compare_set(cfg->instance, AM_HAL_CTIMER_BOTH, 0, now + alarm_cfg->ticks);
	} else {
		am_hal_ctimer_compare_set(cfg->instance, AM_HAL_CTIMER_BOTH, 0, alarm_cfg->ticks);
	}
#else
	/* Enable interrupt, due to counter_ambiq_cancel_alarm() disables it*/
	am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(cfg->instance, AM_HAL_TIMER_COMPARE1));
	am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(cfg->instance, AM_HAL_TIMER_COMPARE1));

	if ((alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) == 0) {
		am_hal_timer_compare1_set(cfg->instance, now + alarm_cfg->ticks);
	} else {
		am_hal_timer_compare1_set(cfg->instance, alarm_cfg->ticks);
	}
#endif

	data->user_data = alarm_cfg->user_data;
	data->callback = alarm_cfg->callback;

	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ambiq_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	ARG_UNUSED(chan_id);
	const struct counter_ambiq_config *cfg = dev->config;
	k_spinlock_key_t key = k_spin_lock(&lock);

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	am_hal_ctimer_int_disable(AM_HAL_CTIMER_INT_TIMERA0);
	/* Reset the compare register */
	am_hal_ctimer_compare_set(cfg->instance, AM_HAL_CTIMER_BOTH, 0, 0);
#else
	am_hal_timer_interrupt_disable(AM_HAL_TIMER_MASK(cfg->instance, AM_HAL_TIMER_COMPARE1));
	/* Reset the compare register */
	am_hal_timer_compare1_set(cfg->instance, 0);
#endif

	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ambiq_set_top_value(const struct device *dev, const struct counter_top_cfg *cfg)
{
	const struct counter_ambiq_config *config = dev->config;

	if (cfg->ticks != config->counter_info.max_top_value) {
		return -ENOTSUP;
	} else {
		return 0;
	}
}

static uint32_t counter_ambiq_get_pending_int(const struct device *dev)
{
	return 0;
}

static uint32_t counter_ambiq_get_top_value(const struct device *dev)
{
	const struct counter_ambiq_config *config = dev->config;

	return config->counter_info.max_top_value;
}

static const struct counter_driver_api counter_api = {
	.start = counter_ambiq_start,
	.stop = counter_ambiq_stop,
	.get_value = counter_ambiq_get_value,
	.set_alarm = counter_ambiq_set_alarm,
	.cancel_alarm = counter_ambiq_cancel_alarm,
	.set_top_value = counter_ambiq_set_top_value,
	.get_pending_int = counter_ambiq_get_pending_int,
	.get_top_value = counter_ambiq_get_top_value,
};

static void counter_ambiq_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct counter_ambiq_data *data = dev->data;
	uint32_t now = 0;

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);
#else
	const struct counter_ambiq_config *cfg = dev->config;
	am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(cfg->instance, AM_HAL_TIMER_COMPARE1));
#endif
	counter_ambiq_get_value(dev, &now);

	if (data->callback) {
		data->callback(dev, 0, now, data->user_data);
	}
}

#define AMBIQ_COUNTER_INIT(idx)                                                                    \
                                                                                                   \
	static void counter_irq_config_func_##idx(void)                                                  \
	{                                                                                          \
		NVIC_ClearPendingIRQ(DT_INST_IRQN(idx));                                                      \
		IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), counter_ambiq_isr,              \
			    DEVICE_DT_INST_GET(idx), 0);                                             \
		irq_enable(DT_INST_IRQN(idx));                                                       \
	};                                                                                         \
	static struct counter_ambiq_data counter_data_##idx;                                       \
                                                                                                   \
	static const struct counter_ambiq_config counter_config_##idx = {                          \
		.instance = (DT_INST_REG_ADDR(idx) - COUNTER_BASE) / DT_INST_REG_SIZE(idx),            \
		.counter_info = {.max_top_value = UINT32_MAX,                                      \
				 .freq = COUNTER_FREQ,                                                  \
				 .flags = COUNTER_CONFIG_INFO_COUNT_UP,                            \
				 .channels = 1},                                                   \
		.irq_config_func = counter_irq_config_func_##idx,                                        \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, counter_ambiq_init, NULL, &counter_data_##idx,                  \
			      &counter_config_##idx, PRE_KERNEL_1, CONFIG_COUNTER_INIT_PRIORITY,   \
			      &counter_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_COUNTER_INIT);
