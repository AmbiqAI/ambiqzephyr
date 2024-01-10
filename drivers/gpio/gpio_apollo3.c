/*
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 * Copyright (c) 2023 Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_apollo3_gpio_bank

#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>

#include <am_mcu_apollo.h>

typedef void (*ambiq_gpio_cfg_func_t)(void);

struct ambiq_gpio_config {
	struct gpio_driver_config common;
	uint32_t base;
	uint32_t offset;
	uint32_t irq_num;
	ambiq_gpio_cfg_func_t cfg_func;
	uint8_t ngpios;
};

struct ambiq_gpio_data {
	struct gpio_driver_data common;
	sys_slist_t cb;
	struct k_spinlock lock;
};

static int ambiq_gpio_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct ambiq_gpio_config *const dev_cfg = dev->config;

	pin += dev_cfg->offset;

	am_hal_gpio_pincfg_t pincfg = g_AM_HAL_GPIO_DISABLE;

	if (flags & GPIO_INPUT) {
		pincfg = g_AM_HAL_GPIO_INPUT;
		if (flags & GPIO_PULL_UP) {
			pincfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
		} else if (flags & GPIO_PULL_DOWN) {
			pincfg.ePullup = AM_HAL_GPIO_PIN_PULLDOWN;
		}
	}
	if (flags & GPIO_OUTPUT) {
		if (flags & GPIO_SINGLE_ENDED) {
			if (flags & GPIO_LINE_OPEN_DRAIN) {
				pincfg.eGPOutcfg = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN;
			}
		} else {
			pincfg.eGPOutcfg = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL;
		}
	}
	if (flags & GPIO_DISCONNECTED) {
		pincfg = g_AM_HAL_GPIO_DISABLE;
	}

	if (flags & GPIO_OUTPUT_INIT_HIGH) {
		pincfg.eCEpol = AM_HAL_GPIO_PIN_CEPOL_ACTIVEHIGH;
		am_hal_gpio_state_write(pin, AM_HAL_GPIO_OUTPUT_SET);

	} else if (flags & GPIO_OUTPUT_INIT_LOW) {
		pincfg.eCEpol = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW;
		am_hal_gpio_state_write(pin, AM_HAL_GPIO_OUTPUT_CLEAR);
	}

	am_hal_gpio_pinconfig(pin, pincfg);

	return 0;
}

uint32_t
am_hal_gpio_pinconfig_get(uint32_t ui32GpioNum, am_hal_gpio_pincfg_t* psGpioCfg)
{
    uint32_t ui32GPCfgAddr, ui32PadregAddr, ui32AltpadAddr;
    uint32_t ui32GPCfgMask, ui32PadMask;
    uint32_t ui32GPCfgShft, ui32PadShft;
    uint32_t ui32GPCfgVal, ui32PadVal, ui32AltVal;

    if ( ui32GpioNum >= AM_HAL_GPIO_MAX_PADS )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    if ( psGpioCfg == (am_hal_gpio_pincfg_t*)0x0 )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32GPCfgAddr       = AM_REGADDR(GPIO, CFGA)       + ((ui32GpioNum >> 1) & ~0x3);
    ui32PadregAddr      = AM_REGADDR(GPIO, PADREGA)    + (ui32GpioNum & ~0x3);
    ui32AltpadAddr      = AM_REGADDR(GPIO, ALTPADCFGA) + (ui32GpioNum & ~0x3);
    ui32GPCfgShft       = ((ui32GpioNum & 0x7) << 2);
    ui32PadShft         = ((ui32GpioNum & 0x3) << 3);
    ui32GPCfgMask  = (uint32_t)0xF  << ui32GPCfgShft;
    ui32PadMask    = (uint32_t)0xFF << ui32PadShft;

    ui32GPCfgVal = (AM_REGVAL(ui32GPCfgAddr) & ui32GPCfgMask) >> ui32GPCfgShft;
    ui32PadVal = (AM_REGVAL(ui32PadregAddr) & ui32PadMask) >> ui32PadShft;
    ui32AltVal = (AM_REGVAL(ui32AltpadAddr) & ui32PadMask) >> ui32PadShft;

    psGpioCfg->eGPOutcfg = (ui32GPCfgVal & GPIO_CFGA_GPIO0OUTCFG_Msk) >> GPIO_CFGA_GPIO0OUTCFG_Pos;
    psGpioCfg->eGPInput = (ui32PadVal & GPIO_PADREGA_PAD0INPEN_Msk) >> GPIO_PADREGA_PAD0INPEN_Pos;
    psGpioCfg->ePullup = (ui32PadVal & GPIO_PADREGA_PAD0RSEL_Msk) >> GPIO_PADREGA_PAD0RSEL_Pos;
    psGpioCfg->uFuncSel = (ui32PadVal & GPIO_PADREGA_PAD0FNCSEL_Msk) >> GPIO_PADREGA_PAD0FNCSEL_Pos;
    psGpioCfg->eCEpol = (ui32GPCfgVal & GPIO_CFGA_GPIO0INTD_Msk) >> GPIO_CFGA_GPIO0INTD_Pos;
    psGpioCfg->eIntDir = (((ui32GPCfgVal & GPIO_CFGA_GPIO0INCFG_Msk) >> GPIO_CFGA_GPIO0INCFG_Pos) << 1) | psGpioCfg->eCEpol;
    psGpioCfg->eDriveStrength = (((ui32AltVal & GPIO_ALTPADCFGA_PAD0_DS1_Msk) >> GPIO_ALTPADCFGA_PAD0_DS1_Pos) << 1) | 
                             ((ui32PadVal & GPIO_PADREGA_PAD0STRNG_Msk) >> GPIO_PADREGA_PAD0STRNG_Pos);
    psGpioCfg->eGPRdZero = (ui32GPCfgVal & GPIO_CFGA_GPIO0INCFG_Msk) >> GPIO_CFGA_GPIO0INCFG_Pos;

    return AM_HAL_STATUS_SUCCESS;
}

#ifdef CONFIG_GPIO_GET_CONFIG
static int ambiq_gpio_get_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t *out_flags)
{
	const struct ambiq_gpio_config *const dev_cfg = dev->config;
	am_hal_gpio_pincfg_t pincfg;

	pin += dev_cfg->offset;

	am_hal_gpio_pinconfig_get(pin, &pincfg);

	if (pincfg.eGPOutcfg == AM_HAL_GPIO_PIN_OUTCFG_DISABLE &&
	    pincfg.eGPInput == AM_HAL_GPIO_PIN_INPUT_NONE) {
		*out_flags = GPIO_DISCONNECTED;
	}
	if (pincfg.eGPInput == AM_HAL_GPIO_PIN_INPUT_ENABLE) {
		*out_flags = GPIO_INPUT;
		if (pincfg.ePullup == AM_HAL_GPIO_PIN_PULLUP_1_5K) {
			*out_flags |= GPIO_PULL_UP;
		} else if (pincfg.ePullup == AM_HAL_GPIO_PIN_PULLDOWN) {
			*out_flags |= GPIO_PULL_DOWN;
		}
	}
	if (pincfg.eGPOutcfg == AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL) {
		*out_flags = GPIO_OUTPUT | GPIO_PUSH_PULL;
		if (pincfg.eCEpol == AM_HAL_GPIO_PIN_CEPOL_ACTIVEHIGH) {
			*out_flags |= GPIO_OUTPUT_HIGH;
		} else if (pincfg.eCEpol == AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW) {
			*out_flags |= GPIO_OUTPUT_LOW;
		}
	}
	if (pincfg.eGPOutcfg == AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN) {
		*out_flags = GPIO_OUTPUT | GPIO_OPEN_DRAIN;
		if (pincfg.eCEpol == AM_HAL_GPIO_PIN_CEPOL_ACTIVEHIGH) {
			*out_flags |= GPIO_OUTPUT_HIGH;
		} else if (pincfg.eCEpol == AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW) {
			*out_flags |= GPIO_OUTPUT_LOW;
		}
	}

	return 0;
}
#endif

#ifdef CONFIG_GPIO_GET_DIRECTION
static int ambiq_gpio_port_get_direction(const struct device *dev, gpio_port_pins_t map,
					 gpio_port_pins_t *inputs, gpio_port_pins_t *outputs)
{
	const struct ambiq_gpio_config *const dev_cfg = dev->config;
	am_hal_gpio_pincfg_t pincfg;
	gpio_port_pins_t ip = 0;
	gpio_port_pins_t op = 0;
	uint32_t pin_offset = dev_cfg->offset;

	if (inputs != NULL) {
		for (int i = 0; i < dev_cfg->ngpios; i++) {
			if ((map >> i) & 1) {
				am_hal_gpio_pinconfig_get(i + pin_offset, &pincfg);
				if (pincfg.eGPInput == AM_HAL_GPIO_PIN_INPUT_ENABLE) {
					ip |= BIT(i);
				}
			}
		}
		*inputs = ip;
	}
	if (outputs != NULL) {
		for (int i = 0; i < dev_cfg->ngpios; i++) {
			if ((map >> i) & 1) {
				am_hal_gpio_pinconfig_get(i + pin_offset, &pincfg);
				if (pincfg.eGPOutcfg == AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL ||
				    pincfg.eGPOutcfg == AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN) {
					op |= BIT(i);
				}
			}
		}
		*outputs = op;
	}

	return 0;
}
#endif

static int ambiq_gpio_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	const struct ambiq_gpio_config *const dev_cfg = dev->config;

	*value = am_hal_gpio_input_read(dev_cfg->offset);

	return 0;
}

static int ambiq_gpio_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	const struct ambiq_gpio_config *const dev_cfg = dev->config;
	uint32_t pin_offset = dev_cfg->offset;

	for (int i = 0; i < dev_cfg->ngpios; i++) {
		if ((mask >> i) & 1) {
			am_hal_gpio_state_write(i + pin_offset, ((value >> i) & 1));
		}
	}

	return 0;
}

static int ambiq_gpio_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct ambiq_gpio_config *const dev_cfg = dev->config;
	uint32_t pin_offset = dev_cfg->offset;

	for (int i = 0; i < dev_cfg->ngpios; i++) {
		if ((pins >> i) & 1) {
			am_hal_gpio_state_write(i + pin_offset, AM_HAL_GPIO_OUTPUT_SET);
		}
	}

	return 0;
}

static int ambiq_gpio_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct ambiq_gpio_config *const dev_cfg = dev->config;
	uint32_t pin_offset = dev_cfg->offset;

	for (int i = 0; i < dev_cfg->ngpios; i++) {
		if ((pins >> i) & 1) {
			am_hal_gpio_state_write(i + pin_offset, AM_HAL_GPIO_OUTPUT_CLEAR);
		}
	}

	return 0;
}

static int ambiq_gpio_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	const struct ambiq_gpio_config *const dev_cfg = dev->config;
	uint32_t pin_offset = dev_cfg->offset;

	for (int i = 0; i < dev_cfg->ngpios; i++) {
		if ((pins >> i) & 1) {
			am_hal_gpio_state_write(i + pin_offset, AM_HAL_GPIO_OUTPUT_TOGGLE);
		}
	}

	return 0;
}

static void ambiq_gpio_isr(const struct device *dev)
{
	struct ambiq_gpio_data *const data = dev->data;

#if defined(CONFIG_SOC_APOLLO3_BLUE)
	uint64_t ui64Status;
	am_hal_gpio_interrupt_status_get(false, &ui64Status);
	am_hal_gpio_interrupt_clear(ui64Status);
	gpio_fire_callbacks(&data->cb, dev, (uint32_t)ui64Status);
#elif defined(CONFIG_SOC_APOLLO3P_BLUE)
	AM_HAL_GPIO_MASKCREATE(GpioIntStatusMask);
	am_hal_gpio_interrupt_status_get(false, pGpioIntStatusMask);
	am_hal_gpio_interrupt_clear(pGpioIntStatusMask);
	gpio_fire_callbacks(&data->cb, dev, *(uint32_t*)pGpioIntStatusMask);
#endif
}

static int ambiq_gpio_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
							enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct ambiq_gpio_config *const dev_cfg = dev->config;
	struct ambiq_gpio_data *const data = dev->data;

	am_hal_gpio_pincfg_t pincfg = {0};
	int gpio_pin = pin + dev_cfg->offset;
	int ret;

	ret = am_hal_gpio_pinconfig_get(gpio_pin, &pincfg);

	if (mode == GPIO_INT_MODE_DISABLED) {
		pincfg.eIntDir = AM_HAL_GPIO_PIN_INTDIR_NONE;
		ret = am_hal_gpio_pinconfig(gpio_pin, pincfg);

		k_spinlock_key_t key = k_spin_lock(&data->lock);

#if defined(CONFIG_SOC_APOLLO3_BLUE)
		ret = am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(gpio_pin));
		ret = am_hal_gpio_interrupt_enable(AM_HAL_GPIO_BIT(gpio_pin));
#elif defined(CONFIG_SOC_APOLLO3P_BLUE)
		AM_HAL_GPIO_MASKCREATE(GpioIntMask);
		ret = am_hal_gpio_interrupt_clear( AM_HAL_GPIO_MASKBIT(pGpioIntMask, gpio_pin));
		ret = am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, gpio_pin));
#endif
		k_spin_unlock(&data->lock, key);

	} else {
		if (mode == GPIO_INT_MODE_LEVEL) {
			return -ENOTSUP;
		}
		switch (trig) {
		case GPIO_INT_TRIG_LOW:
			pincfg.eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO;
			break;
		case GPIO_INT_TRIG_HIGH:
			pincfg.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
			break;
		case GPIO_INT_TRIG_BOTH:
			return -ENOTSUP;
		}
		ret = am_hal_gpio_pinconfig(gpio_pin, pincfg);

		irq_enable(dev_cfg->irq_num);

		k_spinlock_key_t key = k_spin_lock(&data->lock);

#if defined(CONFIG_SOC_APOLLO3_BLUE)
		ret = am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(gpio_pin));
		ret = am_hal_gpio_interrupt_enable(AM_HAL_GPIO_BIT(gpio_pin));
#elif defined(CONFIG_SOC_APOLLO3P_BLUE)
		AM_HAL_GPIO_MASKCREATE(GpioIntMask);
		ret = am_hal_gpio_interrupt_clear( AM_HAL_GPIO_MASKBIT(pGpioIntMask, gpio_pin));
		ret = am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, gpio_pin));
#endif
		k_spin_unlock(&data->lock, key);
	}
	return ret;
}

static int ambiq_gpio_manage_callback(const struct device *dev, struct gpio_callback *callback,
						bool set)
{
	struct ambiq_gpio_data *const data = dev->data;

	return gpio_manage_callback(&data->cb, callback, set);
}

static int ambiq_gpio_init(const struct device *port)
{
	const struct ambiq_gpio_config *const dev_cfg = port->config;

	NVIC_ClearPendingIRQ(dev_cfg->irq_num);

	dev_cfg->cfg_func();

	return 0;
}

static void ambiq_gpio_cfg_func(void)
{
	IRQ_CONNECT(GPIO_IRQn, DT_INST_IRQ(0, priority), ambiq_gpio_isr, DEVICE_DT_INST_GET(0), 0);
	return;
}

static const struct gpio_driver_api ambiq_gpio_drv_api = {
	.pin_configure = ambiq_gpio_pin_configure,
#ifdef CONFIG_GPIO_GET_CONFIG
	.pin_get_config = ambiq_gpio_get_config,
#endif
	.port_get_raw = ambiq_gpio_port_get_raw,
	.port_set_masked_raw = ambiq_gpio_port_set_masked_raw,
	.port_set_bits_raw = ambiq_gpio_port_set_bits_raw,
	.port_clear_bits_raw = ambiq_gpio_port_clear_bits_raw,
	.port_toggle_bits = ambiq_gpio_port_toggle_bits,
	.pin_interrupt_configure = ambiq_gpio_pin_interrupt_configure,
	.manage_callback = ambiq_gpio_manage_callback,
#ifdef CONFIG_GPIO_GET_DIRECTION
	.port_get_direction = ambiq_gpio_port_get_direction,
#endif
};

#define AMBIQ_GPIO_DEFINE(n)                                                                       \
	static struct ambiq_gpio_data ambiq_gpio_data_##n;                                         \
	static const struct ambiq_gpio_config ambiq_gpio_config_##n = {                            \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),               \
			},                                                                         \
		.base = DT_REG_ADDR(DT_INST_PARENT(n)),                                            \
		.offset = DT_INST_REG_ADDR(n),                                                     \
		.ngpios = DT_INST_PROP(n, ngpios),                                                 \
		.irq_num = DT_INST_IRQN(n),                                                        \
		.cfg_func = ambiq_gpio_cfg_func};                                              \
	DEVICE_DT_INST_DEFINE(n, &ambiq_gpio_init, NULL, &ambiq_gpio_data_##n,                     \
			      &ambiq_gpio_config_##n, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,     \
			      &ambiq_gpio_drv_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_GPIO_DEFINE)
