/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "am_mcu_apollo.h"

static int lp_init(void)
{
	am_hal_pwrctrl_mcu_memory_config_t McuMemCfg =
	{
		.eCacheCfg    = AM_HAL_PWRCTRL_CACHE_ALL,
		.bRetainCache = true,
		.eDTCMCfg     = AM_HAL_PWRCTRL_DTCM_128K,
		.eRetainDTCM  = AM_HAL_PWRCTRL_DTCM_128K,
		.bEnableNVM0  = true,
		.bRetainNVM0  = false
	};

	am_hal_pwrctrl_sram_memcfg_t SRAMMemCfg =
	{
		.eSRAMCfg           = AM_HAL_PWRCTRL_SRAM_NONE,
		.eActiveWithMCU     = AM_HAL_PWRCTRL_SRAM_NONE,
		.eActiveWithGFX     = AM_HAL_PWRCTRL_SRAM_NONE,
		.eActiveWithDISP    = AM_HAL_PWRCTRL_SRAM_NONE,
		.eActiveWithDSP     = AM_HAL_PWRCTRL_SRAM_NONE,
		.eSRAMRetain        = AM_HAL_PWRCTRL_SRAM_NONE
	};

    am_hal_pwrctrl_dsp_memory_config_t    DSPMemCfg =
    {
        .bEnableICache      = false,
        .bRetainCache       = false,
        .bEnableRAM         = false,
        .bActiveRAM         = false,
        .bRetainRAM         = false
    };

	am_hal_sysctrl_fpu_disable();
    am_hal_pwrctrl_low_power_init();
    am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_SIMOBUCK_INIT, 0);
    am_hal_rtc_osc_disable();

	am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
	am_hal_pwrctrl_mcu_memory_config(&McuMemCfg);
	am_hal_pwrctrl_sram_config(&SRAMMemCfg);
	am_hal_pwrctrl_dsp_memory_config(AM_HAL_DSP0, &DSPMemCfg);
    am_hal_gpio_state_write(55, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_hal_gpio_pinconfig(55, am_hal_gpio_pincfg_output);
    am_hal_gpio_state_write(55, AM_HAL_GPIO_OUTPUT_SET);
    am_hal_gpio_state_write(55, AM_HAL_GPIO_OUTPUT_CLEAR);

	return 0;
}

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD);

	lp_init();

	while (1) {
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
	}
	return 0;
}
