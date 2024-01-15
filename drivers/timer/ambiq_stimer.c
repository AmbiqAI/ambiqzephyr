/*
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_stimer

/**
 * @file
 * @brief Ambiq Apollo STIMER-based sys_clock driver
 *
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>

/* ambiq-sdk includes */
#include <am_mcu_apollo.h>

#define COUNTER_MAX UINT32_MAX

#define CYC_PER_TICK (sys_clock_hw_cycles_per_sec() / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MAX_TICKS    ((k_ticks_t)(COUNTER_MAX / CYC_PER_TICK) - 1)
#define MAX_CYCLES   (MAX_TICKS * CYC_PER_TICK)
#define MIN_DELAY    1

#define TIMER_IRQ (DT_INST_IRQN(0))

#if defined(CONFIG_TEST)
const int32_t z_sys_timer_irq_for_test = TIMER_IRQ;
#endif

/* Value of STIMER counter when the previous kernel tick was announced */
static atomic_t g_last_count;

/* Spinlock to sync between Compare ISR and update of Compare register */
static struct k_spinlock g_lock;

static void stimer_isr(const void *arg)
{
	ARG_UNUSED(arg);

	uint32_t irq_status = am_hal_stimer_int_status_get(false);

	if (irq_status & AM_HAL_STIMER_INT_COMPAREA) {
		am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);

		k_spinlock_key_t key = k_spin_lock(&g_lock);

		/* Read current cycle count. */
		uint32_t now = am_hal_stimer_counter_get();

		/* Get elapsed cycles, compare current count and recorded count to
		 * cope with the hardware counter overflow condition.
		 */
		uint32_t elapsed_cycle = (now >= g_last_count) ? (now - g_last_count)
							       : (now + COUNTER_MAX - g_last_count);

		/* Get elapsed ticks. */
		uint32_t dticks = elapsed_cycle / CYC_PER_TICK;

		if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
			/* Get the counter value to trigger the next tick interrupt. */
			uint32_t next = now + (dticks + 1) * CYC_PER_TICK;

			/* Read again to prevent time skew */
			now = am_hal_stimer_counter_get();

			/* Calculate the delta value. */
			uint32_t delta = (next >= now) ? (next - now) : (next + COUNTER_MAX - now);

			/* A delta value smaller than MIN_DELAY means the stimer interrupt will be
			 * trigger again as soon as we exits current ISR. We will prevent this
			 * situation by increasing the dticks and delta as if the interrupt has
			 * already been triggered.
			 */
			if (delta < MIN_DELAY) {
				dticks += 1;
				delta += CYC_PER_TICK;
			}

			/* Set delta. */
			am_hal_stimer_compare_delta_set(0, delta);
		}

		g_last_count += dticks * CYC_PER_TICK;

		k_spin_unlock(&g_lock, key);
		sys_clock_announce(dticks);
	}
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

	/* Adjust the ticks to the range of [0, MAX_TICKS]. */
	ticks = (ticks == K_TICKS_FOREVER) ? MAX_TICKS : ticks;
	ticks = CLAMP(ticks, 1, (int32_t)MAX_TICKS);

	k_spinlock_key_t key = k_spin_lock(&g_lock);

	/* Get current hardware counter value.*/
	uint32_t current = am_hal_stimer_counter_get();

	/* announced: the last announced counter value.
	 * now: current counter value. Extended to uint64_t to easy the handing of hardware counter
	 * overflow. next: counter values where to trigger the scheduled timeout. upperBoundary: the
	 * maximal counter value allowed to be set, if a timeout interrupt triggered after this
	 * point, the gap between that triggered point and the last announced point will overflow
	 * uint32_t. announced < now < next
	 */
	uint64_t announced = (uint64_t)g_last_count;
	uint64_t now =
		(g_last_count < current) ? (uint64_t)current : (uint64_t)current + COUNTER_MAX;
	uint64_t next = now + ticks * CYC_PER_TICK;
	uint64_t upperBoundary = announced + MAX_CYCLES;

	uint32_t delta;

	if (upperBoundary < now) {
		/* This only happens in a corner case where the following
		 * two conditions satisfied at the same time:
		 * 1. new timeouts keep being set before the existing one fires.
		 * 2. stimer interrupt is pended for over 2ms before it is handled.
		 */
		delta = MIN_DELAY;
	} else if (upperBoundary <= next) {
		delta = upperBoundary - now;
	} else {
		uint32_t gap = next - announced;
		uint32_t gapAligned = (gap / CYC_PER_TICK) * CYC_PER_TICK;
		uint64_t nextAligned = announced + gapAligned;

		delta = nextAligned - now;
	}

	if (delta <= MIN_DELAY) {
		/*If the delta value is smaller than MIN_DELAY, trigger a interrupt immediately*/
		am_hal_stimer_int_set(AM_HAL_STIMER_INT_COMPAREA);
	} else {
		am_hal_stimer_compare_delta_set(0, delta);
	}

	k_spin_unlock(&g_lock, key);
}

uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&g_lock);

	uint32_t now = am_hal_stimer_counter_get();
	uint32_t elapsed =
		(now >= g_last_count) ? (now - g_last_count) : (now + COUNTER_MAX - g_last_count);

	uint32_t ret = elapsed / CYC_PER_TICK;

	k_spin_unlock(&g_lock, key);
	return ret;
}

uint32_t sys_clock_cycle_get_32(void)
{
	return am_hal_stimer_counter_get();
}

static int stimer_init(void)
{
	uint32_t oldCfg;
	k_spinlock_key_t key = k_spin_lock(&g_lock);

	oldCfg = am_hal_stimer_config(AM_HAL_STIMER_CFG_FREEZE);

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	am_hal_stimer_config((oldCfg & ~(AM_HAL_STIMER_CFG_FREEZE | CTIMER_STCFG_CLKSEL_Msk)) |
			     AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE);
#else
	am_hal_stimer_config((oldCfg & ~(AM_HAL_STIMER_CFG_FREEZE | STIMER_STCFG_CLKSEL_Msk)) |
			     AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE);
#endif
	g_last_count = am_hal_stimer_counter_get();

	k_spin_unlock(&g_lock, key);

	NVIC_ClearPendingIRQ(TIMER_IRQ);
	IRQ_CONNECT(TIMER_IRQ, 0, stimer_isr, 0, 0);
	irq_enable(TIMER_IRQ);

	am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREA);
	/* Start timer with period CYC_PER_TICK if tickless is not enabled */
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		am_hal_stimer_compare_delta_set(0, CYC_PER_TICK);
	}
	return 0;
}

SYS_INIT(stimer_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
