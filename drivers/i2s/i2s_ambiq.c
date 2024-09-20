/*
 * Copyright (c) 2024 Ambiq, LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_i2s

/** @file
 * @brief I2S bus driver for AMBIQ MCU family.
 *
 */

#include <errno.h>
#include <string.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/device_runtime.h>

#include <soc.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#include <am_mcu_apollo.h>

LOG_MODULE_REGISTER(i2s_ambiq, CONFIG_I2S_LOG_LEVEL);

typedef int (*ambiq_i2s_pwr_func_t)(void);

#define MODULO_INC(val, max) { val = (++val < max) ? val : 0; }

struct queue_item {
	void *mem_block;
	size_t size;
};

/* Minimal ring buffer implementation */
struct ring_buf {
	struct queue_item *buf;
	uint16_t len;
	uint16_t head;
	uint16_t tail;
};

struct stream {
	struct k_sem sem;
	int32_t state;
	struct i2s_config cfg;
	bool is_configured : 1;
	struct ring_buf mem_block_queue;
	void *mem_block;
};

struct i2s_ambiq_dev_config {
	am_hal_i2s_config_t i2s_config;
	am_hal_i2s_transfer_t i2s_transfer_config;
	void *i2s_handle;
};

/* Device run time data */
struct i2s_ambiq_data {
	struct stream rx;
	struct stream tx;
	struct i2s_ambiq_dev_config ambiq_cfg;
};

struct i2s_ambiq_config {
	uint32_t base;
	int size;
	uint32_t clock_freq;
	const struct pinctrl_dev_config *pcfg;
	ambiq_i2s_pwr_func_t pwr_func;
	void (*irq_config_func)(void);
};

static am_hal_i2s_data_format_t g_sI2SDataConfig =
{
    .ePhase = AM_HAL_I2S_DATA_PHASE_SINGLE,
    .eDataDelay = 0x1,
    .ui32ChannelNumbersPhase1 = 2,
    .ui32ChannelNumbersPhase2 = 2,
    .eDataJust = AM_HAL_I2S_DATA_JUSTIFIED_LEFT,
    .eChannelLenPhase1 = AM_HAL_I2S_FRAME_WDLEN_32BITS,
    .eChannelLenPhase2 = AM_HAL_I2S_FRAME_WDLEN_32BITS,
    .eSampleLenPhase1 = AM_HAL_I2S_SAMPLE_LENGTH_24BITS,
    .eSampleLenPhase2 = AM_HAL_I2S_SAMPLE_LENGTH_24BITS
};

// Programmer Reference setting.
static am_hal_i2s_io_signal_t g_sI2SIOConfig =
{
    .eFyncCpol = AM_HAL_I2S_IO_FSYNC_CPOL_HIGH,
    .eTxCpol = AM_HAL_I2S_IO_TX_CPOL_FALLING,
    .eRxCpol = AM_HAL_I2S_IO_RX_CPOL_RISING
};

/*
 * Get data from the queue
 */
static int queue_get(struct ring_buf *rb, void **mem_block, size_t *size)
{
	unsigned int key = 0;

	key = irq_lock();

	if (rb->tail == rb->head) {
		/* Ring buffer is empty */
		irq_unlock(key);
		return -ENOMEM;
	}

	*mem_block = rb->buf[rb->tail].mem_block;
	*size = rb->buf[rb->tail].size;
	MODULO_INC(rb->tail, rb->len);

	irq_unlock(key);

	return 0;
}

/*
 * Put data in the queue
 */
static int queue_put(struct ring_buf *rb, void *mem_block, size_t size)
{
	uint16_t head_next = 0;
	unsigned int key = 0;

	key = irq_lock();

	head_next = rb->head;
	MODULO_INC(head_next, rb->len);

	if (head_next == rb->tail) {
		/* Ring buffer is full */
		irq_unlock(key);
		return -ENOMEM;
	}

	rb->buf[rb->head].mem_block = mem_block;
	rb->buf[rb->head].size = size;
	rb->head = head_next;

	irq_unlock(key);

	return 0;
}

static void free_tx_buffer(struct i2s_ambiq_data *data,
			   const void *buffer)
{
	k_mem_slab_free(data->tx.cfg.mem_slab, (void *)buffer);
	LOG_DBG("Freed TX %p", buffer);
}

static void free_rx_buffer(struct i2s_ambiq_data *data, void *buffer)
{
	k_mem_slab_free(data->rx.cfg.mem_slab, buffer);
	LOG_DBG("Freed RX %p", buffer);
}

static void rx_queue_drop(struct stream *stream)
{
	size_t size;
	void *mem_block;

	while (queue_get(&stream->mem_block_queue, &mem_block, &size) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, mem_block);
	}

	k_sem_reset(&stream->sem);
}

static void tx_queue_drop(struct stream *stream)
{
	size_t size;
	void *mem_block;
	unsigned int n = 0U;

	while (queue_get(&stream->mem_block_queue, &mem_block, &size) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, mem_block);
		n++;
	}

	for (; n > 0; n--) {
		k_sem_give(&stream->sem);
	}
}

static void purge_queue(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_ambiq_data *data = dev->data;
	struct queue_item q_item = {0};

	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		tx_queue_drop(&data->tx);
	}

	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		rx_queue_drop(&data->rx);
	}
}

int i2s_config(const struct device *dev, const struct i2s_config *config)
{
	struct i2s_ambiq_data *data = dev->data;
	const struct i2s_ambiq_config *cfg = dev->config;
	return 0;
}

static const struct i2s_config *i2s_ambiq_config_get(const struct device *dev,
						enum i2s_dir dir)
{
	struct i2s_ambiq_data *data = dev->data;

	if (dir == I2S_DIR_TX && data->tx.is_configured) {
		return &data->tx.cfg;
	}
	if (dir == I2S_DIR_RX && data->rx.is_configured) {
		return &data->rx.cfg;
	}

	return NULL;
}

static int i2s_ambiq_configure(const struct device *dev, enum i2s_dir dir,
				const struct i2s_config *i2s_cfg)
{
	struct i2s_ambiq_data *data = dev->data;
	const struct i2s_ambiq_cfg *cfg = dev->config;

	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		if (data->rx.state != I2S_STATE_NOT_READY &&
			data->rx.state != I2S_STATE_READY) {
			LOG_ERR("Invalid State for configuration");
			return -EINVAL;
		}
	}

	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		if (data->tx.state != I2S_STATE_NOT_READY &&
			data->tx.state != I2S_STATE_READY) {
			LOG_ERR("Invalid State for configuration");
			return -EINVAL;
		}
	}

 	/* reset state */
	if (i2s_cfg->frame_clk_freq == 0) { /* -> reset state */
		purge_queue(dev, dir);
		if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
			data->tx.is_configured = false;
			memset(&data->tx, 0, sizeof(data->tx));
			data->tx.state = I2S_STATE_NOT_READY;
		}
		if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
			data->rx.is_configured = false;
			memset(&data->rx, 0, sizeof(data->rx));
			data->rx.state = I2S_STATE_NOT_READY;
		}
		return 0;
	}

	__ASSERT_NO_MSG(i2s_cfg->mem_slab != NULL &&
			i2s_cfg->block_size != 0);

	/* Check if Master or Slave */
	data->ambiq_cfg.i2s_config.eMode = AM_HAL_I2S_IO_MODE_SLAVE;
	if (!(i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE) &&
	    !(i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE)) {
		data->ambiq_cfg.i2s_config.eMode = AM_HAL_I2S_IO_MODE_MASTER;
	}

	/* word size */
	switch (i2s_cfg->word_size) {
	case 8:
		data->ambiq_cfg.i2s_config.eData->eSampleLenPhase1 = AM_HAL_I2S_SAMPLE_LENGTH_8BITS;
		break;
	case 16:
		data->ambiq_cfg.i2s_config.eData->eSampleLenPhase1 = AM_HAL_I2S_SAMPLE_LENGTH_16BITS;
		break;
	case 24:
		data->ambiq_cfg.i2s_config.eData->eSampleLenPhase1 = AM_HAL_I2S_SAMPLE_LENGTH_24BITS;
		break;
	case 32:
		data->ambiq_cfg.i2s_config.eData->eSampleLenPhase1 = AM_HAL_I2S_SAMPLE_LENGTH_32BITS;
		break;
	default:
		LOG_ERR("Unsupported word size: %u", i2s_cfg->word_size);
		return -EINVAL;
	}

	/* data formatting */
	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		data->ambiq_cfg.i2s_config.eData->eDataJust = AM_HAL_I2S_DATA_JUSTIFIED_LEFT;
		break;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		data->ambiq_cfg.i2s_config.eData->eDataJust = AM_HAL_I2S_DATA_JUSTIFIED_LEFT;
		break;
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		data->ambiq_cfg.i2s_config.eData->eDataJust = AM_HAL_I2S_DATA_JUSTIFIED_RIGHT;
		break;
	default:
		LOG_ERR("Unsupported data format: 0x%02x", i2s_cfg->format);
		return -EINVAL;
	}

	if ((i2s_cfg->format & I2S_FMT_DATA_ORDER_LSB) ||
	    (i2s_cfg->format & I2S_FMT_BIT_CLK_INV) ||
	    (i2s_cfg->format & I2S_FMT_FRAME_CLK_INV)) {
		LOG_ERR("Unsupported format: 0x%02x", i2s_cfg->format);
		return -EINVAL;
	}

	if (i2s_cfg->channels == 2) {
	} else if (i2s_cfg->channels == 1) {
	} else {
		LOG_ERR("Unsupported number of channels: %u",
			i2s_cfg->channels);
		return -EINVAL;
	}

	if ((i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) &&
		(i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE)) {
	} else if (!(i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) &&
			!(i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE)) {
	} else {
		LOG_ERR("Unsupported operation mode: 0x%02x", i2s_cfg->options);
		return -EINVAL;
	}

	/* If the master clock generator is needed (i.e. in Master mode or when
	 * the MCK output is used), find a suitable clock configuration for it.
	 */
	if (data->ambiq_cfg.i2s_config.eMode == AM_HAL_I2S_IO_MODE_MASTER) {
		//find_suitable_clock(cfg, &nrfx_cfg, i2s_cfg);
		//data->request_clock = true; //TODO (true or false)
	} else {
		//data->request_clock = false;
	}

	/* internal loopback not supported */
	// if (i2s_cfg->options & I2S_OPT_LOOPBACK) {
	// 	LOG_ERR("Unsupported options: 0x%02x", i2s_cfg->options);
	// 	return -EINVAL;
	// }

	/* configure tx */
	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		memcpy(&data->tx.cfg, i2s_cfg, sizeof(struct i2s_config));
		data->tx.is_configured = true;
		data->tx.state = I2S_STATE_READY;
	}

	/* configure rx */
	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		memcpy(&data->rx.cfg, i2s_cfg, sizeof(struct i2s_config));
		data->rx.is_configured = true;
		data->rx.state = I2S_STATE_READY;
	}

	return 0;
}

static int i2s_ambiq_trigger(const struct device *dev, enum i2s_dir dir,
				enum i2s_trigger_cmd cmd)
{
	struct i2s_ambiq_data *data = dev->data;
	const struct i2s_ambiq_cfg *cfg = dev->config;
	bool configured = false;
	bool cmd_allowed;

	/* This driver does not use the I2S_STATE_NOT_READY value.
	 * Instead, if a given stream is not configured, the respective
	 * flag (tx_configured or rx_configured) is cleared.
	 */
	if (dir == I2S_DIR_BOTH) {
		configured = data->tx.is_configured && data->rx.is_configured;
	} else if (dir == I2S_DIR_TX) {
		configured = data->tx.is_configured;
	} else if (dir == I2S_DIR_RX) {
		configured = data->rx.is_configured;
	}

	if (!configured) {
		LOG_ERR("Device is not configured");
		return -EIO;
	}

	if (dir == I2S_DIR_BOTH && ((data->tx.cfg.format != data->rx.cfg.format) ||
	    (data->tx.cfg.block_size != data->rx.cfg.block_size)) ){
		LOG_ERR("TX and RX configurations are different");
		return -EIO;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		cmd_allowed = (data->tx.state == I2S_STATE_READY) || (data->rx.state == I2S_STATE_READY);
		break;
	case I2S_TRIGGER_STOP:
	case I2S_TRIGGER_DRAIN:
		cmd_allowed = (data->tx.state == I2S_STATE_RUNNING) || (data->rx.state == I2S_STATE_RUNNING);
		break;
	case I2S_TRIGGER_DROP:
		cmd_allowed = configured;
		break;
	case I2S_TRIGGER_PREPARE:
		cmd_allowed = (data->tx.state == I2S_STATE_ERROR) || (data->rx.state == I2S_STATE_ERROR);
		break;
	default:
		LOG_ERR("Invalid trigger: %d", cmd);
		return -EINVAL;
	}

	if (!cmd_allowed) {
		return -EIO;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		if((dir == I2S_DIR_RX) || (I2S_DIR_BOTH)) {
		}
		if((dir == I2S_DIR_TX) || (I2S_DIR_BOTH)) {
		}
		break;

	case I2S_TRIGGER_STOP:
		if((dir == I2S_DIR_RX) || (I2S_DIR_BOTH)) {
			data->rx.state = I2S_STATE_STOPPING;
		}
		if((dir == I2S_DIR_TX) || (I2S_DIR_BOTH)) {
			data->tx.state = I2S_STATE_STOPPING;
		}
		break;

	case I2S_TRIGGER_DRAIN:
		if((dir == I2S_DIR_RX) || (I2S_DIR_BOTH)) {
			data->rx.state = I2S_STATE_STOPPING;
		}
		if((dir == I2S_DIR_TX) || (I2S_DIR_BOTH)) {
			data->tx.state = I2S_STATE_STOPPING;
		}
		/* If only RX is active, DRAIN is equivalent to STOP. */
		break;

	case I2S_TRIGGER_DROP:
		if ((data->rx.state != I2S_STATE_READY) || (data->tx.state != I2S_STATE_READY)) {
			/* stop rx, tx, or both */
		}
	case I2S_TRIGGER_PREPARE:
		purge_queue(dev, dir);
		if((dir == I2S_DIR_RX) || (I2S_DIR_BOTH)) {
			data->rx.state = I2S_STATE_READY;
		}
		if((dir == I2S_DIR_TX) || (I2S_DIR_BOTH)) {
			data->tx.state = I2S_STATE_READY;
		}
		break;

	default:
		LOG_ERR("Invalid trigger: %d", cmd);
		return -EINVAL;
	}

	return 0;
}

static int i2s_ambiq_read(const struct device *dev, void **mem_block,
			size_t *size)
{
	struct i2s_ambiq_data *const data = dev->data;
	int ret;

	if (data->rx.state == I2S_STATE_NOT_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	if (data->rx.state != I2S_STATE_ERROR) {
		ret = k_sem_take(&data->rx.sem,
				 SYS_TIMEOUT_MS(data->rx.cfg.timeout));
		if (ret < 0) {
			return ret;
		}
	}

	/* Get data from the beginning of RX queue */
	ret = queue_get(&data->rx.mem_block_queue, mem_block, size);
	if (ret < 0) {
		return -EIO;
	}

	return 0;
}

static int i2s_ambiq_write(const struct device *dev, void *mem_block,
			 size_t size)
{
	struct i2s_ambiq_data *const data = dev->data;
	int ret;

	if (data->tx.state != I2S_STATE_RUNNING &&
	    data->tx.state != I2S_STATE_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	ret = k_sem_take(&data->tx.sem,
			 SYS_TIMEOUT_MS(data->tx.cfg.timeout));
	if (ret < 0) {
		return ret;
	}

	/* Add data to the end of the TX queue */
	queue_put(&data->tx.mem_block_queue, mem_block, size);

	return 0;
}

static void i2s_ambiq_isr(const struct device *dev)
{
	struct i2s_ambiq_data *data = dev->data;
	const struct i2s_ambiq_cfg *cfg = dev->config;
	uint32_t ui32Status = 0;

    am_hal_i2s_interrupt_status_get(data->ambiq_cfg.i2s_handle, &ui32Status, true);
    am_hal_i2s_interrupt_clear(data->ambiq_cfg.i2s_handle, ui32Status);

    /* I2S interrupt service */
    am_hal_i2s_interrupt_service(data->ambiq_cfg.i2s_handle, ui32Status, &data->ambiq_cfg.i2s_config);
}

static int i2s_ambiq_initialize(const struct device *dev)
{
	struct i2s_ambiq_data *data = dev->data;
	const struct i2s_ambiq_config *cfg = dev->config;
	int ret = 0;

	uint32_t module = (cfg->base - I2S0_BASE) / cfg->size;

	ret = k_sem_init(&data->rx.sem, 0, CONFIG_I2S_AMBIQ_RX_BLOCK_COUNT);

	if(ret) {
		printk("\n\n*** failed init k_sem_init(&data->rx.sem, 0, CONFIG_I2S_AMBIQ_RX_BLOCK_COUNT) ***\n\n");
	}
	k_sem_init(&data->tx.sem, CONFIG_I2S_AMBIQ_TX_BLOCK_COUNT - 1,
		   CONFIG_I2S_AMBIQ_TX_BLOCK_COUNT);

    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HFRC2_START, false);
    k_sleep(K_MSEC(500));

 	/* enable EXTCLK32M */
     if ( (eAM_HAL_I2S_CLKSEL_XTHS_EXTREF_CLK <= data->ambiq_cfg.i2s_config.eClock)
			&& (data->ambiq_cfg.i2s_config.eClock <= eAM_HAL_I2S_CLKSEL_XTHS_500KHz) )
    {
        am_hal_mcuctrl_control_arg_t ctrlArgs = g_amHalMcuctrlArgDefault;
        ctrlArgs.ui32_arg_hfxtal_user_mask  = 1 << (AM_HAL_HCXTAL_II2S_BASE_EN + module);
        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_NORMAL, (void *)&ctrlArgs);
        k_sleep(K_MSEC(200));
    }

    ret = am_hal_i2s_initialize(module, &data->ambiq_cfg.i2s_handle);
	if (ret != 0) {
		LOG_ERR("Error - am_hal_i2s_initialize Failed\n");
		return ret;
	}

    ret = am_hal_i2s_power_control(data->ambiq_cfg.i2s_handle, AM_HAL_I2S_POWER_ON, false);
	if (ret != 0) {
		LOG_ERR("Error - am_hal_i2s_power_control Failed\n");
		goto end;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("Error - Failed  to config I2S pins\n");
		goto end;
	}

    ret = am_hal_i2s_configure(data->ambiq_cfg.i2s_handle, &data->ambiq_cfg.i2s_config);
	if (ret != 0) {
		LOG_ERR("Error - am_hal_i2s_configure Failed\n");
		goto end;
	}

    ret = am_hal_i2s_enable(data->ambiq_cfg.i2s_handle);
	if (ret != 0) {
		LOG_ERR("Error - am_hal_i2s_enable Failed\n");
		goto end;
	}

    ret = am_hal_i2s_dma_configure(data->ambiq_cfg.i2s_handle, &data->ambiq_cfg.i2s_config, &data->ambiq_cfg.i2s_transfer_config);
	if (ret != 0) {
		LOG_ERR("Error - am_hal_i2s_dma_configure Failed\n");
		goto end;
	}

	cfg->irq_config_func();

end:
	if (ret < 0) {
		am_hal_i2s_deinitialize(data->ambiq_cfg.i2s_handle);
	}

	return ret;
}

static const struct i2s_driver_api i2s_ambiq_driver_api = {
	.configure = i2s_ambiq_configure,
	.config_get = i2s_ambiq_config_get,
	.read = i2s_ambiq_read,
	.write = i2s_ambiq_write,
	.trigger = i2s_ambiq_trigger,
};

#ifdef CONFIG_PM_DEVICE
static int i2s_ambiq_pm_action(const struct device *dev, enum pm_device_action action)
{
	struct i2s_ambiq_data *data = dev->data;
	uint32_t ret = 0;
	am_hal_sysctrl_power_state_e status;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		status = AM_HAL_SYSCTRL_WAKE;
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		status = AM_HAL_SYSCTRL_NORMALSLEEP;
		break;
	default:
		return -ENOTSUP;
	}

	ret = am_hal_i2s_power_control(data->ambiq_cfg.i2s_handle, status, true);

	if (ret != 0) {
		LOG_ERR("am_hal_i2s_power_control failed: %d", ret);
		return -EPERM;
	} else {
		return 0;
	}
}
#endif /* CONFIG_PM_DEVICE */

#define AMBIQ_I2S_INIT(n)                                                      \
	PINCTRL_DT_INST_DEFINE(n);                                                 \
	static int pwr_on_ambiq_i2s_##n(void)                                      \
	{                                                                          \
		uint32_t addr = DT_REG_ADDR(DT_INST_PHANDLE(n, ambiq_pwrcfg)) +        \
				DT_INST_PHA(n, ambiq_pwrcfg, offset);                          \
		sys_write32((sys_read32(addr)                                          \
					| DT_INST_PHA(n, ambiq_pwrcfg, mask)), addr);              \
		k_busy_wait(10);                                                       \
		return 0;                                                              \
	}                                                                          \
	static struct queue_item rx_ring_buf##n[CONFIG_I2S_AMBIQ_RX_BLOCK_COUNT + 3]; \
	static struct queue_item tx_ring_buf##n[CONFIG_I2S_AMBIQ_TX_BLOCK_COUNT + 3]; \
	static struct i2s_ambiq_data i2s_ambiq_data##n =                           \
	{                                                                          \
		.rx =                                                                  \
		{                                                                      \
			.mem_block_queue.buf = rx_ring_buf##n,                             \
			.mem_block_queue.len = ARRAY_SIZE(rx_ring_buf##n),                 \
		},                                                                     \
		.tx =                                                                  \
		{                                                                      \
			.mem_block_queue.buf = tx_ring_buf##n,                             \
			.mem_block_queue.len = ARRAY_SIZE(tx_ring_buf##n)                  \
		},                                                                     \
		.ambiq_cfg =                                                           \
		{                                                                      \
			.i2s_config =                                                      \
			{                                                                  \
				.eClock	= eAM_HAL_I2S_CLKSEL_HFRC_6MHz,                        \
				.eDiv3	= 0,                                                   \
				.eASRC	= 0,                                                   \
				.eMode	 = AM_HAL_I2S_IO_MODE_SLAVE,                           \
				.eXfer	= AM_HAL_I2S_XFER_RXTX,                                \
				.eData	= &g_sI2SDataConfig,                                   \
				.eIO	= &g_sI2SIOConfig,                                     \
			},                                                                 \
			.i2s_transfer_config =                                             \
			{                                                                  \
				.ui32RxTotalCount         = CONFIG_I2S_AMBIQ_RX_BLOCK_COUNT,   \
				.ui32RxTargetAddr         = 0x0,                               \
				.ui32RxTargetAddrReverse  = 0x0,                               \
				.ui32TxTotalCount         = CONFIG_I2S_AMBIQ_RX_BLOCK_COUNT,   \
				.ui32TxTargetAddr         = 0x0,                               \
				.ui32TxTargetAddrReverse  = 0x0,                               \
			},                                                                 \
		},                                                                     \
	};                                                                         \
	static void i2s_ambiq_irq_config_func_##n(void)                            \
	{                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n),                                           \
				DT_INST_IRQ(n, priority),                                      \
				i2s_ambiq_isr, DEVICE_DT_INST_GET(n), 0);                      \
		irq_enable(DT_INST_IRQN(n));                                           \
	}                                                                          \
	static const struct i2s_ambiq_config i2s_ambiq_cfg##n = {                  \
		.base = DT_INST_REG_ADDR(n),                                           \
		.size = DT_INST_REG_SIZE(n),                                           \
		.clock_freq = eAM_HAL_I2S_CLKSEL_HFRC_6MHz,                            \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                             \
		.pwr_func = pwr_on_ambiq_i2s_##n,                                      \
		.irq_config_func = i2s_ambiq_irq_config_func_##n};                     \
	PM_DEVICE_DT_INST_DEFINE(n, i2s_ambiq_pm_action);                          \
	DEVICE_DT_INST_DEFINE(n, i2s_ambiq_initialize, PM_DEVICE_DT_INST_GET(n),   \
			&i2s_ambiq_data##n, &i2s_ambiq_cfg##n, POST_KERNEL,                \
			CONFIG_I2S_INIT_PRIORITY, &i2s_ambiq_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_I2S_INIT)
