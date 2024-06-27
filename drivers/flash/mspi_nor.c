/*
 * Copyright (c) 2024, Ambiq Micro Inc. <www.ambiq.com>
 *
 * This driver strictly follows jedec216, anything that is
 * not in the document should not be added.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT jedec_mspi_nor

#include <errno.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys_clock.h>
#include <zephyr/pm/device.h>

#include "spi_nor.h"
#include "jesd216.h"
#include "flash_priv.h"

LOG_MODULE_REGISTER(mspi_nor, CONFIG_FLASH_LOG_LEVEL);

/* Device Power Management Notes
 *
 * These flash devices have several modes during operation:
 * * When CSn is asserted (during a MSPI operation) the device is
 *   active.
 * * When CSn is deasserted the device enters a standby mode.
 * * Some devices support a Deep Power-Down mode which reduces current
 *   to as little as 0.1% of standby.
 *
 * The power reduction from DPD is sufficient to warrant allowing its
 * use even in cases where Zephyr's device power management is not
 * available.  This is selected through the MSPI_NOR_IDLE_IN_DPD
 * Kconfig option.
 *
 * When mapped to the Zephyr Device Power Management states:
 * * PM_DEVICE_STATE_ACTIVE covers both active and standby modes;
 * * PM_DEVICE_STATE_SUSPENDED, and PM_DEVICE_STATE_OFF all correspond to
 *   deep-power-down mode.
 */

#if CONFIG_SOC_FAMILY_AMBIQ
#include "mspi_ambiq.h"
typedef struct mspi_ambiq_timing_cfg mspi_timing_cfg;
typedef enum mspi_ambiq_timing_param mspi_timing_param;

#else
typedef struct mspi_timing_cfg mspi_timing_cfg;
typedef enum mspi_timing_param mspi_timing_param;
#define TIMING_CFG_GET_RX_DUMMY(cfg)
#define TIMING_CFG_SET_RX_DUMMY(cfg, num)
#endif

#define MSPI_NOR_MAX_ADDR_WIDTH 4
#define MSPI_NOR_WRITE_SIZE     1
#define MSPI_NOR_ERASE_VALUE    0xFF

#define ANY_INST_HAS_TRUE_(idx, bool_prop)	                                                   \
	COND_CODE_1(DT_INST_PROP(idx, bool_prop), (1,), ())

#define ANY_INST_HAS_TRUE(bool_prop)	                                                           \
	COND_CODE_1(IS_EMPTY(DT_INST_FOREACH_STATUS_OKAY_VARGS(ANY_INST_HAS_TRUE_, bool_prop)),    \
			     (0), (1))

#define ANY_INST_HAS_PROP_(idx, prop_name)	                                                   \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, prop_name), (1,), ())
#define ANY_INST_HAS_PROP(prop_name)		                                                   \
	COND_CODE_1(IS_EMPTY(DT_INST_FOREACH_STATUS_OKAY_VARGS(ANY_INST_HAS_PROP_, prop_name)),    \
			     (0), (1))

#define ANY_INST_HAS_BP ANY_INST_HAS_PROP(has_bp)
#define ANY_INST_HAS_DPD ANY_INST_HAS_TRUE(has_dpd)
#define ANY_INST_HAS_T_EXIT_DPD ANY_INST_HAS_PROP(t_exit_dpd)
#define ANY_INST_HAS_DPD_WAKEUP_SEQUENCE ANY_INST_HAS_PROP(dpd_wakeup_sequence)
#define ANY_INST_HAS_RESET_GPIOS ANY_INST_HAS_PROP(reset_gpios)
#define ANY_INST_HAS_WP_GPIOS ANY_INST_HAS_PROP(wp_gpios)
#define ANY_INST_HAS_HOLD_GPIOS ANY_INST_HAS_PROP(hold_gpios)

#define DEV_CFG(_dev_) ((const struct mspi_nor_config * const) (_dev_)->config)

/* Build-time data associated with the device. */
struct mspi_nor_config {

	struct flash_parameters             flash_param;

	const struct device                 *bus;
	struct mspi_dev_id                  dev_id;
	struct mspi_dev_cfg                 serial_cfg;
	struct mspi_dev_cfg                 tar_dev_cfg;
	struct mspi_xip_cfg                 tar_xip_cfg;
	struct mspi_scramble_cfg            tar_scramble_cfg;

	mspi_timing_cfg                     tar_timing_cfg;
	mspi_timing_param                   timing_cfg_mask;

	bool                                sw_multi_periph;

	/* Expected JEDEC ID, from jedec-id property */
	uint8_t jedec_id[SPI_NOR_MAX_ID_LEN];

#ifdef CONFIG_MSPI_NOR_SFDP_MINIMAL
	/* Size of device in bytes, from size property */
	uint32_t flash_size;

	/* Flash page layout can be determined from devicetree. */
	struct flash_pages_layout layout;

	/* legacy erase types. */
	struct jesd216_erase_type erase_types[2];
#endif /* CONFIG_MSPI_NOR_SFDP_MINIMAL */

#if defined(CONFIG_MSPI_NOR_SFDP_DEVICETREE)
	/* Length of BFP structure, in 32-bit words. */
	uint8_t bfp_len;

	/* Pointer to the BFP table as read from the device
	 * (little-endian stored words), from sfdp-bfp property
	 */
	const struct jesd216_bfp *bfp;
#endif /* CONFIG_MSPI_NOR_SFDP_DEVICETREE */

#if ANY_INST_HAS_BP
	/* Optional bits in SR to be cleared on startup.
	 *
	 * This information cannot be derived from SFDP.
	 */
	uint8_t has_bp;
#endif

#if ANY_INST_HAS_RESET_GPIOS
	const struct gpio_dt_spec reset;
#endif

#if ANY_INST_HAS_WP_GPIOS
	/* The write-protect GPIO (wp-gpios) */
	const struct gpio_dt_spec wp;
#endif

#if ANY_INST_HAS_HOLD_GPIOS
	/* The hold GPIO (hold-gpios) */
	const struct gpio_dt_spec hold;
#endif

#if ANY_INST_HAS_DPD
	uint16_t dpd_enter_cmd;
	uint16_t dpd_exit_cmd;

	uint16_t t_enter_dpd; /* in microseconds */
	uint16_t t_dpdd_ms;   /* in microseconds */
#if ANY_INST_HAS_T_EXIT_DPD
	uint16_t t_exit_dpd;  /* in microseconds */
#endif
#endif

#if ANY_INST_HAS_DPD_WAKEUP_SEQUENCE
	uint16_t t_crdp_ms; /* in microseconds */
	uint16_t t_rdp_ms;  /* in microseconds */
#endif

	/* exist flags for dts opt-ins */
	bool dpd_exist:1;
	bool dpd_wakeup_seq_exist:1;
	bool reset_gpios_exist:1;
	bool wp_gpios_exist:1;
	bool hold_gpios_exist:1;
};

/**
 * struct mspi_nor_data - Structure for defining the MSPI NOR access
 * @sem: The semaphore to access to the flash
 */
struct mspi_nor_data {
	struct k_sem                        sem;

	struct mspi_dev_cfg                 dev_cfg;
	struct mspi_xip_cfg                 xip_cfg;
	struct mspi_scramble_cfg            scramble_cfg;
	mspi_timing_cfg                     timing_cfg;
	struct mspi_xfer                    trans;
	struct mspi_xfer_packet             packet;

#if ANY_INST_HAS_DPD
	/* Low 32-bits of uptime counter at which device last entered
	 * deep power-down.
	 */
	uint32_t ts_enter_dpd;
#endif

#if defined(CONFIG_MSPI_NOR_SFDP_RUNTIME) || defined(CONFIG_MSPI_NOR_SFDP_DEVICETREE)

	struct jesd216_erase_type erase_types[JESD216_NUM_ERASE_TYPES];

	/* Size of flash, in bytes */
	uint32_t flash_size;

	struct flash_pages_layout layout;

	enum jesd216_wen_sel wen_sel;

	enum jesd216_qer qer;

	enum jesd216_qen_seq qen_seq;

	enum jesd216_qdis_seq qdis_seq;

	enum jesd216_entr_4B_seq entr_4B_seq;

	enum jesd216_exit_4B_seq exit_4B_seq;

	enum jesd216_sw_rst_seq sw_rst_seq;

	enum jesd216_oer oer;

	enum jesd216_oen_seq oen_seq;

	enum jesd216_odis_seq odis_seq;

#if ANY_INST_HAS_DPD
	uint16_t dpd_enter_cmd;
	uint16_t dpd_exit_cmd;
#if ANY_INST_HAS_T_EXIT_DPD
	uint16_t t_exit_dpd;  /* in microseconds */
#endif
#endif

#endif /* CONFIG_MSPI_NOR_SFDP_RUNTIME */
};

struct cmd_info {
	enum mspi_xfer_direction    dir;
	uint16_t                    cmd;
	uint8_t                     cmd_len;
	uint8_t                     addr_len;
	uint8_t                     dummy_cycles;
	uint8_t                     min_data_len;
};

#ifdef CONFIG_MSPI_NOR_SFDP_MINIMAL
/* The historically supported erase sizes. */
static const struct jesd216_erase_type minimal_erase_types[JESD216_NUM_ERASE_TYPES] = {
	{
		.cmd = SPI_NOR_CMD_BE,
		.exp = 16,
	},
	{
		.cmd = SPI_NOR_CMD_SE,
		.exp = 12,
	},
};
#endif /* CONFIG_MSPI_NOR_SFDP_MINIMAL */

/* Register writes should be ready extremely quickly */
#define WAIT_READY_REGISTER K_NO_WAIT
/* Page writes range from sub-ms to 10ms */
#define WAIT_READY_WRITE K_TICKS(1)
/* Erases can range from 45ms to 240sec */
#define WAIT_READY_ERASE K_MSEC(50)

static int mspi_nor_write_protection_set(const struct device *dev,
					bool write_protect);

/* Get pointer to array of supported erase types.  Static const for
 * minimal, data for runtime and devicetree.
 */
static inline const struct jesd216_erase_type *
dev_erase_types(const struct device *dev)
{
#ifdef CONFIG_MSPI_NOR_SFDP_MINIMAL
	return minimal_erase_types;
#else /* CONFIG_MSPI_NOR_SFDP_MINIMAL */
	const struct mspi_nor_data *data = dev->data;

	return data->erase_types;
#endif /* CONFIG_MSPI_NOR_SFDP_MINIMAL */
}

/* Get the size of the flash device. */
static inline uint32_t dev_flash_size(const struct device *dev)
{
#ifndef CONFIG_MSPI_NOR_SFDP_MINIMAL
	const struct mspi_nor_data *data = dev->data;

	return data->flash_size;
#else
	const struct mspi_nor_config *cfg = dev->config;

	return cfg->flash_size;
#endif /* !CONFIG_MSPI_NOR_SFDP_MINIMAL */
}

/* Get the flash device page size.  Constant for minimal, data for
 * runtime and devicetree.
 */
static inline uint16_t dev_page_size(const struct device *dev)
{
#ifndef CONFIG_MSPI_NOR_SFDP_MINIMAL
	const struct mspi_nor_data *data = dev->data;

	return data->layout.pages_size;
#else
	const struct mspi_nor_config *cfg = dev->config;

	return cfg->layout.pages_size;
#endif /* !CONFIG_MSPI_NOR_SFDP_MINIMAL */
}

/* Capture the time at which the device entered deep power-down. */
static inline void record_entered_dpd(const struct device *const dev)
{
#if ANY_INST_HAS_DPD
	const struct mspi_nor_config *const driver_config = dev->config;

	if (driver_config->dpd_exist) {
		struct mspi_nor_data *const driver_data = dev->data;

		driver_data->ts_enter_dpd = k_uptime_get_32();
	}
#else
	ARG_UNUSED(dev);
#endif
}

/* Check the current time against the time DPD was entered and delay
 * until it's ok to initiate the DPD exit process.
 */
static inline void delay_until_exit_dpd_ok(const struct device *const dev)
{
#if ANY_INST_HAS_DPD
	const struct mspi_nor_config *const driver_config = dev->config;

	if (driver_config->dpd_exist) {
		struct mspi_nor_data *const driver_data = dev->data;
		int32_t since = (int32_t)(k_uptime_get_32() - driver_data->ts_enter_dpd);

		/* If the time is negative the 32-bit counter has wrapped,
		 * which is certainly long enough no further delay is
		 * required.  Otherwise we have to check whether it's been
		 * long enough taking into account necessary delays for
		 * entering and exiting DPD.
		 */
		if (since >= 0) {
			/* Subtract time required for DPD to be reached */
			since -= driver_config->t_enter_dpd;

			/* Subtract time required in DPD before exit */
			since -= driver_config->t_dpdd_ms;

			/* If the adjusted time is negative we have to wait
			 * until it reaches zero before we can proceed.
			 */
			if (since < 0) {
				k_sleep(K_MSEC((uint32_t)-since));
			}
		}
	}
#else
	ARG_UNUSED(dev);
#endif /* ANY_INST_HAS_DPD */
}

/*
 * @brief Send an MSPI command
 *
 * @param dev Device struct
 * @param xfer_mode PIO or DMA
 * @param cmd_info Information regarding how to send the command
 * @param addr The address to send
 * @param data_buf The buffer to store or read the value
 * @param length The size of the buffer
 * @return 0 on success, negative errno code otherwise
 */
static int mspi_nor_access(const struct device *dev,
			   enum mspi_xfer_mode xfer_mode,
			   struct cmd_info *cmd,
			   off_t addr, const void *data_buf, size_t length)
{
	const struct mspi_nor_config *cfg = dev->config;
	struct mspi_nor_data *data = dev->data;
	int ret;

	data->packet.dir              = cmd->dir;
	data->packet.cmd              = cmd->cmd;
	data->packet.address          = addr;
	data->packet.data_buf         = (uint8_t *)data_buf;
	data->packet.num_bytes        = MIN(cmd->min_data_len, length);

	data->trans.async             = false;
	data->trans.xfer_mode         = xfer_mode;
	data->trans.tx_dummy          = cmd->dummy_cycles;
	data->trans.cmd_length        = cmd->cmd_len;
	data->trans.addr_length       = cmd->addr_len;
	data->trans.hold_ce           = false;
	data->trans.packets           = &data->packet;
	data->trans.num_packet        = 1;
	data->trans.timeout           = 10;

	ret = mspi_transceive(cfg->bus, &cfg->dev_id, (const struct mspi_xfer *)&data->trans);
	if (ret) {
		LOG_ERR("MSPI read transaction failed with code: %d/%u", ret, __LINE__);
		return -EIO;
	}
	return ret;
}

/**
 * @brief Wait until the flash is ready
 *
 * @note The device must be externally acquired before invoking this
 * function.
 *
 * This function should be invoked after every ERASE, PROGRAM, or
 * WRITE_STATUS operation before continuing.  This allows us to assume
 * that the device is ready to accept new commands at any other point
 * in the code.
 *
 * @param dev The device structure
 * @param poll_delay Duration between polls of status register
 * @return 0 on success, negative errno code otherwise
 */
static int mspi_nor_wait_until_ready(const struct device *dev, k_timeout_t poll_delay)
{
	struct cmd_info rdsr = {
		.dir = MSPI_RX,
		.cmd = SPI_NOR_CMD_RDSR,
		.cmd_len = 1,
		.addr_len = 0,
		.dummy_cycles = 0,
		.min_data_len = 1,
	};
	int ret;
	uint8_t reg;

	ARG_UNUSED(poll_delay);

	while (true) {
		ret = mspi_nor_access(dev, MSPI_PIO, &rdsr, 0, &reg, sizeof(reg));
		/* Exit on error or no longer WIP */
		if (ret || !(reg & SPI_NOR_WIP_BIT)) {
			break;
		}
#ifdef CONFIG_MSPI_NOR_SLEEP_WHILE_WAITING_UNTIL_READY
		/* Don't monopolise the CPU while waiting for ready */
		k_sleep(poll_delay);
#endif /* CONFIG_MSPI_NOR_SLEEP_WHILE_WAITING_UNTIL_READY */
	}
	return ret;
}

#if defined(CONFIG_MSPI_NOR_SFDP_RUNTIME) || defined(CONFIG_FLASH_JESD216_API)
/*
 * @brief Read content from the SFDP hierarchy
 *
 * @note The device must be externally acquired before invoking this
 * function.
 *
 * @param dev Device struct
 * @param addr The address to send
 * @param data The buffer to store or read the value
 * @param length The size of the buffer
 * @return 0 on success, negative errno code otherwise
 */
static int read_sfdp(const struct device *const dev,
		     off_t addr, void *data, size_t length)
{
	/* READ_SFDP requires a 24-bit address followed by a 8 dummy cycles. */
	struct cmd_info r_sfdp = {
		.dir = MSPI_RX,
		.cmd = JESD216_CMD_READ_SFDP,
		.cmd_len = 1,
		.addr_len = 3,
		.dummy_cycles = 8,
		.min_data_len = 1,
	};

	return mspi_nor_access(dev, MSPI_DMA, &r_sfdp, addr, data, length);
}
#endif /* CONFIG_MSPI_NOR_SFDP_RUNTIME */

static int enter_dpd(const struct device *const dev)
{
	int ret = 0;
#if ANY_INST_HAS_DPD
	const struct mspi_nor_config *cfg = dev->config;
	struct cmd_info dpd = {
		.dir = MSPI_TX,
		.cmd = SPI_NOR_CMD_DPD,
		.cmd_len = 1,
		.addr_len = 0,
		.dummy_cycles = 0,
		.min_data_len = 0,
	};

	if (cfg->dpd_exist) {
		ret = mspi_nor_access(dev, MSPI_PIO, &dpd, 0, NULL, 0);
		if (ret == 0) {
			record_entered_dpd(dev);
		}
	}
#endif
	return ret;
}

static int exit_dpd(const struct device *const dev)
{
	int ret = 0;
#if ANY_INST_HAS_DPD
	const struct mspi_nor_config *cfg = dev->config;
	struct mspi_nor_data *data = dev->data;
	struct mspi_dev_cfg bkp = data->dev_cfg;

	if (cfg->dpd_exist) {
		delay_until_exit_dpd_ok(dev);

		if (cfg->dpd_wakeup_seq_exist) {
#if ANY_INST_HAS_DPD_WAKEUP_SEQUENCE
			struct cmd_info dummy = {
				.dir = MSPI_TX,
				.cmd = 0x00,
				.cmd_len = 1,
				.addr_len = 0,
				.dummy_cycles = 0,
				.min_data_len = 0,
			};
			/* Assert CSn and wait for tCRDP by
			 * sending a dummy command with a fixed frequency
			 * to gauaratee a fixed CE interval
			 */
			data->dev_cfg.freq = 50000000;
			mspi_dev_config(cfg->bus, &cfg->dev_id, MSPI_DEVICE_CONFIG_FREQUENCY,
					(const struct mspi_dev_cfg *)&data->dev_cfg);
			ret = mspi_nor_access(dev, MSPI_PIO, &dummy, 0, NULL, 0);

			/* Deassert CSn and wait for tRDP */
			k_sleep(K_MSEC(cfg->t_rdp_ms));

			data->dev_cfg = bkp;
			mspi_dev_config(cfg->bus, &cfg->dev_id, MSPI_DEVICE_CONFIG_FREQUENCY,
					(const struct mspi_dev_cfg *)&data->dev_cfg);
#endif /* ANY_INST_HAS_DPD_WAKEUP_SEQUENCE */
		} else {
			struct cmd_info rdpd = {
				.dir = MSPI_TX,
				.cmd = SPI_NOR_CMD_RDPD,
				.cmd_len = 1,
				.addr_len = 0,
				.dummy_cycles = 0,
				.min_data_len = 0,
			};
			ret = mspi_nor_access(dev, MSPI_PIO, &rdpd, 0, NULL, 0);

#if ANY_INST_HAS_T_EXIT_DPD
			if (ret == 0) {
				if (cfg->dpd_exist) {
					k_sleep(K_MSEC(cfg->t_exit_dpd));
				}
			}
#endif /* T_EXIT_DPD */
		}
	}
#endif /* ANY_INST_HAS_DPD */
	return ret;
}

/* Everything necessary to acquire owning access to the device.
 *
 * This means taking the lock and, if necessary, waking the device
 * from deep power-down mode.
 */
static void acquire_device(const struct device *dev)
{
	const struct mspi_nor_config *cfg = dev->config;
	struct mspi_nor_data *data = dev->data;

	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		k_sem_take(&data->sem, K_FOREVER);
	}

	if (cfg->sw_multi_periph) {
		while (mspi_dev_config(cfg->bus, &cfg->dev_id,
				       MSPI_DEVICE_CONFIG_ALL, &data->dev_cfg))
			;
	} else {
		while (mspi_dev_config(cfg->bus, &cfg->dev_id,
				       MSPI_DEVICE_CONFIG_NONE, NULL))
			;
	}

	if (IS_ENABLED(CONFIG_MSPI_NOR_IDLE_IN_DPD)) {
		exit_dpd(dev);
	}
}

/* Everything necessary to release access to the device.
 *
 * This means (optionally) putting the device into deep power-down
 * mode, and releasing the lock.
 */
static void release_device(const struct device *dev)
{
	const struct mspi_nor_config *cfg = dev->config;
	struct mspi_nor_data *data = dev->data;

	if (IS_ENABLED(CONFIG_MSPI_NOR_IDLE_IN_DPD)) {
		enter_dpd(dev);
	}

	while (mspi_get_channel_status(cfg->bus, 0)) {

	}

	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		k_sem_give(&data->sem);
	}
}

/**
 * @brief Read the status register.
 *
 * @note The device must be externally acquired before invoking this
 * function.
 *
 * @param dev Device struct
 *
 * @return the non-negative value of the status register, or an error code.
 */
static int mspi_nor_rdsr(const struct device *dev)
{
	struct cmd_info rdsr = {
		.dir = MSPI_RX,
		.cmd = SPI_NOR_CMD_RDSR,
		.cmd_len = 1,
		.addr_len = 0,
		.dummy_cycles = 0,
		.min_data_len = 1,
	};
	int ret;
	uint8_t reg = 0;

	ret = mspi_nor_access(dev, MSPI_PIO, &rdsr, 0, &reg, sizeof(reg));
	if (ret == 0) {
		ret = reg;
	}

	return ret;
}

/**
 * @brief Write the status register.
 *
 * @note The device must be externally acquired before invoking this
 * function.
 *
 * @param dev Device struct
 * @param sr The new value of the status register
 *
 * @return 0 on success or a negative error code.
 */
static int mspi_nor_wrsr(const struct device *dev,
			uint8_t sr)
{
	struct cmd_info wcmd = {
		.dir = MSPI_TX,
		.cmd = SPI_NOR_CMD_WREN,
		.cmd_len = 1,
		.addr_len = 0,
		.dummy_cycles = 0,
		.min_data_len = 0,
	};
	int ret;

	ret = mspi_nor_access(dev, MSPI_PIO, &wcmd, 0, NULL, 0);
	if (ret != 0) {
		return ret;
	}

	wcmd.cmd = SPI_NOR_CMD_WRSR;
	wcmd.min_data_len = 1;
	ret = mspi_nor_access(dev, MSPI_PIO, &wcmd, 0, &sr, sizeof(sr));
	if (ret != 0) {
		return ret;
	}

	return mspi_nor_wait_until_ready(dev, WAIT_READY_REGISTER);
}

#if defined(CONFIG_FLASH_EX_OP_ENABLED)
static int flash_mspi_nor_ex_op(const struct device *dev, uint16_t code,
			const uintptr_t in, void *out)
{
	struct cmd_info wcmd = {
		.dir = MSPI_TX,
		.cmd = SPI_NOR_CMD_RESET_EN,
		.cmd_len = 1,
		.addr_len = 0,
		.dummy_cycles = 0,
		.min_data_len = 0,
	};
	int ret;

	ARG_UNUSED(in);
	ARG_UNUSED(out);

	acquire_device(dev);

	switch (code) {
	case FLASH_EX_OP_RESET:
		ret = mspi_nor_access(dev, MSPI_PIO, &wcmd, 0, NULL, 0);
		if (ret == 0) {
			wcmd.cmd = SPI_NOR_CMD_RESET_MEM;
			ret = mspi_nor_access(dev, MSPI_PIO, &wcmd, 0, NULL, 0);
		}
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	release_device(dev);
	return ret;
}
#endif

static int mspi_nor_read(const struct device *dev, off_t addr, void *dest,
			size_t size)
{
	//const struct mspi_nor_config *cfg = dev->config;
	struct mspi_nor_data *data = dev->data;
	const size_t flash_size = dev_flash_size(dev);
	/** may need to get specifics from SFDP */
	struct cmd_info rcmd = {
		.dir = MSPI_RX,
		.cmd = SPI_NOR_CMD_READ,
		.cmd_len = 1,
		.addr_len = data->dev_cfg.addr_length,
		.dummy_cycles = 0,
		.min_data_len = 1,
	};
	int ret;

	/* should be between 0 and flash size */
	if ((addr < 0) || ((addr + size) > flash_size)) {
		return -EINVAL;
	}

	acquire_device(dev);
	ret = mspi_nor_access(dev, MSPI_DMA, &rcmd, addr, dest, size);
	release_device(dev);
	return ret;
}

static int mspi_nor_write(const struct device *dev, off_t addr,
			  const void *src,
			  size_t size)
{
	//const struct mspi_nor_config *cfg = dev->config;
	struct mspi_nor_data *data = dev->data;
	const size_t flash_size = dev_flash_size(dev);
	const uint16_t page_size = dev_page_size(dev);
	struct cmd_info wcmd = {
		.dir = MSPI_TX,
		.cmd = SPI_NOR_CMD_WREN,
		.cmd_len = 1,
		.addr_len = 0,
		.dummy_cycles = 0,
		.min_data_len = 0,
	};
	int ret;

	/* should be between 0 and flash size */
	if ((addr < 0) || ((size + addr) > flash_size)) {
		return -EINVAL;
	}

	acquire_device(dev);
	ret = mspi_nor_write_protection_set(dev, false);
	if (ret == 0) {
		while (size > 0) {
			size_t to_write = size;

			/* Don't write more than a page. */
			if (to_write >= page_size) {
				to_write = page_size;
			}

			/* Don't write across a page boundary */
			if (((addr + to_write - 1U) / page_size)
			!= (addr / page_size)) {
				to_write = page_size - (addr % page_size);
			}

			ret = mspi_nor_access(dev, MSPI_PIO, &wcmd, 0, NULL, 0);
			if (ret != 0) {
				break;
			}

			wcmd.cmd = SPI_NOR_CMD_PP;
			wcmd.addr_len = data->dev_cfg.addr_length;
			wcmd.min_data_len = 1;
			ret = mspi_nor_access(dev, MSPI_DMA, &wcmd, addr, src, to_write);
			if (ret != 0) {
				break;
			}

			size -= to_write;
			src = (const uint8_t *)src + to_write;
			addr += to_write;

			ret = mspi_nor_wait_until_ready(dev, WAIT_READY_WRITE);
			if (ret != 0) {
				break;
			}
		}
	}

	int ret2 = mspi_nor_write_protection_set(dev, true);

	if (!ret) {
		ret = ret2;
	}

	release_device(dev);
	return ret;
}

static int mspi_nor_erase(const struct device *dev, off_t addr, size_t size)
{
	//const struct mspi_nor_config *cfg = dev->config;
	struct mspi_nor_data *data = dev->data;
	const size_t flash_size = dev_flash_size(dev);
	struct cmd_info wcmd = {
		.dir = MSPI_TX,
		.cmd = SPI_NOR_CMD_WREN,
		.cmd_len = 1,
		.addr_len = 0,
		.dummy_cycles = 0,
		.min_data_len = 0,
	};
	int ret;

	/* erase area must be subregion of device */
	if ((addr < 0) || ((size + addr) > flash_size)) {
		return -EINVAL;
	}

	/* address must be sector-aligned */
	if (!SPI_NOR_IS_SECTOR_ALIGNED(addr)) {
		return -EINVAL;
	}

	/* size must be a multiple of sectors */
	if ((size % SPI_NOR_SECTOR_SIZE) != 0) {
		return -EINVAL;
	}

	acquire_device(dev);
	ret = mspi_nor_write_protection_set(dev, false);

	while ((size > 0) && (ret == 0)) {
		ret = mspi_nor_access(dev, MSPI_PIO, &wcmd, 0, NULL, 0);
		if (ret) {
			break;
		}

		if (size == flash_size) {
			/* chip erase */
			wcmd.cmd = SPI_NOR_CMD_CE;
			ret = mspi_nor_access(dev, MSPI_PIO, &wcmd, 0, NULL, 0);
			size -= flash_size;
		} else {
			const struct jesd216_erase_type *erase_types =
				dev_erase_types(dev);
			const struct jesd216_erase_type *bet = NULL;

			for (uint8_t ei = 0; ei < JESD216_NUM_ERASE_TYPES; ++ei) {
				const struct jesd216_erase_type *etp =
					&erase_types[ei];

				if ((etp->exp != 0)
				    && SPI_NOR_IS_ALIGNED(addr, etp->exp)
				    && (size >= BIT(etp->exp))
				    && ((bet == NULL)
					|| (etp->exp > bet->exp))) {
					bet = etp;
				}
			}
			if (bet != NULL) {
				wcmd.cmd = bet->cmd;
				/** Todo: need to get address size from SFDP */
				wcmd.addr_len = data->dev_cfg.addr_length;
				ret = mspi_nor_access(dev, MSPI_PIO, &wcmd, addr, NULL, 0);
				addr += BIT(bet->exp);
				size -= BIT(bet->exp);
			} else {
				LOG_DBG("Can't erase %zu at 0x%lx",
					size, (long)addr);
				ret = -EINVAL;
			}
		}
		if (ret != 0) {
			break;
		}

		ret = mspi_nor_wait_until_ready(dev, WAIT_READY_ERASE);
	}

	int ret2 = mspi_nor_write_protection_set(dev, true);

	if (!ret) {
		ret = ret2;
	}

	release_device(dev);

	return ret;
}

/* @note The device must be externally acquired before invoking this
 * function.
 */
static int mspi_nor_write_protection_set(const struct device *dev,
					bool write_protect)
{
	struct cmd_info wcmd = {
		.dir = MSPI_TX,
		.cmd = write_protect ? SPI_NOR_CMD_WRDI : SPI_NOR_CMD_WREN,
		.cmd_len = 1,
		.addr_len = 0,
		.dummy_cycles = 0,
		.min_data_len = 0,
	};
	int ret;

#if ANY_INST_HAS_WP_GPIOS
	if (DEV_CFG(dev)->wp_gpios_exist && write_protect == false) {
		gpio_pin_set_dt(&(DEV_CFG(dev)->wp), 0);
	}
#endif

	ret = mspi_nor_access(dev, MSPI_PIO, &wcmd, 0, NULL, 0);


#if ANY_INST_HAS_WP_GPIOS
	if (DEV_CFG(dev)->wp_gpios_exist && write_protect == true) {
		gpio_pin_set_dt(&(DEV_CFG(dev)->wp), 1);
	}
#endif

	return ret;
}

#if defined(CONFIG_FLASH_JESD216_API) || defined(CONFIG_MSPI_NOR_SFDP_RUNTIME)

static int mspi_nor_sfdp_read(const struct device *dev, off_t addr,
			     void *dest, size_t size)
{
	acquire_device(dev);

	int ret = read_sfdp(dev, addr, dest, size);

	release_device(dev);

	return ret;
}

#endif /* CONFIG_FLASH_JESD216_API || CONFIG_MSPI_NOR_SFDP_RUNTIME */

static int mspi_nor_read_jedec_id(const struct device *dev,
				 uint8_t *id)
{
	struct cmd_info rcmd = {
		.dir = MSPI_RX,
		.cmd = SPI_NOR_CMD_RDID,
		.cmd_len = 1,
		.addr_len = 0,
		.dummy_cycles = 0,
		.min_data_len = 1,
	};

	if (id == NULL) {
		return -EINVAL;
	}

	acquire_device(dev);
	int ret = mspi_nor_access(dev, MSPI_PIO, &rcmd, 0, id, SPI_NOR_MAX_ID_LEN);
	release_device(dev);

	return ret;
}

#ifndef CONFIG_MSPI_NOR_SFDP_MINIMAL
/* Put the device into the appropriate address mode, if supported.
 *
 * On successful return mspi_nor_data::flag_access_32bit has been set
 * (cleared) if the device is configured for 4-byte (3-byte) addresses
 * for read, write, and erase commands.
 *
 * @param dev the device
 *
 * @param enter_4byte_addr the Enter 4-Byte Addressing bit set from
 * DW16 of SFDP BFP.  A value of all zeros or all ones is interpreted
 * as "not supported".
 *
 * @retval -ENOTSUP if 4-byte addressing is supported but not in a way
 * that the driver can handle.
 * @retval negative codes if the attempt was made and failed
 * @retval 0 if the device is successfully left in 24-bit mode or
 *         reconfigured to 32-bit mode.
 */
static int mspi_nor_set_address_mode(const struct device *dev,
				    uint8_t enter_4byte_addr)
{
	struct cmd_info wcmd = {
		.dir = MSPI_TX,
		.cmd = SPI_NOR_CMD_WREN,
		.cmd_len = 1,
		.addr_len = 0,
		.dummy_cycles = 0,
		.min_data_len = 0,
	};
	int ret = 0;

	LOG_DBG("Checking enter-4byte-addr %02x", enter_4byte_addr);

	/* Do nothing if not provided (either no bits or all bits
	 * set).
	 */
	if ((enter_4byte_addr == 0)
	    || (enter_4byte_addr == 0xff)) {
		return 0;
	}

	/* This currently only supports command 0xB7 (Enter 4-Byte
	 * Address Mode), with or without preceding WREN.
	 */
	if ((enter_4byte_addr & 0x03) == 0) {
		return -ENOTSUP;
	}

	acquire_device(dev);

	if ((enter_4byte_addr & 0x02) != 0) {
		/* Enter after WREN. */
		ret = mspi_nor_access(dev, MSPI_PIO, &wcmd, 0, NULL, 0);
	}

	if (ret == 0) {
		wcmd.cmd = SPI_NOR_CMD_4BA;
		ret = mspi_nor_access(dev, MSPI_PIO, &wcmd, 0, NULL, 0);
	}

	release_device(dev);

	return ret;
}

static int mspi_nor_process_bfp(const struct device *dev,
			       const struct jesd216_param_header *php,
			       const struct jesd216_bfp *bfp)
{
	const struct mspi_nor_config *cfg = dev->config;
	struct mspi_nor_data *data = dev->data;
	struct jesd216_erase_type *etp = data->erase_types;
	const size_t flash_size = jesd216_bfp_density(bfp) / 8U;

	LOG_INF("%s: %u MiBy flash", dev->name, (uint32_t)(flash_size >> 20));

	/* Copy over the erase types, preserving their order.  (The
	 * Sector Map Parameter table references them by index.)
	 */
	memset(data->erase_types, 0, sizeof(data->erase_types));
	for (uint8_t ti = 1; ti <= ARRAY_SIZE(data->erase_types); ++ti) {
		if (jesd216_bfp_erase(bfp, ti, etp) == 0) {
			LOG_DBG("Erase %u with %02x", (uint32_t)BIT(etp->exp), etp->cmd);
		}
		++etp;
	}

	data->layout.pages_size = jesd216_bfp_page_size(php, bfp);
	data->layout.pages_count = flash_size / data->layout.pages_size;
	data->flash_size = flash_size;

	LOG_DBG("Page size %u bytes", data->layout.pages_size);

	/* If 4-byte addressing is supported, switch to it. */
	if (jesd216_bfp_addrbytes(bfp) != JESD216_SFDP_BFP_DW1_ADDRBYTES_VAL_3B) {
		struct jesd216_bfp_dw16 dw16;
		int rc = 0;
		if (cfg->tar_dev_cfg.addr_length == 4)
		{
			if (jesd216_bfp_decode_dw16(php, bfp, &dw16) == 0) {
				rc = mspi_nor_set_address_mode(dev, dw16.enter_4ba);
			}
			if (rc != 0) {
				LOG_ERR("Unable to enter 4-byte mode: %d\n", rc);
				return rc;
			}
			data->dev_cfg.addr_length = 4;
		}
	} else {
		if (cfg->tar_dev_cfg.addr_length == 4) {
			LOG_ERR("Invalid device tree address length\n");
			return -EINVAL;
		}
	}

	/* add more stuff here
	fill out
	enum jesd216_wen_sel;

	enum jesd216_qer;

	enum jesd216_qen_seq;

	enum jesd216_qdis_seq;

	enum jesd216_entr_4B_seq;

	enum jesd216_exit_4B_seq;

	enum jesd216_sw_rst_seq;

	enum jesd216_oer;

	enum jesd216_oen_seq;

	enum jesd216_odis_seq;
	 */

	return 0;
}

static int mspi_nor_process_sfdp(const struct device *dev)
{
	int rc;

#if defined(CONFIG_MSPI_NOR_SFDP_RUNTIME)
	/* For runtime we need to read the SFDP table, identify the
	 * BFP block, and process it.
	 */
	const uint8_t decl_nph = 2;
	union {
		/* We only process BFP so use one parameter block */
		uint8_t raw[JESD216_SFDP_SIZE(decl_nph)];
		struct jesd216_sfdp_header sfdp;
	} u_header;
	const struct jesd216_sfdp_header *hp = &u_header.sfdp;

	rc = mspi_nor_sfdp_read(dev, 0, u_header.raw, sizeof(u_header.raw));
	if (rc != 0) {
		LOG_ERR("SFDP read failed: %d", rc);
		return rc;
	}

	uint32_t magic = jesd216_sfdp_magic(hp);

	if (magic != JESD216_SFDP_MAGIC) {
		LOG_ERR("SFDP magic %08x invalid", magic);
		return -EINVAL;
	}

	LOG_INF("%s: SFDP v %u.%u AP %x with %u PH", dev->name,
		hp->rev_major, hp->rev_minor, hp->access, 1 + hp->nph);

	const struct jesd216_param_header *php = hp->phdr;
	const struct jesd216_param_header *phpe = php + MIN(decl_nph, 1 + hp->nph);

	while (php != phpe) {
		uint16_t id = jesd216_param_id(php);

		LOG_INF("PH%u: %04x rev %u.%u: %u DW @ %x",
			(php - hp->phdr), id, php->rev_major, php->rev_minor,
			php->len_dw, jesd216_param_addr(php));

		if (id == JESD216_SFDP_PARAM_ID_BFP) {
			union {
				uint32_t dw[MIN(php->len_dw, 20)];
				struct jesd216_bfp bfp;
			} u_param;
			const struct jesd216_bfp *bfp = &u_param.bfp;

			rc = mspi_nor_sfdp_read(dev, jesd216_param_addr(php),
				u_param.dw, sizeof(u_param.dw));
			if (rc == 0) {
				rc = mspi_nor_process_bfp(dev, php, bfp);
			}

			if (rc != 0) {
				LOG_INF("SFDP BFP failed: %d", rc);
				break;
			}
		}
		++php;
	}
#elif defined(CONFIG_MSPI_NOR_SFDP_DEVICETREE)
	/* For devicetree we need to synthesize a parameter header and
	 * process the stored BFP data as if we had read it.
	 */
	const struct mspi_nor_config *cfg = dev->config;
	struct jesd216_param_header bfp_hdr = {
		.len_dw = cfg->bfp_len,
	};

	rc = mspi_nor_process_bfp(dev, &bfp_hdr, cfg->bfp);
#else
#error Unhandled SFDP choice
#endif

	return rc;
}

#endif /* !CONFIG_MSPI_NOR_SFDP_MINIMAL */

/**
 * @brief Configure the flash
 *
 * @param dev The flash device structure
 * @param info The flash info structure
 * @return 0 on success, negative errno code otherwise
 */
static int mspi_nor_configure(const struct device *dev)
{
	const struct mspi_nor_config *cfg = dev->config;
	uint8_t jedec_id[SPI_NOR_MAX_ID_LEN];
	int rc;

	/* Validate bus is ready */
	if (!device_is_ready(cfg->bus)) {
		return -ENODEV;
	}

#if ANY_INST_HAS_RESET_GPIOS

	if (cfg->reset_gpios_exist) {
		if (!gpio_is_ready_dt(&cfg->reset)) {
			LOG_ERR("Reset pin not ready");
			return -ENODEV;
		}
		if (gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_ACTIVE)) {
			LOG_ERR("Couldn't configure reset pin");
			return -ENODEV;
		}
		rc = gpio_pin_set_dt(&cfg->reset, 0);
		if (rc) {
			return rc;
		}
	}
#endif

	/* After a soft-reset the flash might be in DPD or busy writing/erasing.
	 * Exit DPD and wait until flash is ready.
	 */
	acquire_device(dev);

	rc = exit_dpd(dev);
	if (rc < 0) {
		LOG_ERR("Failed to exit DPD (%d)", rc);
		release_device(dev);
		return -ENODEV;
	}

	rc = mspi_nor_rdsr(dev);
	if (rc > 0 && (rc & SPI_NOR_WIP_BIT)) {
		LOG_WRN("Waiting until flash is ready");
		rc = mspi_nor_wait_until_ready(dev, WAIT_READY_REGISTER);
	} else if (rc < 0) {
		LOG_ERR("Failed to wait until flash is ready (%d)", rc);
		release_device(dev);
		return -ENODEV;
	}

	release_device(dev);

	/* now the mspi bus is configured, we can verify MSPI
	 * connectivity by reading the JEDEC ID.
	 */

	rc = mspi_nor_read_jedec_id(dev, jedec_id);
	if (rc != 0) {
		LOG_ERR("JEDEC ID read failed: %d", rc);
		return -ENODEV;
	}

#ifndef CONFIG_MSPI_NOR_SFDP_MINIMAL
	/* For minimal and devicetree we need to check the JEDEC ID
	 * against the one from devicetree, to ensure we didn't find a
	 * device that has different parameters.
	 */

	if (memcmp(jedec_id, cfg->jedec_id, sizeof(jedec_id)) != 0) {
		LOG_ERR("Device id %02x %02x %02x does not match config %02x %02x %02x",
			jedec_id[0], jedec_id[1], jedec_id[2],
			cfg->jedec_id[0], cfg->jedec_id[1], cfg->jedec_id[2]);
		return -EINVAL;
	}
#endif

	/* Check for block protect bits that need to be cleared.  This
	 * information cannot be determined from SFDP content, so the
	 * devicetree node property must be set correctly for any device
	 * that powers up with block protect enabled.
	 */
#if ANY_INST_HAS_BP
	acquire_device(dev);
	if (cfg->has_bp != 0) {

		rc = mspi_nor_rdsr(dev);

		/* Only clear if RDSR worked and something's set. */
		if (rc > 0) {
			rc = mspi_nor_wrsr(dev, rc & ~cfg->has_bp);
		}

		if (rc != 0) {
			LOG_ERR("BP clear failed: %d\n", rc);
			release_device(dev);
			return -ENODEV;
		}
	}
	release_device(dev);
#endif

#ifndef CONFIG_MSPI_NOR_SFDP_MINIMAL
	/* For devicetree and runtime we need to process BFP data and
	 * set up or validate page layout.
	 */
	rc = mspi_nor_process_sfdp(dev);
	if (rc != 0) {
		LOG_ERR("SFDP read failed: %d", rc);
		return -ENODEV;
	}
#endif /* !CONFIG_MSPI_NOR_SFDP_MINIMAL */

	return 0;
}

static int mspi_nor_pm_control(const struct device *dev, enum pm_device_action action)
{
	int rc = 0;

	switch (action) {
#ifdef CONFIG_MSPI_NOR_IDLE_IN_DPD
	case PM_DEVICE_ACTION_SUSPEND:
	case PM_DEVICE_ACTION_RESUME:
		break;
#else
	case PM_DEVICE_ACTION_SUSPEND:
		acquire_device(dev);
		rc = enter_dpd(dev);
		release_device(dev);
		break;
	case PM_DEVICE_ACTION_RESUME:
		acquire_device(dev);
		rc = exit_dpd(dev);
		release_device(dev);
		break;
#endif /* CONFIG_MSPI_NOR_IDLE_IN_DPD */
	case PM_DEVICE_ACTION_TURN_ON:
		/* Coming out of power off */
		rc = mspi_nor_configure(dev);
#ifndef CONFIG_MSPI_NOR_IDLE_IN_DPD
		if (rc == 0) {
			/* Move to DPD, the correct device state
			 * for PM_DEVICE_STATE_SUSPENDED
			 */
			acquire_device(dev);
			rc = enter_dpd(dev);
			release_device(dev);
		}
#endif /* CONFIG_MSPI_NOR_IDLE_IN_DPD */
		break;
	case PM_DEVICE_ACTION_TURN_OFF:
		break;
	default:
		rc = -ENOSYS;
	}

	return rc;
}

/**
 * @brief Initialize and configure the flash
 *
 * @param name The flash name
 * @return 0 on success, negative errno code otherwise
 */
static int mspi_nor_init(const struct device *dev)
{
	const struct mspi_nor_config *cfg = dev->config;
	struct mspi_nor_data *data = dev->data;

	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct mspi_nor_data *const driver_data = dev->data;

		k_sem_init(&driver_data->sem, 1, K_SEM_MAX_LIMIT);
	}

#if ANY_INST_HAS_WP_GPIOS
	if (DEV_CFG(dev)->wp_gpios_exist) {
		if (!device_is_ready(DEV_CFG(dev)->wp.port)) {
			LOG_ERR("Write-protect pin not ready");
			return -ENODEV;
		}
		if (gpio_pin_configure_dt(&(DEV_CFG(dev)->wp), GPIO_OUTPUT_ACTIVE)) {
			LOG_ERR("Write-protect pin failed to set active");
			return -ENODEV;
		}
	}
#endif /* ANY_INST_HAS_WP_GPIOS */
#if ANY_INST_HAS_HOLD_GPIOS
	if (DEV_CFG(dev)->hold_gpios_exist) {
		if (!device_is_ready(DEV_CFG(dev)->hold.port)) {
			LOG_ERR("Hold pin not ready");
			return -ENODEV;
		}
		if (gpio_pin_configure_dt(&(DEV_CFG(dev)->hold), GPIO_OUTPUT_INACTIVE)) {
			LOG_ERR("Hold pin failed to set inactive");
			return -ENODEV;
		}
	}
#endif /* ANY_INST_HAS_HOLD_GPIOS */

	if (mspi_dev_config(cfg->bus, &cfg->dev_id, MSPI_DEVICE_CONFIG_ALL, &cfg->serial_cfg)) {
		LOG_ERR("Failed to config mspi controller/%u", __LINE__);
		return -EIO;
	}
	data->dev_cfg = cfg->serial_cfg;

	if (pm_device_driver_init(dev, mspi_nor_pm_control)) {
		LOG_ERR("Failed to config device/%u", __LINE__);
		return -EIO;
	}

	if (mspi_dev_config(cfg->bus, &cfg->dev_id, MSPI_DEVICE_CONFIG_ALL, &cfg->tar_dev_cfg)) {
		LOG_ERR("Failed to config mspi controller/%u", __LINE__);
		return -EIO;
	}
	data->dev_cfg = cfg->tar_dev_cfg;

#if CONFIG_MSPI_TIMING
	if (mspi_timing_config(cfg->bus, &cfg->dev_id, cfg->timing_cfg_mask,
			       (void *)&cfg->tar_timing_cfg)) {
		LOG_ERR("Failed to config mspi timing/%u", __LINE__);
		return -EIO;
	}
	data->timing_cfg = cfg->tar_timing_cfg;
#endif

#if CONFIG_MSPI_XIP
	if (cfg->tar_xip_cfg.enable) {
		if (mspi_xip_config(cfg->bus, &cfg->dev_id, &cfg->tar_xip_cfg)) {
			LOG_ERR("Failed to enable XIP/%u", __LINE__);
			return -EIO;
		}
		data->xip_cfg = cfg->tar_xip_cfg;
	}
#endif

#if CONFIG_MSPI_SCRAMBLE
	if (cfg->tar_scramble_cfg.enable) {
		if (mspi_scramble_config(cfg->bus, &cfg->dev_id, &cfg->tar_scramble_cfg)) {
			LOG_ERR("Failed to enable scrambling/%u", __LINE__);
			return -EIO;
		}
		data->scramble_cfg = cfg->tar_scramble_cfg;
	}
#endif

	return 0;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)

static void mspi_nor_pages_layout(const struct device *dev,
				 const struct flash_pages_layout **layout,
				 size_t *layout_size)
{
	/* Data for runtime and devicetree, const for minimal. */
#ifndef CONFIG_MSPI_NOR_SFDP_MINIMAL
	const struct mspi_nor_data *data = dev->data;

	*layout = &data->layout;
#else
	const struct mspi_nor_config *cfg = dev->config;

	*layout = &cfg->layout;
#endif /* !CONFIG_MSPI_NOR_SFDP_MINIMAL */

	*layout_size = 1;
}

#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *
flash_nor_get_parameters(const struct device *dev)
{
	const struct mspi_nor_config *cfg = dev->config;

	return &cfg->flash_param;
}

static const struct flash_driver_api mspi_nor_api = {
	.read = mspi_nor_read,
	.write = mspi_nor_write,
	.erase = mspi_nor_erase,
	.get_parameters = flash_nor_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = mspi_nor_pages_layout,
#endif
#if defined(CONFIG_FLASH_JESD216_API)
	.sfdp_read = mspi_nor_sfdp_read,
	.read_jedec_id = mspi_nor_read_jedec_id,
#endif
#if defined(CONFIG_FLASH_EX_OP_ENABLED)
	.ex_op = flash_mspi_nor_ex_op,
#endif
};

#define PAGE_LAYOUT_GEN(idx)                                                                      \
	BUILD_ASSERT(DT_INST_NODE_HAS_PROP(idx, size),                                            \
		"jedec,mspi-nor size required for CONFIG_MSPI_NOR_SFDP_MINIMAL");                 \
	BUILD_ASSERT(DT_INST_NODE_HAS_PROP(idx, page_size),                                       \
		"jedec,mspi-nor page-size required for CONFIG_MSPI_NOR_SFDP_MINIMAL");            \
	BUILD_ASSERT(SPI_NOR_IS_ALIGNED(DT_INST_PROP(idx, page_size), 8),                         \
		"jedec,mspi-nor page-size must be multiple of 256");                              \
	BUILD_ASSERT(((DT_INST_PROP(idx, size) / 8) % DT_INST_PROP(idx, page_size)) == 0,         \
		"jedec,mspi-nor page-size incompatible with jedec,mspi-nor size");

#define SFDP_BFP_ATTR_GEN(idx)                                                                    \
	BUILD_ASSERT(DT_INST_NODE_HAS_PROP(idx, sfdp_bfp),                                        \
		     "jedec,mspi-nor sfdp-bfp required for devicetree SFDP");                     \
	static const __aligned(4) uint8_t bfp_##idx##_data[] = DT_INST_PROP(idx, sfdp_bfp);

#define INST_ATTR_GEN(idx)                                                                        \
	BUILD_ASSERT(DT_INST_NODE_HAS_PROP(idx, jedec_id),                                        \
		     "jedec,mspi-nor jedec-id required for non-runtime SFDP");                    \
	IF_ENABLED(CONFIG_MSPI_NOR_SFDP_DEVICETREE, (SFDP_BFP_ATTR_GEN(idx)))

#define ATTRIBUTES_DEFINE(idx) COND_CODE_1(CONFIG_MSPI_NOR_SFDP_MINIMAL, (PAGE_LAYOUT_GEN(idx)),  \
									 (INST_ATTR_GEN(idx)))

#define INIT_BP(idx)                                                                              \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, has_bp),                                           \
			(.has_bp = DT_INST_PROP(idx, has_bp)), (.has_bp = 0))                     \

#define INIT_T_ENTER_DPD(idx)                                                                     \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, t_enter_dpd),                                      \
		(.t_enter_dpd =	DIV_ROUND_UP(DT_INST_PROP(idx, t_enter_dpd), NSEC_PER_MSEC)),     \
		(.t_enter_dpd = 0))

#define INIT_T_EXIT_DPD(idx)                                                                      \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, t_exit_dpd),                                       \
		(.t_exit_dpd = DIV_ROUND_UP(DT_INST_PROP(idx, t_exit_dpd), NSEC_PER_MSEC)),       \
		(.t_exit_dpd = 0))

#define INIT_WAKEUP_SEQ_PARAMS(idx)                                                               \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, dpd_wakeup_sequence),                              \
		(.t_dpdd_ms = DIV_ROUND_UP(                                                       \
			DT_INST_PROP_BY_IDX(idx, dpd_wakeup_sequence, 0), NSEC_PER_MSEC),         \
		.t_crdp_ms = DIV_ROUND_UP(                                                        \
			DT_INST_PROP_BY_IDX(idx, dpd_wakeup_sequence, 1), NSEC_PER_MSEC),         \
		.t_rdp_ms = DIV_ROUND_UP(                                                         \
			DT_INST_PROP_BY_IDX(idx, dpd_wakeup_sequence, 2), NSEC_PER_MSEC)),        \
		(.t_dpdd_ms = 0, .t_crdp_ms = 0, .t_rdp_ms = 0))

#define INIT_WP_GPIOS(idx)                                                                        \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, wp_gpios),                                         \
		(.wp = GPIO_DT_SPEC_INST_GET(idx, wp_gpios)),                                     \
		(.wp = {0}))

#define INIT_HOLD_GPIOS(idx)                                                                      \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, hold_gpios),                                       \
		(.hold = GPIO_DT_SPEC_INST_GET(idx, hold_gpios)),                                 \
		(.hold = {0},))

#define INIT_RESET_GPIOS(idx)                                                                     \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, reset_gpios),                                      \
		(.reset = GPIO_DT_SPEC_INST_GET(idx, reset_gpios)),                               \
		(.reset = {0}))

#define MSPI_DEVICE_CONFIG_SERIAL(idx)                                                            \
	{                                                                                         \
		.ce_num             = DT_INST_PROP(idx, mspi_hardware_ce_num),                    \
		.freq               = 12000000,                                                   \
		.io_mode            = MSPI_IO_MODE_SINGLE,                                        \
		.data_rate          = MSPI_DATA_RATE_SINGLE,                                      \
		.cpp                = MSPI_CPP_MODE_0,                                            \
		.endian             = MSPI_XFER_LITTLE_ENDIAN,                                    \
		.ce_polarity        = MSPI_CE_ACTIVE_LOW,                                         \
		.dqs_enable         = false,                                                      \
		.rx_dummy           = 8,                                                          \
		.tx_dummy           = 0,                                                          \
		.read_cmd           = SPI_NOR_CMD_READ_FAST,                                      \
		.write_cmd          = SPI_NOR_CMD_PP,                                             \
		.cmd_length         = 1,                                                          \
		.addr_length        = 3,                                                          \
		.mem_boundary       = 0,                                                          \
		.time_to_break      = 0,                                                          \
	}

#if CONFIG_SOC_FAMILY_AMBIQ
#define MSPI_TIMING_CONFIG(idx)                                                                   \
	{                                                                                         \
		.ui8WriteLatency    = DT_INST_PROP_BY_IDX(idx, ambiq_timing_config, 0),           \
		.ui8TurnAround      = DT_INST_PROP_BY_IDX(idx, ambiq_timing_config, 1),           \
		.bTxNeg             = DT_INST_PROP_BY_IDX(idx, ambiq_timing_config, 2),           \
		.bRxNeg             = DT_INST_PROP_BY_IDX(idx, ambiq_timing_config, 3),           \
		.bRxCap             = DT_INST_PROP_BY_IDX(idx, ambiq_timing_config, 4),           \
		.ui32TxDQSDelay     = DT_INST_PROP_BY_IDX(idx, ambiq_timing_config, 5),           \
		.ui32RxDQSDelay     = DT_INST_PROP_BY_IDX(idx, ambiq_timing_config, 6),           \
		.ui32RXDQSDelayEXT  = DT_INST_PROP_BY_IDX(idx, ambiq_timing_config, 7),           \
	}
#define MSPI_TIMING_CONFIG_MASK(idx) DT_INST_PROP(idx, ambiq_timing_config_mask)
#else
#define MSPI_TIMING_CONFIG(idx)
#define MSPI_TIMING_CONFIG_MASK(idx)
#endif

#define GENERATE_CONFIG_STRUCT(idx)								  \
	static const struct mspi_nor_config mspi_nor_##idx##_config = {				  \
		.flash_param =                                                                    \
			{                                                                         \
				.write_block_size    = MSPI_NOR_WRITE_SIZE,                       \
				.erase_value         = MSPI_NOR_ERASE_VALUE,                      \
			},                                                                        \
		.bus                = DEVICE_DT_GET(DT_INST_BUS(idx)),                            \
		.dev_id             = MSPI_DEVICE_ID_DT_INST(idx),                                \
		.serial_cfg         = MSPI_DEVICE_CONFIG_SERIAL(idx),                             \
		.tar_dev_cfg        = MSPI_DEVICE_CONFIG_DT_INST(idx),                            \
		.tar_xip_cfg        = MSPI_XIP_CONFIG_DT_INST(idx),                               \
		.tar_scramble_cfg   = MSPI_SCRAMBLE_CONFIG_DT_INST(idx),                          \
		.tar_timing_cfg     = MSPI_TIMING_CONFIG(idx),                                    \
		.timing_cfg_mask    = MSPI_TIMING_CONFIG_MASK(idx),                               \
		.sw_multi_periph    = DT_PROP(DT_INST_BUS(idx), software_multiperipheral),        \
		IF_ENABLED(CONFIG_MSPI_NOR_SFDP_MINIMAL,                                          \
			(                                                                         \
				.flash_size = DT_INST_PROP(idx, size) / 8,                        \
				.layout = {                                                       \
						.pages_count = DT_INST_PROP(idx, size) / 8 /      \
							       DT_INST_PROP(idx, page_size),      \
						.pages_size  = DT_INST_PROP(idx, page_size),      \
				},                                                                \
			))                                                                        \
		IF_ENABLED(CONFIG_MSPI_NOR_SFDP_DEVICETREE,                                       \
			(                                                                         \
				.jedec_id = DT_INST_PROP(idx, jedec_id),                          \
				.bfp_len = sizeof(bfp_##idx##_data) / 4,                          \
		 		.bfp     = (const struct jesd216_bfp *)bfp_##idx##_data,          \
			))                                                                        \
		IF_ENABLED(ANY_INST_HAS_BP, (INIT_BP(idx),))                                      \
		.dpd_exist            = DT_INST_PROP(idx, has_dpd),                               \
		.dpd_wakeup_seq_exist = DT_INST_NODE_HAS_PROP(idx, dpd_wakeup_sequence),          \
		.reset_gpios_exist    = DT_INST_NODE_HAS_PROP(idx, reset_gpios),                  \
		.wp_gpios_exist       = DT_INST_NODE_HAS_PROP(idx, wp_gpios),                     \
		.hold_gpios_exist     = DT_INST_NODE_HAS_PROP(idx, hold_gpios),                   \
		IF_ENABLED(ANY_INST_HAS_DPD, (INIT_T_ENTER_DPD(idx),))                            \
		IF_ENABLED(ANY_INST_HAS_T_EXIT_DPD, (INIT_T_EXIT_DPD(idx),))                      \
		IF_ENABLED(ANY_INST_HAS_DPD_WAKEUP_SEQUENCE, (INIT_WAKEUP_SEQ_PARAMS(idx),))      \
		IF_ENABLED(ANY_INST_HAS_RESET_GPIOS, (INIT_RESET_GPIOS(idx),))                    \
		IF_ENABLED(ANY_INST_HAS_WP_GPIOS, (INIT_WP_GPIOS(idx),))                          \
		IF_ENABLED(ANY_INST_HAS_HOLD_GPIOS, (INIT_HOLD_GPIOS(idx),))                      \
	};

#define ASSIGN_PM(idx) PM_DEVICE_DT_INST_DEFINE(idx, mspi_nor_pm_control);

#define MSPI_NOR_INST(idx)                                                                        \
	ASSIGN_PM(idx)                                                                            \
	ATTRIBUTES_DEFINE(idx)                                                                    \
	GENERATE_CONFIG_STRUCT(idx)                                                               \
	static struct mspi_nor_data mspi_nor_##idx##_data;                                        \
	DEVICE_DT_INST_DEFINE(idx, &mspi_nor_init, PM_DEVICE_DT_INST_GET(idx),                    \
			&mspi_nor_##idx##_data, &mspi_nor_##idx##_config,                         \
			POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY, &mspi_nor_api);

DT_INST_FOREACH_STATUS_OKAY(MSPI_NOR_INST)
