/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Ambiq SPI based Bluetooth HCI driver.
 */

#define DT_DRV_COMPAT ambiq_bt_hci_spi

#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/bluetooth/hci_driver.h>
#include <zephyr/bluetooth/hci.h>

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_hci_driver);

#if defined(CONFIG_SOC_APOLLO4P_BLUE)
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_ambiq.h>
#include "am_devices_cooper.h"
#endif /* defined(CONFIG_SOC_APOLLO4P_BLUE) */

#define HCI_SPI_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(ambiq_bt_hci_spi)
#define SPI_DEV_NODE DT_BUS(HCI_SPI_NODE)
#if defined(CONFIG_SOC_APOLLO4P_BLUE)
#define CLK_32M_NODE DT_NODELABEL(xo32m)
#define CLK_32K_NODE DT_NODELABEL(xo32k)
#endif /* defined(CONFIG_SOC_APOLLO4P_BLUE) */

#define HCI_CMD 0x01
#define HCI_ACL 0x02
#define HCI_SCO 0x03
#define HCI_EVT 0x04

/* Offset of special item */
#define PACKET_TYPE         0
#define PACKET_TYPE_SIZE    1
#define EVT_HEADER_TYPE     0
#define EVT_HEADER_EVENT    1
#define EVT_HEADER_SIZE     2
#define EVT_VENDOR_CODE_LSB 3
#define EVT_VENDOR_CODE_MSB 4
#define CMD_OGF             1
#define CMD_OCF             2

#define EVT_OK      0
#define EVT_DISCARD 1
#define EVT_NOP     2

/* Max SPI buffer length for transceive operations.
 * The maximum TX packet number is 512 bytes data + 12 bytes header.
 * The maximum RX packet number is 255 bytes data + 3 header.
 */
#define SPI_MAX_TX_MSG_LEN 524
#define SPI_MAX_RX_MSG_LEN 258

#if defined(CONFIG_SOC_APOLLO4P_BLUE)
/* Command/response for SPI operation */
#define SPI_WRITE   0x80
#define SPI_READ    0x04
#define READY_BYTE0 0x68
#define READY_BYTE1 0xA8

/* Maximum attempts of SPI write */
#define SPI_WRITE_TIMEOUT 200
#endif /* defined(CONFIG_SOC_APOLLO4P_BLUE) */

static uint8_t g_hciRxMsg[SPI_MAX_RX_MSG_LEN];
const struct device *spi_dev = DEVICE_DT_GET(SPI_DEV_NODE);
static struct spi_config spi_cfg = {
	.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA |
		     SPI_WORD_SET(8),
};
static K_KERNEL_STACK_DEFINE(spi_rx_stack, CONFIG_BT_DRV_RX_STACK_SIZE);
static struct k_thread spi_rx_thread_data;

static struct spi_buf spi_tx_buf;
static struct spi_buf spi_rx_buf;
static const struct spi_buf_set spi_tx = {.buffers = &spi_tx_buf, .count = 1};
static const struct spi_buf_set spi_rx = {.buffers = &spi_rx_buf, .count = 1};

#if defined(CONFIG_SOC_APOLLO4P_BLUE)
static const struct gpio_dt_spec irq_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, irq_gpios);
static const struct gpio_dt_spec rst_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, reset_gpios);
static const struct gpio_dt_spec cs_gpio = GPIO_DT_SPEC_GET(SPI_DEV_NODE, cs_gpios);
static const struct gpio_dt_spec clkreq_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, clkreq_gpios);

static struct gpio_callback irq_gpio_cb;
static struct gpio_callback clkreq_gpio_cb;

const struct device *clk32m_dev = DEVICE_DT_GET(CLK_32M_NODE);
const struct device *clk32k_dev = DEVICE_DT_GET(CLK_32K_NODE);
#endif /* defined(CONFIG_SOC_APOLLO4P_BLUE) */

static K_SEM_DEFINE(sem_irq, 0, 1);
static K_SEM_DEFINE(sem_spi_available, 1, 1);

#if defined(CONFIG_SOC_APOLLO4P_BLUE)
static bool irq_pin_state(void)
{
	int pin_state;
	pin_state = gpio_pin_get_dt(&irq_gpio);
	LOG_DBG("IRQ Pin: %d", pin_state);
	return pin_state > 0;
}

static bool clkreq_pin_state(void)
{
	int pin_state;
	pin_state = gpio_pin_get_dt(&clkreq_gpio);
	LOG_DBG("CLKREQ Pin: %d", pin_state);
	return pin_state > 0;
}

static void bt_packet_irq_isr(const struct device *unused1, struct gpio_callback *unused2,
			      uint32_t unused3)
{
	k_sem_give(&sem_irq);
}

static void bt_clkreq_isr(const struct device *unused1, struct gpio_callback *unused2,
			  uint32_t unused3)
{
	if (clkreq_pin_state()) {
		/* Enable XO32MHz */
		clock_control_on(clk32m_dev,
				 (clock_control_subsys_t)CLOCK_CONTROL_AMBIQ_TYPE_HFXTAL_BLE);
		gpio_pin_interrupt_configure_dt(&clkreq_gpio, GPIO_INT_EDGE_FALLING);
	} else {
		/* Disable XO32MHz */
		clock_control_off(clk32m_dev,
				  (clock_control_subsys_t)CLOCK_CONTROL_AMBIQ_TYPE_HFXTAL_BLE);
		gpio_pin_interrupt_configure_dt(&clkreq_gpio, GPIO_INT_EDGE_RISING);
	}
}

static void bt_controller_ready_wait(void)
{
	/* The CS pin is used to wake up the controller as well. If the controller is not ready
	 * to receive the SPI packet, need to inactivate the CS at first and reconfigure the pin
	 * to CS function again before next sending attempt.
	 */
	gpio_pin_configure_dt(&cs_gpio, GPIO_OUTPUT_INACTIVE);
	k_busy_wait(200);
	PINCTRL_DT_DEFINE(SPI_DEV_NODE);
	pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(SPI_DEV_NODE), PINCTRL_STATE_DEFAULT);
	k_busy_wait(2000);
}

static void bt_controller_reset(void)
{
	/* Reset the controller*/
	gpio_pin_set_dt(&rst_gpio, 1);

	/* Take controller out of reset */
	k_sleep(K_MSEC(10));
	gpio_pin_set_dt(&rst_gpio, 0);

	/* Give the controller some time to boot */
	k_sleep(K_MSEC(500));
}
#endif /* defined(CONFIG_SOC_APOLLO4P_BLUE) */

static inline int bt_spi_transceive(void *tx, uint32_t tx_len, void *rx, uint32_t rx_len)
{
	spi_tx_buf.buf = tx;
	spi_tx_buf.len = (size_t)tx_len;
	spi_rx_buf.buf = rx;
	spi_rx_buf.len = (size_t)rx_len;
	return spi_transceive(spi_dev, &spi_cfg, &spi_tx, &spi_rx);
}

int spi_send_packet(uint8_t *data, uint32_t len)
{
	int ret = -ENOTSUP;

	/* Wait for SPI bus to be available */
	k_sem_take(&sem_spi_available, K_FOREVER);

#if defined(CONFIG_SOC_APOLLO4P_BLUE)
	uint8_t command[1] = {SPI_WRITE};
	uint8_t response[2] = {0, 0};
	uint16_t fail_count = 0;

	do {
		/* Check if the controller is ready to receive the HCI packets. */
		ret = bt_spi_transceive(command, 1, response, 2);
		if ((response[0] != READY_BYTE0) || (response[1] != READY_BYTE1) || ret) {
			bt_controller_ready_wait();
		} else {
			/* Transmit the message */
			ret = bt_spi_transceive(data, len, NULL, 0);
			if (ret) {
				LOG_ERR("SPI write error %d", ret);
			}
			break;
		}
	} while (fail_count++ < SPI_WRITE_TIMEOUT);
#endif /* defined(CONFIG_SOC_APOLLO4P_BLUE) */

	/* Free the SPI bus */
	k_sem_give(&sem_spi_available);

	return ret;
}

static int spi_receive_packet(uint8_t *data, uint16_t *len)
{
	int ret = -ENOTSUP;

	/* Wait for SPI bus to be available */
	k_sem_take(&sem_spi_available, K_FOREVER);

#if defined(CONFIG_SOC_APOLLO4P_BLUE)
	uint8_t command[1] = {SPI_READ};
	uint8_t response[2] = {0, 0};
	uint16_t read_size = 0;

	do {
		/* Skip if the IRQ pin is not in high state */
		if (!irq_pin_state()) {
			ret = -1;
			break;
		}

		/* Check the available packet bytes */
		ret = bt_spi_transceive(command, 1, response, 2);
		if (ret) {
			break;
		}

		/* Check if the read size is acceptable */
		read_size = (uint16_t)(response[0] | response[1] << 8);
		if ((read_size == 0) || (read_size > SPI_MAX_RX_MSG_LEN)) {
			ret = -1;
			break;
		}

		*len = read_size;

		/* Read the HCI data from controller */
		ret = bt_spi_transceive(NULL, 0, data, read_size);

		if (ret) {
			LOG_ERR("SPI read error %d", ret);
			break;
		}
	} while (0);
#else
	*len = 0;
#endif /* defined(CONFIG_SOC_APOLLO4P_BLUE) */

	/* Free the SPI bus */
	k_sem_give(&sem_spi_available);

	return ret;
}

static int hci_event_filter(const uint8_t *evt_data)
{
	uint8_t evt_type = evt_data[0];

	switch (evt_type) {
	case BT_HCI_EVT_LE_META_EVENT: {
		uint8_t subevt_type = evt_data[sizeof(struct bt_hci_evt_hdr)];
		switch (subevt_type) {
		case BT_HCI_EVT_LE_ADVERTISING_REPORT:
			return EVT_DISCARD;
		default:
			return EVT_OK;
		}
	}
	case BT_HCI_EVT_CMD_COMPLETE: {
		uint16_t opcode = (uint16_t)(evt_data[3] + (evt_data[4] << 8));
		switch (opcode) {
		case BT_OP_NOP:
			return EVT_NOP;
		default:
			return EVT_OK;
		}
	}
	default:
		return EVT_OK;
	}
}

static struct net_buf *bt_hci_evt_recv(uint8_t *data, size_t len)
{
	int evt_filter;
	bool discardable = false;
	struct bt_hci_evt_hdr hdr = {0};
	struct net_buf *buf;
	size_t buf_tailroom;

	if (len < sizeof(hdr)) {
		LOG_ERR("Not enough data for event header");
		return NULL;
	}

	evt_filter = hci_event_filter(data);
	if (evt_filter == EVT_NOP) {
		/* The controller sends NOP event when wakes up based on
		 * hardware specific requirement, do not post this event to
		 * host stack.
		 */
		return NULL;
	} else if (evt_filter == EVT_DISCARD) {
		discardable = true;
	}

	memcpy((void *)&hdr, data, sizeof(hdr));
	data += sizeof(hdr);
	len -= sizeof(hdr);

	if (len != hdr.len) {
		LOG_ERR("Event payload length is not correct");
		return NULL;
	}

	buf = bt_buf_get_evt(hdr.evt, discardable, K_NO_WAIT);
	if (!buf) {
		if (discardable) {
			LOG_DBG("Discardable buffer pool full, ignoring event");
		} else {
			LOG_ERR("No available event buffers!");
		}
		return buf;
	}

	net_buf_add_mem(buf, &hdr, sizeof(hdr));

	buf_tailroom = net_buf_tailroom(buf);
	if (buf_tailroom < len) {
		LOG_ERR("Not enough space in buffer %zu/%zu", len, buf_tailroom);
		net_buf_unref(buf);
		return NULL;
	}

	net_buf_add_mem(buf, data, len);

	return buf;
}

static struct net_buf *bt_hci_acl_recv(uint8_t *data, size_t len)
{
	struct bt_hci_acl_hdr hdr = {0};
	struct net_buf *buf;
	size_t buf_tailroom;

	if (len < sizeof(hdr)) {
		LOG_ERR("Not enough data for ACL header");
		return NULL;
	}

	buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_NO_WAIT);
	if (buf) {
		memcpy((void *)&hdr, data, sizeof(hdr));
		data += sizeof(hdr);
		len -= sizeof(hdr);
	} else {
		LOG_ERR("No available ACL buffers!");
		return NULL;
	}

	if (len != sys_le16_to_cpu(hdr.len)) {
		LOG_ERR("ACL payload length is not correct");
		net_buf_unref(buf);
		return NULL;
	}

	net_buf_add_mem(buf, &hdr, sizeof(hdr));
	buf_tailroom = net_buf_tailroom(buf);
	if (buf_tailroom < len) {
		LOG_ERR("Not enough space in buffer %zu/%zu", len, buf_tailroom);
		net_buf_unref(buf);
		return NULL;
	}

	net_buf_add_mem(buf, data, len);

	return buf;
}

static void bt_spi_rx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct net_buf *buf;
	int ret;
	uint16_t len = 0;

	while (true) {
		/* Wait for controller interrupt */
		k_sem_take(&sem_irq, K_FOREVER);

		do {
			/* Recevive the HCI packet via SPI */
			ret = spi_receive_packet(&g_hciRxMsg[0], &len);
			if (ret) {
				break;
			}

#if defined(CONFIG_BT_HCI_SETUP)
#if defined(CONFIG_SOC_APOLLO4P_BLUE)
			/* The vendor specfic handshake command/response is incompatible with
			 * standard Bluetooth HCI format, need to handle the received packets
			 * specifically.
			 */
			if (am_devices_cooper_get_initialize_state() !=
			    AM_DEVICES_COOPER_STATE_INITALIZED) {
				am_devices_cooper_handshake_recv(&g_hciRxMsg[0], len);
				break;
			}
#endif /* defined(CONFIG_SOC_APOLLO4P_BLUE) */
#endif /* defined(CONFIG_BT_HCI_SETUP) */

			switch (g_hciRxMsg[PACKET_TYPE]) {
			case HCI_EVT:
				buf = bt_hci_evt_recv(&g_hciRxMsg[PACKET_TYPE + PACKET_TYPE_SIZE],
						      (len - PACKET_TYPE_SIZE));
				break;
			case HCI_ACL:
				buf = bt_hci_acl_recv(&g_hciRxMsg[PACKET_TYPE + PACKET_TYPE_SIZE],
						      (len - PACKET_TYPE_SIZE));
				break;
			default:
				buf = NULL;
				LOG_WRN("Unknown BT buf type %d", g_hciRxMsg[PACKET_TYPE]);
				break;
			}

			/* Post the RX message to host stack to process */
			if (buf) {
				bt_recv(buf);
			}
		} while (0);
	}
}

static int bt_hci_send(struct net_buf *buf)
{
	int ret = 0;

	/* Buffer needs an additional byte for type */
	if (buf->len >= SPI_MAX_TX_MSG_LEN) {
		LOG_ERR("Message too long");
		return -EINVAL;
	}

	switch (bt_buf_get_type(buf)) {
	case BT_BUF_ACL_OUT:
		net_buf_push_u8(buf, HCI_ACL);
		break;
	case BT_BUF_CMD:
		net_buf_push_u8(buf, HCI_CMD);
		break;
	default:
		LOG_ERR("Unsupported type");
		return -EINVAL;
	}

	/* Send the SPI packet */
	ret = spi_send_packet(buf->data, buf->len);

	net_buf_unref(buf);

	return ret;
}

static int bt_hci_open(void)
{
	int ret = 0;

#if defined(CONFIG_SOC_APOLLO4P_BLUE)
	/* Configure the XO32MHz and XO32kHz clocks.*/
	clock_control_configure(clk32k_dev, NULL, NULL);
	clock_control_configure(clk32m_dev, NULL, NULL);

	/* Enable XO32kHz for Controller */
	clock_control_on(clk32k_dev, (clock_control_subsys_t)CLOCK_CONTROL_AMBIQ_TYPE_LFXTAL);

	/* Enable XO32MHz for Controller */
	clock_control_on(clk32m_dev, (clock_control_subsys_t)CLOCK_CONTROL_AMBIQ_TYPE_HFXTAL_BLE);

	/* Configure RST pin and hold BLE in Reset */
	ret = gpio_pin_configure_dt(&rst_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		return ret;
	}

	/* Configure IRQ pin and register the callback */
	ret = gpio_pin_configure_dt(&irq_gpio, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	gpio_init_callback(&irq_gpio_cb, bt_packet_irq_isr, BIT(irq_gpio.pin));
	ret = gpio_add_callback(irq_gpio.port, &irq_gpio_cb);
	if (ret) {
		return ret;
	}

	/* Configure CLKREQ pin and register the callback */
	ret = gpio_pin_configure_dt(&clkreq_gpio, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	gpio_init_callback(&clkreq_gpio_cb, bt_clkreq_isr, BIT(clkreq_gpio.pin));
	ret = gpio_add_callback(clkreq_gpio.port, &clkreq_gpio_cb);
	if (ret) {
		return ret;
	}

	/* Configure the interrupt edge for CLKREQ pin */
	gpio_pin_interrupt_configure_dt(&clkreq_gpio, GPIO_INT_EDGE_RISING);

	/* Take controller out of reset */
	k_sleep(K_MSEC(10));
	gpio_pin_set_dt(&rst_gpio, 0);

	/* Give the controller some time to boot */
	k_sleep(K_MSEC(500));

	/* Configure the interrupt edge for IRQ pin */
	gpio_pin_interrupt_configure_dt(&irq_gpio, GPIO_INT_EDGE_RISING);
#endif /* defined(CONFIG_SOC_APOLLO4P_BLUE) */

	/* Start RX thread */
	k_thread_create(&spi_rx_thread_data, spi_rx_stack, K_KERNEL_STACK_SIZEOF(spi_rx_stack),
			(k_thread_entry_t)bt_spi_rx_thread, NULL, NULL, NULL,
			K_PRIO_COOP(CONFIG_BT_DRIVER_RX_HIGH_PRIO), 0, K_NO_WAIT);

	return ret;
}

#if defined(CONFIG_BT_HCI_SETUP)
static int bt_spi_setup(const struct bt_hci_setup_params *params)
{
	ARG_UNUSED(params);

	int ret = -ENOTSUP;

#if defined(CONFIG_SOC_APOLLO4P_BLUE)
	struct net_buf *buf;
	uint8_t *p;
	am_devices_cooper_callback_t cb = {
		.write = spi_send_packet,
		.reset = bt_controller_reset,
	};

	/* Initialize the BLE controller */
	ret = am_devices_cooper_init(&cb);
	if (ret == AM_DEVICES_COOPER_STATUS_SUCCESS) {
		am_devices_cooper_set_initialize_state(AM_DEVICES_COOPER_STATE_INITALIZED);
		LOG_INF("BT controller initialized");
	} else {
		am_devices_cooper_set_initialize_state(AM_DEVICES_COOPER_STATE_INITALIZE_FAIL);
		LOG_ERR("BT controller initialization fail");
		return ret;
	}

	/* Set the NVDS parameters to BLE Controller */
	buf = bt_hci_cmd_create(HCI_VSC_UPDATE_NVDS_CFG_CMD_OPCODE,
				HCI_VSC_UPDATE_NVDS_CFG_CMD_LENGTH);
	if (!buf) {
		return -ENOBUFS;
	}

	p = net_buf_add(buf, HCI_VSC_UPDATE_NVDS_CFG_CMD_LENGTH);
	memcpy(p, &am_devices_cooper_nvds[0], HCI_VSC_UPDATE_NVDS_CFG_CMD_LENGTH);
	ret = bt_hci_cmd_send_sync(HCI_VSC_UPDATE_NVDS_CFG_CMD_OPCODE, buf, NULL);

	if (!ret) {
		/* Give some time to make NVDS take effect in BLE Controller */
		k_sleep(K_MSEC(5));
	}
#endif /* defined(CONFIG_SOC_APOLLO4P_BLUE) */

	return ret;
}
#endif /* defined(CONFIG_BT_HCI_SETUP) */

static const struct bt_hci_driver drv = {
	.name = "ambiq hci",
	.bus = BT_HCI_DRIVER_BUS_SPI,
	.open = bt_hci_open,
	.send = bt_hci_send,
#if defined(CONFIG_BT_HCI_SETUP)
	.setup = bt_spi_setup,
#endif /* defined(CONFIG_BT_HCI_SETUP) */
};

static int bt_hci_init(void)
{
	if (!device_is_ready(spi_dev)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

#if defined(CONFIG_SOC_APOLLO4P_BLUE)
	if (!device_is_ready(irq_gpio.port)) {
		LOG_ERR("IRQ GPIO device not ready");
		return -ENODEV;
	}

	if (!device_is_ready(rst_gpio.port)) {
		LOG_ERR("Reset GPIO device not ready");
		return -ENODEV;
	}

	if (!device_is_ready(clkreq_gpio.port)) {
		LOG_ERR("CLKREQ GPIO device not ready");
		return -ENODEV;
	}
#endif /* defined(CONFIG_SOC_APOLLO4P_BLUE) */

	bt_hci_driver_register(&drv);

	LOG_DBG("BT HCI initialized");

	return 0;
}

SYS_INIT(bt_hci_init, POST_KERNEL, CONFIG_BT_HCI_INIT_PRIORITY);
