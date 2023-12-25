/*
 * Copyright (c) 2016 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 * Copyright (c) 2023 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/usb/usb_device.h>

#include <zephyr/net/buf.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/l2cap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/bluetooth/hci_raw.h>

#define LOG_MODULE_NAME uart_bridge
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static K_THREAD_STACK_DEFINE(tx_thread_stack, CONFIG_BT_HCI_TX_STACK_SIZE);
static struct k_thread tx_thread_data;
static K_FIFO_DEFINE(tx_queue);

/* RX in terms of bluetooth communication */
static K_FIFO_DEFINE(uart_tx_queue);

#define HCI_CMD 0x01
#define HCI_ACL 0x02
#define HCI_EVT 0x04

/* Receiver states. */
#define STATE_IDLE 0	/* Waiting for packet type. */
#define STATE_HDR 1	/* Receiving packet header. */
#define STATE_PAYLOAD 2	/* Receiving packet payload. */
#define STATE_DISCARD 3	/* Dropping packet. */

/* Length of a discard/flush buffer.
 * This is sized to align with a BLE HCI packet:
 * 1 byte H:4 header + 32 bytes ACL/event data
 * Bigger values might overflow the stack since this is declared as a local
 * variable, smaller ones will force the caller to call into discard more
 * often.
 */
#define HCI_DISCARD_LEN 33

static int uart_read(const struct device *uart, uint8_t *buf, size_t len)
{
	int rx = uart_fifo_read(uart, buf, len);

	LOG_DBG("read %d req %d", rx, len);

	return rx;
}

static bool valid_type(uint8_t type)
{
	return (type == HCI_CMD) | (type == HCI_ACL);
}

/* Function expects that type is validated and only CMD or ACL will be used. */
static uint32_t get_len(const uint8_t *hdr_buf, uint8_t type)
{
	switch (type) {
	case HCI_CMD:
		return ((const struct bt_hci_cmd_hdr *)hdr_buf)->param_len;
	case HCI_ACL:
		return sys_le16_to_cpu(((const struct bt_hci_acl_hdr *)hdr_buf)->len);
	default:
		LOG_ERR("Invalid type: %u", type);
		return 0;
	}
}

/* Function expects that type is validated and only CMD or ACL will be used. */
static int hdr_len(uint8_t type)
{
	switch (type) {
	case HCI_CMD:
		return sizeof(struct bt_hci_cmd_hdr);
	case HCI_ACL:
		return sizeof(struct bt_hci_acl_hdr);
	default:
		LOG_ERR("Invalid type: %u", type);
		return 0;
	}
}

static void rx_isr(void)
{
	static struct net_buf *buf;
	static int remaining;
	static uint8_t state;
	static uint8_t type;
	static uint8_t hdr_buf[MAX(sizeof(struct bt_hci_cmd_hdr),
			sizeof(struct bt_hci_acl_hdr))];
	int read;

	do {
		switch (state) {
		case STATE_IDLE:
			/* Get packet type */
			read = uart_read(uart_dev, &type, sizeof(type));
			/* since we read in loop until no data is in the fifo,
			 * it is possible that read = 0.
			 */
			if (read) {
				if (valid_type(type)) {
					/* Get expected header size and switch
					 * to receiving header.
					 */
					remaining = hdr_len(type);
					state = STATE_HDR;
				} else {
					LOG_WRN("Unknown header %d", type);
				}
			}
			break;
		case STATE_HDR:
			read = uart_read(uart_dev,
				       &hdr_buf[hdr_len(type) - remaining],
				       remaining);
			remaining -= read;
			if (remaining == 0) {
				/* Header received. Allocate buffer and get
				 * payload length. If allocation fails leave
				 * interrupt. On failed allocation state machine
				 * is reset.
				 */
				switch (type) {
				case HCI_CMD:
					buf = bt_buf_get_tx(BT_BUF_CMD, K_NO_WAIT,
								hdr_buf, hdr_len(type));
				break;
				case HCI_ACL:
					buf = bt_buf_get_tx(BT_BUF_ACL_OUT, K_NO_WAIT,
								hdr_buf, hdr_len(type));
				break;
				default:
				break;
				}

				if (!buf) {
					LOG_ERR("No available command buffers!");
					state = STATE_IDLE;
					return;
				}

				remaining = get_len(hdr_buf, type);
				if (remaining > net_buf_tailroom(buf)) {
					LOG_ERR("Not enough space in buffer");
					net_buf_unref(buf);
					state = STATE_DISCARD;
				} else {
					state = STATE_PAYLOAD;
				}

			}
			break;
		case STATE_PAYLOAD:
			read = uart_read(uart_dev, net_buf_tail(buf),
					remaining);
			buf->len += read;
			remaining -= read;
			if (remaining == 0) {
				/* Packet received */
				LOG_DBG("putting RX packet in queue.");
				net_buf_put(&tx_queue, buf);
				state = STATE_IDLE;
			}
			break;
		case STATE_DISCARD:
		{
			uint8_t discard[HCI_DISCARD_LEN];
			size_t to_read = MIN(remaining, sizeof(discard));

			read = uart_read(uart_dev, discard, to_read);
			remaining -= read;
			if (remaining == 0) {
				state = STATE_IDLE;
			}

			break;

		}
		default:
			read = 0;
			__ASSERT_NO_MSG(0);
			break;

		}
	} while (read);
}

static void tx_isr(void)
{
	static struct net_buf *buf;
	int len;

	if (!buf) {
		buf = net_buf_get(&uart_tx_queue, K_NO_WAIT);
		if (!buf) {
			uart_irq_tx_disable(uart_dev);
			return;
		}
	}

	len = uart_fifo_fill(uart_dev, buf->data, buf->len);
	net_buf_pull(buf, len);
	if (!buf->len) {
		net_buf_unref(buf);
		buf = NULL;
	}
}

static void bt_uart_isr(const struct device *unused, void *user_data)
{
	ARG_UNUSED(unused);
	ARG_UNUSED(user_data);

	if (!(uart_irq_rx_ready(uart_dev) ||
	      uart_irq_tx_ready(uart_dev))) {
		LOG_DBG("spurious interrupt");
	}

	if (uart_irq_tx_ready(uart_dev)) {
		tx_isr();
	}

	if (uart_irq_rx_ready(uart_dev)) {
		rx_isr();
	}
}

static void tx_thread(void *p1, void *p2, void *p3)
{
	while (1) {
		struct net_buf *buf;
		int err;

		/* Wait until a buffer is available */
		buf = net_buf_get(&tx_queue, K_FOREVER);
		/* Pass buffer to the stack */
		err = bt_send(buf);
		if (err) {
			LOG_ERR("Unable to send (err %d)", err);
			net_buf_unref(buf);
		}

		/* Give other threads a chance to run if tx_queue keeps getting
		 * new data all the time.
		 */
		k_yield();
	}
}

static int uart_send(struct net_buf *buf)
{
	LOG_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf),
		    buf->len);

	switch (bt_buf_get_type(buf)) {
	case BT_BUF_ACL_IN:
		net_buf_push_u8(buf, HCI_ACL);
		break;
	case BT_BUF_EVT:
		net_buf_push_u8(buf, HCI_EVT);
		break;
	default:
		LOG_ERR("Unknown type %u", bt_buf_get_type(buf));
		net_buf_unref(buf);
		return -EINVAL;
	}

	net_buf_put(&uart_tx_queue, buf);
	uart_irq_tx_enable(uart_dev);

	return 0;
}

static int hci_uart_init(void)
{
	LOG_DBG("");

	if (IS_ENABLED(CONFIG_USB_CDC_ACM)) {
		if (usb_enable(NULL)) {
			LOG_ERR("Failed to enable USB");
			return -EINVAL;
		}
	}

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("HCI UART %s is not ready", uart_dev->name);
		return -EINVAL;
	}

	uart_irq_rx_disable(uart_dev);
	uart_irq_tx_disable(uart_dev);

	uart_irq_callback_set(uart_dev, bt_uart_isr);

	uart_irq_rx_enable(uart_dev);

	return 0;
}

SYS_INIT(hci_uart_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

int main(void)
{
	/* incoming events and data from the controller */
	static K_FIFO_DEFINE(rx_queue);
	int err;

	__ASSERT(uart_dev, "UART device is NULL");

	/* Enable the raw interface, this will in turn open the HCI driver */
	bt_enable_raw(&rx_queue);

	extern int bt_apollo_vnd_setup(void);
	bt_apollo_vnd_setup();
	/* Spawn the TX thread and start feeding commands and data to the
	 * controller
	 */
	k_thread_create(&tx_thread_data, tx_thread_stack,
			K_THREAD_STACK_SIZEOF(tx_thread_stack), tx_thread,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	k_thread_name_set(&tx_thread_data, "HCI uart TX");

	while (1) {
		struct net_buf *buf;

		buf = net_buf_get(&rx_queue, K_FOREVER);
		err = uart_send(buf);
		if (err) {
			LOG_ERR("Failed to send");
		}
	}
	return 0;
}
