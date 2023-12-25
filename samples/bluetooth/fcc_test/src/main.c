/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/gpio.h>

#define SW0_NODE DT_ALIAS(sw0)
#define SW1_NODE DT_ALIAS(sw1)

#define LE_1M_PHY                 0x01
#define LE_2M_PHY                 0x02
#define MIN_SWITCHING_PATTERN_LEN 0x02
#define TEST_LEN_DEFAULT          0x25
#define PAYL_CONTINUOUS_WAVE      0x10
#define PAYL_CONTINUOUS_MODULATE  0x11
#define CHL_2402                  0
#define CHL_2440                  19
#define CHL_2480                  39

enum {
	TX_2402_CONTINUOUS,
	TX_2440_CONTINUOUS,
	TX_2480_CONTINUOUS,
	TX_2402_CONTINUOUS_MODULATE,
	TX_2440_CONTINUOUS_MODULATE,
	TX_2480_CONTINUOUS_MODULATE,
	RX_2402,
	RX_2440,
	RX_2480,
	TEST_CMD_MAX
};

static uint8_t current_test_case = TX_2402_CONTINUOUS_MODULATE;
static uint8_t current_phy = LE_1M_PHY;
static bool test_started = false;
static char *phy_str[] = {
	NULL,
	"LE 1M PHY",
	"LE 2M PHY",
};

#if DT_NODE_HAS_STATUS(SW0_NODE, okay) && DT_NODE_HAS_STATUS(SW1_NODE, okay)
K_SEM_DEFINE(sem_button1_pressed, 0, 1);
static const struct gpio_dt_spec button[2] = {GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0}),
					      GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios, {0})};
static struct gpio_callback button_cb_data[2];

static void button0_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (current_phy == LE_2M_PHY) {
		current_phy = LE_1M_PHY;
	} else {
		current_phy = LE_2M_PHY;
	}

	printk("Set phy = %s\n", phy_str[current_phy]);
}

static void button1_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_sem_give(&sem_button1_pressed);
}

static void button_init(void)
{
	int ret;

	for (uint8_t i = 0; i < 2; i++) {
		if (!gpio_is_ready_dt(&button[i])) {
			printk("Error: button1 device %s is not ready\n", button[i].port->name);
			return;
		}
		ret = gpio_pin_configure_dt(&button[i], GPIO_INPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure %s pin %d\n", ret,
			       button[i].port->name, button[i].pin);
			return;
		}
		ret = gpio_pin_interrupt_configure_dt(&button[i], GPIO_INT_EDGE_TO_ACTIVE);
		if (ret != 0) {
			printk("Error %d: failed to configure interrupt on %s pin %d\n", ret,
			       button[i].port->name, button[i].pin);
			return;
		}
	}

	gpio_init_callback(&button_cb_data[0], button0_pressed, BIT(button[0].pin));
	gpio_add_callback(button[0].port, &button_cb_data[0]);
	gpio_init_callback(&button_cb_data[1], button1_pressed, BIT(button[1].pin));
	gpio_add_callback(button[1].port, &button_cb_data[1]);
}
#endif

static int send_tx_test_cmd(uint8_t tx_ch, uint8_t payload)
{
	int ret;
	struct bt_hci_cp_le_tx_test_v3 *tx_cmd;
	struct net_buf *buf;

	if (test_started) {
		/* End the last testing */
		ret = bt_hci_cmd_send_sync(BT_HCI_OP_LE_TEST_END, NULL, NULL);
		if (ret) {
			return ret;
		}
		test_started = false;
	}

	buf = bt_hci_cmd_create(BT_HCI_OP_LE_TX_TEST_V3,
				(sizeof(*tx_cmd) + sizeof(uint8_t) * MIN_SWITCHING_PATTERN_LEN));
	if (!buf) {
		return -ENOBUFS;
	}

	tx_cmd = net_buf_add(buf, (sizeof(*tx_cmd) + sizeof(uint8_t) * MIN_SWITCHING_PATTERN_LEN));
	tx_cmd->tx_ch = tx_ch;
	tx_cmd->test_data_len = TEST_LEN_DEFAULT;
	tx_cmd->pkt_payload = payload;
	tx_cmd->phy = current_phy;
	tx_cmd->cte_len = 0;
	tx_cmd->cte_type = 0;
	tx_cmd->switch_pattern_len = MIN_SWITCHING_PATTERN_LEN;
	memset(&tx_cmd->ant_ids[0], 0x00, MIN_SWITCHING_PATTERN_LEN);
	ret = bt_hci_cmd_send_sync(BT_HCI_OP_LE_TX_TEST_V3, buf, NULL);

	return ret;
}

static int send_rx_test_cmd(uint8_t rx_ch)
{
	int ret;
	struct bt_hci_cp_le_rx_test_v3 *rx_cmd;
	struct net_buf *buf;

	if (test_started) {
		/* End the last testing */
		ret = bt_hci_cmd_send_sync(BT_HCI_OP_LE_TEST_END, NULL, NULL);
		if (ret) {
			return ret;
		}
		test_started = false;
	}

	buf = bt_hci_cmd_create(BT_HCI_OP_LE_RX_TEST_V3,
				(sizeof(*rx_cmd) + sizeof(uint8_t) * MIN_SWITCHING_PATTERN_LEN));
	if (!buf) {
		return -ENOBUFS;
	}

	rx_cmd = net_buf_add(buf, (sizeof(*rx_cmd) + sizeof(uint8_t) * MIN_SWITCHING_PATTERN_LEN));
	rx_cmd->rx_ch = rx_ch;
	rx_cmd->phy = current_phy;
	rx_cmd->expected_cte_len = 0;
	rx_cmd->expected_cte_type = 0;
	rx_cmd->slot_durations = 1; /* 1us */
	rx_cmd->switch_pattern_len = MIN_SWITCHING_PATTERN_LEN;
	memset(&rx_cmd->ant_ids[0], 0x00, MIN_SWITCHING_PATTERN_LEN);
	ret = bt_hci_cmd_send_sync(BT_HCI_OP_LE_RX_TEST_V3, buf, NULL);

	return ret;
}

static int test_handler(void)
{
	int ret = -ENOTSUP;

	switch (current_test_case) {
	case TX_2402_CONTINUOUS:
		ret = send_tx_test_cmd(CHL_2402, PAYL_CONTINUOUS_WAVE);
		printk("Continuous Wave on Channel 2402MHz, %s\n", phy_str[current_phy]);
		break;
	case TX_2440_CONTINUOUS:
		ret = send_tx_test_cmd(CHL_2440, PAYL_CONTINUOUS_WAVE);
		printk("Continuous Wave on Channel 2440MHz, %s\n", phy_str[current_phy]);
		break;
	case TX_2480_CONTINUOUS:
		ret = send_tx_test_cmd(CHL_2480, PAYL_CONTINUOUS_WAVE);
		printk("Continuous Wave on Channel 2480MHz, %s\n", phy_str[current_phy]);
		break;
	case TX_2402_CONTINUOUS_MODULATE:
		ret = send_tx_test_cmd(CHL_2402, PAYL_CONTINUOUS_MODULATE);
		printk("Continuous Modulation on Channel 2402MHz, %s\n", phy_str[current_phy]);
		break;
	case TX_2440_CONTINUOUS_MODULATE:
		ret = send_tx_test_cmd(CHL_2440, PAYL_CONTINUOUS_MODULATE);
		printk("Continuous Modulation on Channel 2440MHz, %s\n", phy_str[current_phy]);
		break;
	case TX_2480_CONTINUOUS_MODULATE:
		ret = send_tx_test_cmd(CHL_2480, PAYL_CONTINUOUS_MODULATE);
		printk("Continuous Modulation on Channel 2480MHz, %s\n", phy_str[current_phy]);
		break;
	case RX_2402:
		ret = send_rx_test_cmd(CHL_2402);
		printk("Receiver Test on Channel 2402MHz, %s\n", phy_str[current_phy]);
		break;
	case RX_2440:
		ret = send_rx_test_cmd(CHL_2440);
		printk("Receiver Test on Channel 2440MHz, %s\n", phy_str[current_phy]);
		break;
	case RX_2480:
		ret = send_rx_test_cmd(CHL_2480);
		printk("Receiver Test on Channel 2480MHz, %s\n", phy_str[current_phy]);
		break;
	default:
		printk("Not supported test item\n");
		break;
	}

	current_test_case++;
	if (current_test_case >= TEST_CMD_MAX) {
		current_test_case = TX_2402_CONTINUOUS;
	}

	return ret;
}

int main(void)
{
	int err;

	printk("BLE FCC Testing Application\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

#if DT_NODE_HAS_STATUS(SW0_NODE, okay) && DT_NODE_HAS_STATUS(SW1_NODE, okay)
	button_init();
	printk("Press button0 to switch Phy\n");
	printk("Press button1 to switch test case\n");
#else
	printk("No button detected, will switch test case every 10s.\n");
#endif

	while (1) {
#if DT_NODE_HAS_STATUS(SW0_NODE, okay) && DT_NODE_HAS_STATUS(SW1_NODE, okay)
		k_sem_take(&sem_button1_pressed, K_FOREVER);
#else
		k_sleep(K_SECONDS(10));
		if (current_phy == LE_2M_PHY) {
			current_phy = LE_1M_PHY;
		} else {
			current_phy = LE_2M_PHY;
		}
#endif
		err = test_handler();
		if (err) {
			printk("Test command sending failed (err %d)\n", err);
		} else {
			test_started = true;
		}
	}
	return 0;
}
