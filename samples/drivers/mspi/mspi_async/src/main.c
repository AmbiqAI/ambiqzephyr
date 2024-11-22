/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/mspi.h>

#define MSPI_BUS                  DT_BUS(DT_ALIAS(dev0))
#define MSPI_TARGET               DT_ALIAS(dev0)
/* This size is arbitrary and should be modified based on how fast the controller is */
#define BUF_SIZE 16*1024

#define DEVICE_MEM_WRITE_INSTR    DT_PROP(DT_ALIAS(dev0), write_command)
#define DEVICE_MEM_READ_INSTR     DT_PROP(DT_ALIAS(dev0), read_command)
#define DEVICE_MEM_TX_DUMMY       DT_PROP(DT_ALIAS(dev0), tx_dummy)
#define DEVICE_MEM_RX_DUMMY       DT_PROP(DT_ALIAS(dev0), rx_dummy)
#define DEVICE_MEM_CMD_LENGTH     DT_ENUM_IDX(DT_ALIAS(dev0), command_length)
#define DEVICE_MEM_ADDR_LENGTH    DT_ENUM_IDX(DT_ALIAS(dev0), address_length)

uint8_t memc_write_buffer[BUF_SIZE];
uint8_t memc_read_buffer[BUF_SIZE];

struct user_context {
	uint32_t status;
	uint32_t total_packets;
};

void async_cb(struct mspi_callback_context *mspi_cb_ctx)
{
	struct user_context *usr_ctx = mspi_cb_ctx->ctx;
	struct mspi_event *evt = &mspi_cb_ctx->mspi_evt;

	if (evt->evt_data.packet_idx == usr_ctx->total_packets - 1) {
		usr_ctx->status = 0;
	}
}
/* The packets doesn't have to have equal size or consecutive address or same transfer direction */
struct mspi_xfer_packet packet1[] = {
	{
		.dir                = MSPI_TX,
		.cmd                = DEVICE_MEM_WRITE_INSTR,
		.address            = 0,
		.num_bytes          = BUF_SIZE / 4,
		.data_buf           = memc_write_buffer,
		.cb_mask            = MSPI_BUS_NO_CB,
	},
	{
		.dir                = MSPI_RX,
		.cmd                = DEVICE_MEM_READ_INSTR,
		.address            = 0,
		.num_bytes          = BUF_SIZE / 4,
		.data_buf           = memc_read_buffer,
		.cb_mask            = MSPI_BUS_NO_CB,
	},
	{
		.dir                = MSPI_TX,
		.cmd                = DEVICE_MEM_WRITE_INSTR,
		.address            = BUF_SIZE / 4,
		.num_bytes          = BUF_SIZE / 4,
		.data_buf           = memc_write_buffer + BUF_SIZE / 4,
		.cb_mask            = MSPI_BUS_NO_CB,
	},
	{
		.dir                = MSPI_RX,
		.cmd                = DEVICE_MEM_READ_INSTR,
		.address            = BUF_SIZE / 4,
		.num_bytes          = BUF_SIZE / 4,
		.data_buf           = memc_read_buffer + BUF_SIZE / 4,
		.cb_mask            = MSPI_BUS_XFER_COMPLETE_CB,
	},
};

struct mspi_xfer_packet packet2[] = {
	{
		.dir                = MSPI_TX,
		.cmd                = DEVICE_MEM_WRITE_INSTR,
		.address            = BUF_SIZE / 2,
		.num_bytes          = BUF_SIZE / 4,
		.data_buf           = memc_write_buffer + BUF_SIZE / 2,
		.cb_mask            = MSPI_BUS_NO_CB,
	},
	{
		.dir                = MSPI_TX,
		.cmd                = DEVICE_MEM_WRITE_INSTR,
		.address            = BUF_SIZE / 2 + BUF_SIZE / 4,
		.num_bytes          = BUF_SIZE / 4,
		.data_buf           = memc_write_buffer + BUF_SIZE / 2 + BUF_SIZE / 4,
		.cb_mask            = MSPI_BUS_NO_CB,
	},
	{
		.dir                = MSPI_RX,
		.cmd                = DEVICE_MEM_READ_INSTR,
		.address            = BUF_SIZE / 2 + BUF_SIZE / 4,
		.num_bytes          = BUF_SIZE / 4,
		.data_buf           = memc_read_buffer + BUF_SIZE / 2 + BUF_SIZE / 4,
		.cb_mask            = MSPI_BUS_NO_CB,
	},
	{
		.dir                = MSPI_RX,
		.cmd                = DEVICE_MEM_READ_INSTR,
		.address            = BUF_SIZE / 2,
		.num_bytes          = BUF_SIZE / 4,
		.data_buf           = memc_read_buffer + BUF_SIZE / 2,
		.cb_mask            = MSPI_BUS_XFER_COMPLETE_CB,
	},
};

struct mspi_xfer xfer1 = {
	.async                      = true,
	.xfer_mode                  = MSPI_DMA,
	.tx_dummy                   = DEVICE_MEM_TX_DUMMY,
	.rx_dummy                   = DEVICE_MEM_RX_DUMMY,
	.cmd_length                 = DEVICE_MEM_CMD_LENGTH,
	.addr_length                = DEVICE_MEM_ADDR_LENGTH,
	.priority                   = MSPI_XFER_PRIORITY_MEDIUM,
	.packets                    = (struct mspi_xfer_packet *)&packet1,
	.num_packet                 = sizeof(packet1) / sizeof(struct mspi_xfer_packet),
};

struct mspi_xfer xfer2 = {
	.async                      = true,
	.xfer_mode                  = MSPI_DMA,
	.tx_dummy                   = DEVICE_MEM_TX_DUMMY,
	.rx_dummy                   = DEVICE_MEM_RX_DUMMY,
	.cmd_length                 = DEVICE_MEM_CMD_LENGTH,
	.addr_length                = DEVICE_MEM_ADDR_LENGTH,
	.priority                   = MSPI_XFER_PRIORITY_MEDIUM,
	.packets                    = (struct mspi_xfer_packet *)&packet2,
	.num_packet                 = sizeof(packet2) / sizeof(struct mspi_xfer_packet),
};

int main(void)
{
	const struct device *controller = DEVICE_DT_GET(MSPI_BUS);
	struct mspi_dev_id dev_id = MSPI_DEVICE_ID_DT(MSPI_TARGET);
	volatile struct mspi_callback_context cb_ctx1, cb_ctx2;
	volatile struct user_context write_ctx, read_ctx;
	int i, j;
	int ret;

	/* Initialize write buffer */
	for (i = 0; i < BUF_SIZE; i++) {
		memc_write_buffer[i] = (uint8_t)i;
	}

	ret = mspi_dev_config(controller, &dev_id, MSPI_DEVICE_CONFIG_NONE, NULL);
	if (ret) {
		printk("Failed to get controller access\n");
		return 1;
	}

	write_ctx.total_packets = xfer1.num_packet;
	write_ctx.status        = ~0;
	cb_ctx1.ctx             = (void *)&write_ctx;
	ret = mspi_register_callback(controller, &dev_id, MSPI_BUS_XFER_COMPLETE,
				     (mspi_callback_handler_t)async_cb,
				     (struct mspi_callback_context *)&cb_ctx1);
	if (ret) {
		printk("Failed to register callback\n");
		return 1;
	}

	ret = mspi_transceive(controller, &dev_id, &xfer1);
	if (ret) {
		printk("Failed to send transceive\n");
		return 1;
	}

	read_ctx.total_packets  = xfer2.num_packet;
	read_ctx.status         = ~0;
	cb_ctx2.ctx             = (void *)&read_ctx;
	ret = mspi_register_callback(controller, &dev_id, MSPI_BUS_XFER_COMPLETE,
				     (mspi_callback_handler_t)async_cb,
				     (struct mspi_callback_context *)&cb_ctx2);
	if (ret) {
		printk("Failed to register callback\n");
		return 1;
	}

	ret = mspi_transceive(controller, &dev_id, &xfer2);
	if (ret) {
		printk("Failed to send transceive\n");
		return 1;
	}

	while (write_ctx.status != 0 || read_ctx.status != 0) {
		printk("Waiting for complete..., write completed:%d, read completed:%d\n",
			cb_ctx1.mspi_evt.evt_data.packet_idx,
			cb_ctx2.mspi_evt.evt_data.packet_idx);
		k_busy_wait(100000);
	}

	printk("write completed:%d, read completed:%d\n",
		cb_ctx1.mspi_evt.evt_data.packet_idx,
		cb_ctx2.mspi_evt.evt_data.packet_idx);

	for (j = 0; j < BUF_SIZE; j++) {
		if (memc_write_buffer[j] != memc_read_buffer[j]) {
			printk("Error: data differs at offset %d\n", j);
			break;
		}
	}
	if (j == BUF_SIZE) {
		printk("Read data matches written data\n");
	}

	return 0;
}
