/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

int main(void)
{
	// printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
	*(volatile uint32_t*)(0x40010200) = 0x73;
	*(volatile uint32_t*)(0x40010000) = 6;
	*(volatile uint32_t*)(0x40010200) = 0;

	return 0;
}
