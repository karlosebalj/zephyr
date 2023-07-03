/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

// static const struct device *fpreader = DEVICE_DT_GET(DT_NODELABEL(fpreader));
// static const struct device *lock = DEVICE_DT_GET(DT_NODELABEL(lock));

int main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);
	return 0;
}
