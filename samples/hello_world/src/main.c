/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(MAIN_DISPLAY_TEST, LOG_LEVEL_INF);


int main(void)
{
	LOG_INF("Hello World!");

	return 0;
}
