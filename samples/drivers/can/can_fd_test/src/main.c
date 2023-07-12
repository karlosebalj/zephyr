/*
 * Copyright (c) 2018 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>

#define RX_THREAD_STACK_SIZE 512
#define RX_THREAD_PRIORITY 2
#define STATE_POLL_THREAD_STACK_SIZE 512
#define STATE_POLL_THREAD_PRIORITY 2
#define LED_MSG_ID 0x10
#define COUNTER_MSG_ID 0x12345
#define SET_LED 1
#define RESET_LED 0
#define SLEEP_TIME K_MSEC(250)

K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(poll_state_stack, STATE_POLL_THREAD_STACK_SIZE);

const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

int main(void)
{

	struct can_frame change_led_frame = {
		.flags = 0,
		.id = LED_MSG_ID,
		.dlc = 1
	};
	struct can_frame counter_frame = {
		.flags = CAN_FRAME_IDE,
		.id = COUNTER_MSG_ID,
		.dlc = 2
	};
	uint8_t toggle = 1;
	uint16_t counter = 0;

	int ret;

	if (!device_is_ready(can_dev)) {
		printf("CAN: Device %s not ready.\n", can_dev->name);
		return 0;
	}


	ret = can_start(can_dev);
	if (ret != 0) {
		printf("Error starting CAN controller [%d]", ret);
		return 0;
	}

	while (1) {
		change_led_frame.data[0] = toggle++ & 0x01 ? SET_LED : RESET_LED;
		/* This sending call is none blocking. */
		can_send(can_dev, &change_led_frame, K_FOREVER,
			 NULL,
			 "LED change");
		k_sleep(SLEEP_TIME);

		UNALIGNED_PUT(sys_cpu_to_be16(counter),
			      (uint16_t *)&counter_frame.data[0]);
		counter++;
		/* This sending call is blocking until the message is sent. */
		can_send(can_dev, &counter_frame, K_MSEC(100), NULL, NULL);
		k_sleep(SLEEP_TIME);
	}
}
