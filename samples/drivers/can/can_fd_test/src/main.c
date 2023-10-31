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

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(can_fd);

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

struct k_thread rx_thread_data;
CAN_MSGQ_DEFINE(counter_msgq, 2);

void tx_callback(const struct device *dev, int error, void *user_data)
{
	char *sender = (char *)user_data;

	if (error != 0) {
		printf("Sending failed [%d]\nSender: %s\n", error, sender);
	}
}

int send_function(const struct device *can_dev)
{
	struct can_frame frame = {
			.flags = 0,
			.id = 0x1234567,
			.dlc = 8
	};

	frame.data[0] = 1;
	frame.data[1] = 2;
	frame.data[2] = 3;
	frame.data[3] = 4;
	frame.data[4] = 5;
	frame.data[5] = 6;
	frame.data[6] = 7;
	frame.data[7] = 8;

	return can_send(can_dev, &frame, K_FOREVER, tx_callback, "Sender 1");
}


void rx_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	const struct can_filter filter = {
		.flags = CAN_FILTER_DATA | CAN_FILTER_IDE,
		.id = COUNTER_MSG_ID,
		.mask = CAN_EXT_ID_MASK
	};
	struct can_frame frame;
	int filter_id;

	filter_id = can_add_rx_filter_msgq(can_dev, &counter_msgq, &filter);
	printf("Counter filter id: %d\n", filter_id);

	while (1) {
		k_msgq_get(&counter_msgq, &frame, K_FOREVER);

		if (frame.dlc != 2U) {
			printf("Wrong data length: %u\n", frame.dlc);
			continue;
		}

		printf("Counter received: %u\n",
		       sys_be16_to_cpu(UNALIGNED_GET((uint16_t *)&frame.data)));
	}
}

void can_rx_callback(const struct device *dev, struct can_frame *frame, void *arg)
{
    printf("Received CAN Frame\n");
    printf("ID: 0x%08X\n", frame->id);
    printf("DLC: %d\n", frame->dlc);
    printf("Data: ");
    for (int i = 0; i < frame->dlc; i++) {
        printf("%02X ", frame->data[i]);
    }
    printf("\n");
}

int main(void)
{

	// struct can_frame change_led_frame = {
	// 	.flags = 0,
	// 	.id = LED_MSG_ID,
	// 	.dlc = 1
	// };
	// struct can_frame counter_frame = {
	// 	.flags = CAN_FRAME_IDE,
	// 	.id = COUNTER_MSG_ID,
	// 	.dlc = 2
	// };

	// uint8_t toggle = 1;
	// uint16_t counter = 0;

	int ret;

	if (!device_is_ready(can_dev)) {
		printf("CAN: Device %s not ready.\n", can_dev->name);
		return 0;
	}

	can_set_bitrate(can_dev, 250000);

	ret = can_start(can_dev);
	if (ret != 0) {
		printf("Error starting CAN controller [%d]", ret);
		return 0;
	}

	struct can_frame frame = {
        .flags = 0,
        .id = COUNTER_MSG_ID,
        .dlc = 8,
        .data = {1,2,3,4,5,6,7,8}
	};

	k_tid_t rx_tid;
	rx_tid = k_thread_create(&rx_thread_data, rx_thread_stack,
				 K_THREAD_STACK_SIZEOF(rx_thread_stack),
				 rx_thread, NULL, NULL, NULL,
				 RX_THREAD_PRIORITY, 0, K_NO_WAIT);
	if (!rx_tid) {
		printf("ERROR spawning rx thread\n");
	}

	struct can_filter filter = {
        .id = COUNTER_MSG_ID,
        .mask = CAN_STD_ID_MASK,
        .flags = CAN_FILTER_DATA
    };

    ret = can_add_rx_filter(can_dev, can_rx_callback, NULL, &filter);
    if (ret < 0) {
        printf("Failed to add RX filter [%d]", ret);
        return 0;
    }

	while (1) {
		// change_led_frame.data[0] = toggle++ & 0x01 ? SET_LED : RESET_LED;
		// /* This sending call is none blocking. */
		// can_send(can_dev, &change_led_frame, K_FOREVER,
		// 	 NULL,
		// 	 "LED change");
		// k_sleep(SLEEP_TIME);

		// UNALIGNED_PUT(sys_cpu_to_be16(counter),
		// 	      (uint16_t *)&counter_frame.data[0]);
		// counter++;

		/* This sending call is blocking until the message is sent. */
		// can_send(can_dev, &counter_frame, K_MSEC(100), NULL, NULL);

		// k_sleep(SLEEP_TIME);
		// send_function(can_dev);

		ret = can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
		if (ret != 0) {
				LOG_ERR("Sending failed [%d]", ret);
		}

		k_sleep(SLEEP_TIME);
	}
}
