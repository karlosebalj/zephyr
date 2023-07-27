/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/printk.h"
#include <zephyr/kernel.h>

#define MY_STACK_SIZE 500
#define MY_PRIORITY 5
#define SLEEPTIME 500

extern void my_entry_point(void *, void *, void *) {
	printk("thread_a: thread started \n");

	while (1) {
		printk("thread_a: thread loop \n");
		k_msleep(SLEEPTIME);
	}
}

K_THREAD_STACK_DEFINE(my_stack_area, MY_STACK_SIZE);
struct k_thread my_thread_data;

int main(void) {
  k_thread_create(
      &my_thread_data, my_stack_area, K_THREAD_STACK_SIZEOF(my_stack_area),
      my_entry_point, NULL, NULL, NULL, MY_PRIORITY, 0, K_NO_WAIT);
  k_thread_start(&my_thread_data);

  return 0;
}
