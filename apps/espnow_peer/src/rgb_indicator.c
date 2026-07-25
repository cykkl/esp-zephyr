/*
 * RGB status indicator for ESP-NOW activity.
 *
 * Idle: blue, with master/slave brightness.
 * TX:   red.
 * RX:   green.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rgb_indicator.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rgb_indicator, LOG_LEVEL_INF);

#define RGB_NODE DT_ALIAS(led_strip)
#define RGB_EVENT_QUEUE_DEPTH 8
#define RGB_THREAD_STACK_SIZE 1024
#define RGB_THREAD_PRIORITY 6

static const struct device *const rgb_led = DEVICE_DT_GET(RGB_NODE);
static bool indicator_started;

K_MSGQ_DEFINE(rgb_events, sizeof(uint8_t), RGB_EVENT_QUEUE_DEPTH, 1);
K_THREAD_STACK_DEFINE(rgb_thread_stack, RGB_THREAD_STACK_SIZE);
static struct k_thread rgb_thread_data;

static uint8_t role_brightness(void)
{
#if defined(CONFIG_ESPNOW_NODE_ROLE_MASTER)
	return CONFIG_ESPNOW_MASTER_LED_BRIGHTNESS;
#else
	return CONFIG_ESPNOW_SLAVE_LED_BRIGHTNESS;
#endif
}

static int set_pixel(uint8_t red, uint8_t green, uint8_t blue)
{
	struct led_rgb pixel = {
		.r = red,
		.g = green,
		.b = blue,
	};

	return led_strip_update_rgb(rgb_led, &pixel, 1);
}

static void set_idle_color(void)
{
	const int error = set_pixel(0, 0, role_brightness());

	if (error != 0) {
		LOG_ERR("Cannot set RGB idle color: %d", error);
	}
}

static void rgb_thread(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	k_thread_name_set(k_current_get(), "rgb_indicator");
	set_idle_color();

	while (true) {
		uint8_t event;

		k_msgq_get(&rgb_events, &event, K_FOREVER);

		const uint8_t brightness = role_brightness();
		int error;

		if (event == RGB_INDICATOR_TX) {
			error = set_pixel(brightness, 0, 0);
		} else {
			/* Green appears brighter than red/blue at the same value. */
			const uint8_t green = brightness == 0U
						    ? 0U
						    : MAX(1U, brightness / 2U);

			error = set_pixel(0, green, 0);
		}

		if (error != 0) {
			LOG_ERR("Cannot set RGB event color: %d", error);
		}

		k_sleep(K_MSEC(CONFIG_ESPNOW_LED_EVENT_DURATION_MS));
		set_idle_color();
	}
}

int rgb_indicator_start(void)
{
	if (!device_is_ready(rgb_led)) {
		LOG_ERR("RGB LED device is not ready");
		return -ENODEV;
	}

	indicator_started = true;
	k_thread_create(&rgb_thread_data, rgb_thread_stack,
			K_THREAD_STACK_SIZEOF(rgb_thread_stack),
			rgb_thread, NULL, NULL, NULL,
			RGB_THREAD_PRIORITY, 0, K_NO_WAIT);

	LOG_INF("RGB: idle=blue(%u), TX=red, RX=green, event=%d ms",
		role_brightness(), CONFIG_ESPNOW_LED_EVENT_DURATION_MS);
	return 0;
}

void rgb_indicator_notify(enum rgb_indicator_event event)
{
	if (!indicator_started) {
		return;
	}

	const uint8_t queued_event = (uint8_t)event;

	(void)k_msgq_put(&rgb_events, &queued_event, K_NO_WAIT);
}
