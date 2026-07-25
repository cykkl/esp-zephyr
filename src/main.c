/*
 * Waveshare ESP32-C6-DEV-KIT diagnostic starter.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define RGB_NODE DT_ALIAS(led_strip)
#define BUTTON_NODE DT_ALIAS(sw0)

static const struct device *const rgb = DEVICE_DT_GET(RGB_NODE);
static const struct gpio_dt_spec boot_button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);

static const struct led_rgb colors[] = {
	{.r = 32, .g = 0, .b = 0},
	{.r = 0, .g = 32, .b = 0},
	{.r = 0, .g = 0, .b = 32},
	{.r = 16, .g = 16, .b = 16},
};

int main(void)
{
	struct led_rgb pixel = {0};
	bool previous_pressed = false;
	size_t color = 0;
	int ret;

	printk("\nWaveshare ESP32-C6 Zephyr diagnostic\n");
	printk("Board: %s\n", CONFIG_BOARD_TARGET);
	printk("Flash devicetree size: 16 MiB\n");
	printk("UART0: GPIO16 TX / GPIO17 RX, 115200 baud\n");
	printk("RGB: WS2812 on GPIO8; BOOT button: GPIO9\n");

	if (!device_is_ready(rgb)) {
		printk("ERROR: RGB LED device is not ready\n");
		return 0;
	}

	if (!gpio_is_ready_dt(&boot_button)) {
		printk("ERROR: BOOT button GPIO is not ready\n");
		return 0;
	}

	ret = gpio_pin_configure_dt(&boot_button, GPIO_INPUT);
	if (ret != 0) {
		printk("ERROR: cannot configure BOOT button (%d)\n", ret);
		return 0;
	}

	printk("Diagnostic running. Press BOOT to test the button.\n");

	while (true) {
		pixel = colors[color];
		ret = led_strip_update_rgb(rgb, &pixel, 1);
		if (ret != 0) {
			printk("ERROR: RGB update failed (%d)\n", ret);
		} else {
			printk("RGB step %u: R=%u G=%u B=%u\n",
			       (unsigned int)color, pixel.r, pixel.g, pixel.b);
		}

		for (int sample = 0; sample < 10; sample++) {
			const int state = gpio_pin_get_dt(&boot_button);

			if (state < 0) {
				printk("ERROR: BOOT button read failed (%d)\n", state);
				break;
			}

			const bool pressed = state != 0;
			if (pressed && !previous_pressed) {
				printk("BOOT button pressed\n");
			}
			previous_pressed = pressed;
			k_sleep(K_MSEC(100));
		}

		color = (color + 1) % ARRAY_SIZE(colors);
	}

	return 0;
}

