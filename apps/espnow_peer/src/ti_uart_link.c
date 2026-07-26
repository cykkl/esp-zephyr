/*
 * Car-side ESP <-> MSPM0G3507 UART link.
 *
 * ESP -> TI:
 *   CAR,CMD,<sequence>,<command>,<speed>
 *   ESP,READY,CAR_NODE,<baud>
 *   ESP,ALIVE,<uptime_ms>
 *
 * TI -> ESP:
 *   PING
 *   STATUS
 *   ECHO,<text>
 *
 * Every line ends with CRLF.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ti_uart_link.h"

#include <errno.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(ti_uart_link, LOG_LEVEL_INF);

#define TI_UART_NODE DT_ALIAS(ti_uart)
#define TI_RX_LINE_SIZE 128
#define TI_TX_LINE_SIZE 160
#define TI_UART_STACK_SIZE 2048
#define TI_UART_PRIORITY 5

struct car_command_event {
	uint16_t sequence;
	uint8_t command;
	uint8_t speed;
};

static const struct device *const ti_uart = DEVICE_DT_GET(TI_UART_NODE);
static bool link_started;
static bool command_received;
static uint16_t current_sequence;
static enum car_command current_command = CAR_COMMAND_STOP;
static uint8_t current_speed;
static int64_t last_command_time;

K_MUTEX_DEFINE(ti_tx_mutex);
K_MSGQ_DEFINE(car_commands, sizeof(struct car_command_event), 8, 4);
K_THREAD_STACK_DEFINE(ti_uart_stack, TI_UART_STACK_SIZE);
static struct k_thread ti_uart_thread_data;

static void ti_uart_write_line(const char *format, ...)
{
	char line[TI_TX_LINE_SIZE];
	va_list args;

	va_start(args, format);
	const int length = vsnprintk(line, sizeof(line), format, args);
	va_end(args);

	if (length <= 0) {
		return;
	}

	const size_t bytes = MIN((size_t)length, sizeof(line) - 1U);

	k_mutex_lock(&ti_tx_mutex, K_FOREVER);
	for (size_t index = 0; index < bytes; index++) {
		uart_poll_out(ti_uart, line[index]);
	}
	k_mutex_unlock(&ti_tx_mutex);
}

static void send_command_to_ti(uint16_t sequence, enum car_command command,
			       uint8_t speed)
{
	if (command == CAR_COMMAND_STOP) {
		speed = 0U;
	}

	ti_uart_write_line("CAR,CMD,%u,%s,%u\r\n",
			   sequence, car_command_name(command), speed);

	current_sequence = sequence;
	current_command = command;
	current_speed = speed;
	last_command_time = k_uptime_get();
	command_received = true;

	LOG_INF("TI command seq=%u %s speed=%u",
		sequence, car_command_name(command), speed);
}

static void handle_ti_command(char *line)
{
	LOG_INF("TI -> ESP: %s", line);

	if (strcmp(line, "PING") == 0) {
		ti_uart_write_line("ESP,PONG,%" PRIu32 "\r\n",
				   k_uptime_get_32());
	} else if (strcmp(line, "STATUS") == 0) {
		ti_uart_write_line("ESP,STATUS,CAR_NODE,%u,%s,%u\r\n",
				   current_sequence,
				   car_command_name(current_command),
				   current_speed);
	} else if (strncmp(line, "ECHO,", 5) == 0) {
		ti_uart_write_line("ESP,ECHO,%s\r\n", line + 5);
	} else {
		ti_uart_write_line("ESP,ERR,UNKNOWN_COMMAND\r\n");
	}
}

static void read_ti_uart(char *line, size_t *line_length, bool *overflow)
{
	unsigned char byte;

	while (uart_poll_in(ti_uart, &byte) == 0) {
		if (byte == '\r') {
			continue;
		}

		if (byte == '\n') {
			if (*overflow) {
				LOG_WRN("TI UART input line too long");
				ti_uart_write_line("ESP,ERR,LINE_TOO_LONG\r\n");
			} else if (*line_length > 0U) {
				line[*line_length] = '\0';
				handle_ti_command(line);
			}

			*line_length = 0U;
			*overflow = false;
			continue;
		}

		if (*line_length < TI_RX_LINE_SIZE - 1U) {
			line[(*line_length)++] = (char)byte;
		} else {
			*overflow = true;
		}
	}
}

static void ti_uart_thread(void *unused1, void *unused2, void *unused3)
{
	char line[TI_RX_LINE_SIZE];
	size_t line_length = 0U;
	bool overflow = false;
	int64_t next_alive = k_uptime_get();

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	k_thread_name_set(k_current_get(), "ti_uart");
	ti_uart_write_line("ESP,READY,CAR_NODE,%d\r\n",
			   CONFIG_TI_UART_BAUD_RATE);

	while (true) {
		struct car_command_event event;

		read_ti_uart(line, &line_length, &overflow);

		while (k_msgq_get(&car_commands, &event, K_NO_WAIT) == 0) {
			send_command_to_ti(event.sequence,
					   (enum car_command)event.command,
					   event.speed);
		}

		const int64_t now = k_uptime_get();

		if (command_received && current_command != CAR_COMMAND_STOP &&
		    now - last_command_time >= CONFIG_CAR_CONTROL_TIMEOUT_MS) {
			send_command_to_ti(current_sequence,
					   CAR_COMMAND_STOP, 0U);
			LOG_WRN("Control timeout: failsafe STOP");
		}

		if (now >= next_alive) {
			ti_uart_write_line("ESP,ALIVE,%" PRIu32 "\r\n",
					   k_uptime_get_32());
			next_alive = now + CONFIG_TI_UART_STATUS_INTERVAL_MS;
		}

		k_sleep(K_MSEC(5));
	}
}

int ti_uart_link_start(void)
{
	if (!device_is_ready(ti_uart)) {
		LOG_ERR("TI UART1 device is not ready");
		return -ENODEV;
	}

	const struct uart_config config = {
		.baudrate = CONFIG_TI_UART_BAUD_RATE,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};
	const int error = uart_configure(ti_uart, &config);

	if (error != 0) {
		LOG_ERR("TI UART configure failed: %d", error);
		return error;
	}

	link_started = true;
	k_thread_create(&ti_uart_thread_data, ti_uart_stack,
			K_THREAD_STACK_SIZEOF(ti_uart_stack),
			ti_uart_thread, NULL, NULL, NULL,
			TI_UART_PRIORITY, 0, K_NO_WAIT);

	LOG_INF("Car UART ready: UART1 GPIO5 TX / GPIO4 RX, %d 8N1",
		CONFIG_TI_UART_BAUD_RATE);
	LOG_INF("Car failsafe timeout=%d ms",
		CONFIG_CAR_CONTROL_TIMEOUT_MS);
	return 0;
}

int ti_uart_link_send_car_command(uint16_t sequence,
				  enum car_command command,
				  uint8_t speed)
{
	if (!link_started) {
		return -EAGAIN;
	}

	if (command > CAR_COMMAND_RIGHT || speed > CAR_MAX_SPEED) {
		return -EINVAL;
	}

	const struct car_command_event event = {
		.sequence = sequence,
		.command = (uint8_t)command,
		.speed = command == CAR_COMMAND_STOP ? 0U : speed,
	};

	return k_msgq_put(&car_commands, &event, K_NO_WAIT);
}
