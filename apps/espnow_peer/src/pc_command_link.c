/*
 * PC serial command input for the base ESP.
 *
 * Input format:
 *   CAR,<sequence>,<STOP|FORWARD|BACKWARD|LEFT|RIGHT>,<speed>\n
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pc_command_link.h"

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(pc_command_link, LOG_LEVEL_INF);

#define PC_UART_NODE DT_CHOSEN(zephyr_console)
#define PC_RX_LINE_SIZE 96
#define PC_TX_LINE_SIZE 192
#define PC_THREAD_STACK_SIZE 2048
#define PC_THREAD_PRIORITY 5

static const struct device *const pc_uart = DEVICE_DT_GET(PC_UART_NODE);
static pc_command_handler_t command_handler;

K_MUTEX_DEFINE(pc_tx_mutex);
K_MSGQ_DEFINE(pc_telemetry, sizeof(struct car_telemetry_sample), 8, 4);
K_THREAD_STACK_DEFINE(pc_thread_stack, PC_THREAD_STACK_SIZE);
static struct k_thread pc_thread_data;

static void pc_write_line(const char *format, ...)
{
	char line[PC_TX_LINE_SIZE];
	va_list args;

	va_start(args, format);
	const int length = vsnprintk(line, sizeof(line), format, args);
	va_end(args);

	if (length <= 0) {
		return;
	}

	const size_t bytes = MIN((size_t)length, sizeof(line) - 1U);

	k_mutex_lock(&pc_tx_mutex, K_FOREVER);
	for (size_t index = 0; index < bytes; index++) {
		uart_poll_out(pc_uart, line[index]);
	}
	k_mutex_unlock(&pc_tx_mutex);
}

static bool parse_u16(const char *text, uint16_t *value)
{
	char *end;
	errno = 0;
	const unsigned long parsed = strtoul(text, &end, 10);

	if (errno != 0 || end == text || *end != '\0' || parsed > UINT16_MAX) {
		return false;
	}

	*value = (uint16_t)parsed;
	return true;
}

static bool parse_speed(const char *text, uint8_t *speed)
{
	char *end;
	errno = 0;
	const unsigned long parsed = strtoul(text, &end, 10);

	if (errno != 0 || end == text || *end != '\0' ||
	    parsed > CAR_MAX_SPEED) {
		return false;
	}

	*speed = (uint8_t)parsed;
	return true;
}

static bool parse_command(const char *text, enum car_command *command)
{
	if (strcmp(text, "STOP") == 0) {
		*command = CAR_COMMAND_STOP;
	} else if (strcmp(text, "FORWARD") == 0) {
		*command = CAR_COMMAND_FORWARD;
	} else if (strcmp(text, "BACKWARD") == 0) {
		*command = CAR_COMMAND_BACKWARD;
	} else if (strcmp(text, "LEFT") == 0) {
		*command = CAR_COMMAND_LEFT;
	} else if (strcmp(text, "RIGHT") == 0) {
		*command = CAR_COMMAND_RIGHT;
	} else {
		return false;
	}

	return true;
}

static void handle_pc_line(char *line)
{
	char *save;
	char *prefix = strtok_r(line, ",", &save);
	char *sequence_text = strtok_r(NULL, ",", &save);
	char *command_text = strtok_r(NULL, ",", &save);
	char *speed_text = strtok_r(NULL, ",", &save);
	char *extra = strtok_r(NULL, ",", &save);
	uint16_t sequence;
	uint8_t speed;
	enum car_command command;

	if (prefix == NULL || sequence_text == NULL || command_text == NULL ||
	    speed_text == NULL || extra != NULL || strcmp(prefix, "CAR") != 0 ||
	    !parse_u16(sequence_text, &sequence) ||
	    !parse_command(command_text, &command) ||
	    !parse_speed(speed_text, &speed)) {
		LOG_WRN("Invalid PC command");
		pc_write_line("ESP,NACK,FORMAT\r\n");
		return;
	}

	if (command == CAR_COMMAND_STOP) {
		speed = 0U;
	}

	const int error = command_handler(sequence, command, speed);

	if (error == 0) {
		LOG_INF("PC command seq=%u %s speed=%u",
			sequence, car_command_name(command), speed);
		pc_write_line("ESP,ACK,%u,%s,%u\r\n",
			      sequence, car_command_name(command), speed);
	} else {
		LOG_ERR("PC command send failed: %d", error);
		pc_write_line("ESP,NACK,%u,SEND,%d\r\n", sequence, error);
	}
}

static void pc_thread(void *unused1, void *unused2, void *unused3)
{
	char line[PC_RX_LINE_SIZE];
	size_t length = 0U;
	bool overflow = false;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	k_thread_name_set(k_current_get(), "pc_command");
	pc_write_line("ESP,READY,PC_CONTROL\r\n");

	while (true) {
		unsigned char byte;

		while (uart_poll_in(pc_uart, &byte) == 0) {
			if (byte == '\r') {
				continue;
			}

			if (byte == '\n') {
				if (overflow) {
					LOG_WRN("PC command line too long");
					pc_write_line("ESP,NACK,LINE_TOO_LONG\r\n");
				} else if (length > 0U) {
					line[length] = '\0';
					handle_pc_line(line);
				}

				length = 0U;
				overflow = false;
			} else if (length < sizeof(line) - 1U) {
				line[length++] = (char)byte;
			} else {
				overflow = true;
			}
		}

		struct car_telemetry_sample telemetry;
		while (k_msgq_get(&pc_telemetry, &telemetry, K_NO_WAIT) == 0) {
			pc_write_line(
				"CAR,TEL,%u,%" PRIu32 ",%d,%d,%d,%d,%d,%d,%u,"
				"%d,%d,%d,%d,%d\r\n",
				telemetry.sequence, telemetry.uptime_ms,
				telemetry.gyro_x_dps, telemetry.gyro_y_dps,
				telemetry.gyro_z_dps, telemetry.roll_deg,
				telemetry.pitch_deg, telemetry.yaw_deg,
				telemetry.flags, telemetry.heading_target_deg,
				telemetry.heading_error_deg,
				telemetry.heading_correction,
				telemetry.left_duty, telemetry.right_duty);
		}

		k_sleep(K_MSEC(2));
	}
}

int pc_command_link_start(pc_command_handler_t handler)
{
	if (handler == NULL) {
		return -EINVAL;
	}

	if (!device_is_ready(pc_uart)) {
		LOG_ERR("PC UART device is not ready");
		return -ENODEV;
	}

	command_handler = handler;
	k_thread_create(&pc_thread_data, pc_thread_stack,
			K_THREAD_STACK_SIZEOF(pc_thread_stack),
			pc_thread, NULL, NULL, NULL,
			PC_THREAD_PRIORITY, 0, K_NO_WAIT);

	LOG_INF("PC control ready on UART0/CH343, 115200 8N1");
	return 0;
}

int pc_command_link_send_telemetry(
	const struct car_telemetry_sample *sample)
{
	if (sample == NULL) {
		return -EINVAL;
	}
	return k_msgq_put(&pc_telemetry, sample, K_NO_WAIT);
}
