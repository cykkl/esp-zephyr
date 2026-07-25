/*
 * ESP master <-> TI MCU UART link.
 *
 * Line protocol (115200 8N1 by default):
 *   ESP -> TI: ESP,READY,MASTER,<baud>
 *   ESP -> TI: ESP,ALIVE,<sequence>,<uptime_ms>
 *   ESP -> TI: ESP,ESPNOW_RX,<sequence>,<rssi>,<source_mac>
 *   TI  -> ESP: PING
 *   TI  -> ESP: STATUS
 *   TI  -> ESP: ECHO,<text>
 *
 * Every message ends with CRLF.
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
#define ESPNOW_MAC_LENGTH 6

struct espnow_rx_event {
	uint32_t sequence;
	int8_t rssi;
	uint8_t source_mac[ESPNOW_MAC_LENGTH];
};

static const struct device *const ti_uart = DEVICE_DT_GET(TI_UART_NODE);
static bool link_started;

K_MUTEX_DEFINE(ti_tx_mutex);
K_MSGQ_DEFINE(espnow_rx_events, sizeof(struct espnow_rx_event), 8, 4);
K_THREAD_STACK_DEFINE(ti_uart_stack, TI_UART_STACK_SIZE);
static struct k_thread ti_uart_thread_data;

static void ti_uart_write_line(const char *format, ...)
{
	char line[TI_TX_LINE_SIZE];
	va_list args;
	int length;

	va_start(args, format);
	length = vsnprintk(line, sizeof(line), format, args);
	va_end(args);

	if (length < 0) {
		return;
	}

	const size_t bytes =
		MIN((size_t)length, sizeof(line) - 1U);

	k_mutex_lock(&ti_tx_mutex, K_FOREVER);
	for (size_t index = 0; index < bytes; index++) {
		uart_poll_out(ti_uart, line[index]);
	}
	k_mutex_unlock(&ti_tx_mutex);
}

static void handle_ti_command(char *line)
{
	LOG_INF("TI -> ESP: %s", line);

	if (strcmp(line, "PING") == 0) {
		ti_uart_write_line("ESP,PONG,%" PRIu32 "\r\n",
				   k_uptime_get_32());
	} else if (strcmp(line, "STATUS") == 0) {
		ti_uart_write_line("ESP,STATUS,MASTER,%" PRIu32 "\r\n",
				   k_uptime_get_32());
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
	uint32_t alive_sequence = 0U;
	int64_t next_alive = k_uptime_get();

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	k_thread_name_set(k_current_get(), "ti_uart");
	ti_uart_write_line("ESP,READY,MASTER,%d\r\n",
			   CONFIG_TI_UART_BAUD_RATE);

	while (true) {
		struct espnow_rx_event event;

		read_ti_uart(line, &line_length, &overflow);

		while (k_msgq_get(&espnow_rx_events, &event, K_NO_WAIT) == 0) {
			ti_uart_write_line(
				"ESP,ESPNOW_RX,%" PRIu32 ",%d,"
				"%02x:%02x:%02x:%02x:%02x:%02x\r\n",
				event.sequence, event.rssi,
				event.source_mac[0], event.source_mac[1],
				event.source_mac[2], event.source_mac[3],
				event.source_mac[4], event.source_mac[5]);
		}

		const int64_t now = k_uptime_get();

		if (now >= next_alive) {
			ti_uart_write_line("ESP,ALIVE,%" PRIu32 ",%" PRIu32
					   "\r\n",
					   alive_sequence++, k_uptime_get_32());
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

	LOG_INF("TI UART ready: UART1 GPIO5 TX / GPIO4 RX, %d 8N1",
		CONFIG_TI_UART_BAUD_RATE);
	return 0;
}

int ti_uart_link_report_espnow(uint32_t sequence, int8_t rssi,
			      const uint8_t source_mac[ESPNOW_MAC_LENGTH])
{
	if (!link_started) {
		return -EAGAIN;
	}

	struct espnow_rx_event event = {
		.sequence = sequence,
		.rssi = rssi,
	};

	memcpy(event.source_mac, source_mac, sizeof(event.source_mac));
	return k_msgq_put(&espnow_rx_events, &event, K_NO_WAIT);
}
