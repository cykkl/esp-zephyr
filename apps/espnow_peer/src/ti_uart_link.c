/*
 * Car-side ESP <-> MSPM0G3507 UART link.
 *
 * ESP -> TI:
 *   CAR,CMD,<sequence>,<command>,<speed>
 *   ESP,READY,CAR_NODE,<baud>
 *   ESP,ALIVE,<uptime_ms>
 *
 * TI -> ESP:
 *   CAR,READY,...
 *   CAR,ACK,...
 *   CAR,ERR,...
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
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "car_serial_protocol.h"

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
static ti_telemetry_handler_t telemetry_handler;

K_MUTEX_DEFINE(ti_tx_mutex);
K_MSGQ_DEFINE(car_commands, sizeof(struct car_command_event), 8, 4);
K_MSGQ_DEFINE(imu_samples, sizeof(struct car_imu_sample), 8, 4);
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

static void ti_uart_write_bytes(const uint8_t *data, size_t length)
{
	k_mutex_lock(&ti_tx_mutex, K_FOREVER);
	k_sched_lock();
	for (size_t index = 0; index < length; ++index) {
		uart_poll_out(ti_uart, data[index]);
	}
	k_sched_unlock();
	k_mutex_unlock(&ti_tx_mutex);
}

static void send_command_to_ti(uint16_t sequence, enum car_command command,
			       uint8_t speed)
{
	uint8_t payload[CAR_SERIAL_COMMAND_PAYLOAD_SIZE];
	uint8_t frame[CAR_SERIAL_MAX_FRAME_SIZE];

	if (command == CAR_COMMAND_STOP || car_command_is_tracking(command)) {
		speed = 0U;
	}
	payload[0] = (uint8_t)command;
	payload[1] = speed;
	const size_t length = car_serial_encode(
		frame, sizeof(frame), CAR_SERIAL_TYPE_COMMAND, sequence,
		payload, sizeof(payload));
	ti_uart_write_bytes(frame, length);

	if (car_command_is_tracking(command)) {
		LOG_INF("TI tracking seq=%u %s", sequence,
			command == CAR_COMMAND_TRACK_ON ? "ON" : "OFF");
		return;
	}
	current_sequence = sequence;
	current_command = command;
	current_speed = speed;
	last_command_time = k_uptime_get();
	command_received = true;

	LOG_INF("TI command seq=%u %s speed=%u",
		sequence, car_command_name(command), speed);
}

/*
 * IMU 与运动命令共用现有 UART1。所有写操作仍由本模块串行化，
 * 避免两个线程同时发送时把 ASCII 行交叉在一起。
 */
static void send_imu_to_ti(const struct car_imu_sample *sample)
{
	uint8_t payload[CAR_SERIAL_IMU_PAYLOAD_SIZE];
	uint8_t frame[CAR_SERIAL_MAX_FRAME_SIZE];

	sys_put_le16((uint16_t)sample->gyro_x_dps, &payload[0]);
	sys_put_le16((uint16_t)sample->gyro_y_dps, &payload[2]);
	sys_put_le16((uint16_t)sample->gyro_z_dps, &payload[4]);
	sys_put_le16((uint16_t)sample->roll_deg, &payload[6]);
	sys_put_le16((uint16_t)sample->pitch_deg, &payload[8]);
	sys_put_le16((uint16_t)sample->yaw_deg, &payload[10]);
	payload[12] = sample->flags;
	const size_t length = car_serial_encode(
		frame, sizeof(frame), CAR_SERIAL_TYPE_IMU, sample->sequence,
		payload, sizeof(payload));
	ti_uart_write_bytes(frame, length);
}

static bool parse_long(const char *text, long minimum, long maximum,
		       long *value)
{
	char *end;

	errno = 0;
	const long parsed = strtol(text, &end, 10);
	if (errno != 0 || end == text || *end != '\0' ||
	    parsed < minimum || parsed > maximum) {
		return false;
	}
	*value = parsed;
	return true;
}

static bool parse_unsigned_long(const char *text, unsigned long maximum,
				unsigned long *value)
{
	char *end;

	if (text == NULL || *text == '\0' || *text == '-') {
		return false;
	}
	errno = 0;
	const unsigned long parsed = strtoul(text, &end, 10);
	if (errno != 0 || end == text || *end != '\0' || parsed > maximum) {
		return false;
	}
	*value = parsed;
	return true;
}

/*
 * MSPM0 使用紧凑 ASCII 帧跨过 UART；车载 ESP 在这里完成严格范围校验，
 * 随后转换成带 CRC 的二进制 ESP-NOW 遥测包。
 */
static bool parse_telemetry(char *line, struct car_telemetry_sample *sample)
{
	char *save;
	char *fields[15];

	char *token = strtok_r(line, ",", &save);
	if (token == NULL || strcmp(token, "CAR") != 0) {
		return false;
	}
	token = strtok_r(NULL, ",", &save);
	if (token == NULL || strcmp(token, "TEL") != 0) {
		return false;
	}
	for (size_t index = 0; index < ARRAY_SIZE(fields); index++) {
		fields[index] = strtok_r(NULL, ",", &save);
		if (fields[index] == NULL) {
			return false;
		}
	}
	if (strtok_r(NULL, ",", &save) != NULL) {
		return false;
	}

	unsigned long sequence;
	unsigned long uptime_ms;
	unsigned long flags;
	long gyro_x;
	long gyro_y;
	long gyro_z;
	long roll;
	long pitch;
	long yaw;
	long target;
	long error;
	long correction;
	long left;
	long right;
	unsigned long tracking_enabled;

	if (!parse_unsigned_long(fields[0], UINT16_MAX, &sequence) ||
	    !parse_unsigned_long(fields[1], UINT32_MAX, &uptime_ms) ||
	    !parse_long(fields[2], INT16_MIN, INT16_MAX, &gyro_x) ||
	    !parse_long(fields[3], INT16_MIN, INT16_MAX, &gyro_y) ||
	    !parse_long(fields[4], INT16_MIN, INT16_MAX, &gyro_z) ||
	    !parse_long(fields[5], INT16_MIN, INT16_MAX, &roll) ||
	    !parse_long(fields[6], INT16_MIN, INT16_MAX, &pitch) ||
	    !parse_long(fields[7], INT16_MIN, INT16_MAX, &yaw) ||
	    !parse_unsigned_long(fields[8], UINT8_MAX, &flags) ||
	    !parse_long(fields[9], INT16_MIN, INT16_MAX, &target) ||
	    !parse_long(fields[10], INT16_MIN, INT16_MAX, &error) ||
	    !parse_long(fields[11], INT8_MIN, INT8_MAX, &correction) ||
	    !parse_long(fields[12], INT8_MIN, INT8_MAX, &left) ||
	    !parse_long(fields[13], INT8_MIN, INT8_MAX, &right) ||
	    !parse_unsigned_long(fields[14], 1U, &tracking_enabled)) {
		return false;
	}

	*sample = (struct car_telemetry_sample) {
		.sequence = (uint16_t)sequence,
		.uptime_ms = (uint32_t)uptime_ms,
		.gyro_x_dps = (int16_t)gyro_x,
		.gyro_y_dps = (int16_t)gyro_y,
		.gyro_z_dps = (int16_t)gyro_z,
		.roll_deg = (int16_t)roll,
		.pitch_deg = (int16_t)pitch,
		.yaw_deg = (int16_t)yaw,
		.flags = (uint8_t)flags,
		.heading_target_deg = (int16_t)target,
		.heading_error_deg = (int16_t)error,
		.heading_correction = (int8_t)correction,
		.left_duty = (int8_t)left,
		.right_duty = (int8_t)right,
		.tracking_enabled = (uint8_t)tracking_enabled,
	};
	return true;
}

static void handle_ti_command(char *line)
{
	if (strncmp(line, "CAR,TEL,", 8) == 0) {
		struct car_telemetry_sample sample;

		if (!parse_telemetry(line, &sample)) {
			LOG_WRN("Rejected malformed TI telemetry");
		} else if (telemetry_handler != NULL) {
			const int error = telemetry_handler(&sample);
			if (error != 0) {
				LOG_WRN("Telemetry forwarding failed: %d", error);
			}
		}
	} else if (strncmp(line, "CAR,ACK,", 8) == 0) {
		LOG_INF("TI ACK: %s", line + 8);
	} else if (strncmp(line, "CAR,ERR,", 8) == 0) {
		LOG_WRN("TI rejected command: %s", line + 8);
	} else if (strncmp(line, "CAR,", 4) == 0 ||
		   strncmp(line, "STAT ", 5) == 0 ||
		   strncmp(line, "OK ", 3) == 0 ||
		   strncmp(line, "ERR ", 4) == 0 ||
		   strcmp(line, "PONG") == 0) {
		/*
		 * These are TI responses/telemetry. They are logged only; replying
		 * would make two unknown-command handlers echo errors forever.
		 */
		LOG_INF("TI -> ESP: %s", line);
	} else if (strcmp(line, "PING") == 0) {
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
		LOG_WRN("Ignoring unknown TI UART line: %s", line);
	}
}

static void handle_ti_frame(const uint8_t *frame, size_t length)
{
	if (!car_serial_frame_valid(frame, length)) {
		LOG_WRN("Rejected TI frame CRC/type");
		return;
	}
	const uint16_t sequence = car_serial_sequence(frame);
	const uint8_t *payload = car_serial_const_payload(frame);

	if (frame[3] == CAR_SERIAL_TYPE_TELEMETRY) {
		struct car_telemetry_sample sample = {
			.sequence = sequence,
			.uptime_ms = sys_get_le32(&payload[0]),
			.gyro_x_dps = (int16_t)sys_get_le16(&payload[4]),
			.gyro_y_dps = (int16_t)sys_get_le16(&payload[6]),
			.gyro_z_dps = (int16_t)sys_get_le16(&payload[8]),
			.roll_deg = (int16_t)sys_get_le16(&payload[10]),
			.pitch_deg = (int16_t)sys_get_le16(&payload[12]),
			.yaw_deg = (int16_t)sys_get_le16(&payload[14]),
			.flags = payload[16],
			.heading_target_deg =
				(int16_t)sys_get_le16(&payload[17]),
			.heading_error_deg =
				(int16_t)sys_get_le16(&payload[19]),
			.heading_correction = (int8_t)payload[21],
			.left_duty = (int8_t)payload[22],
			.right_duty = (int8_t)payload[23],
			.tracking_enabled = payload[24],
		};
		if (sample.tracking_enabled > 1U) {
			LOG_WRN("Rejected TI telemetry tracking flag");
		} else if (telemetry_handler != NULL) {
			const int error = telemetry_handler(&sample);
			if (error != 0) {
				LOG_WRN("Telemetry forwarding failed: %d", error);
			}
		}
	} else if (frame[3] == CAR_SERIAL_TYPE_ACK) {
		LOG_INF("TI ACK seq=%u command=%u status=%u",
			sequence, payload[0], payload[2]);
	}
}

static void read_ti_uart(char *line, size_t *line_length, bool *overflow)
{
	unsigned char byte;
	static uint8_t frame[CAR_SERIAL_MAX_FRAME_SIZE];
	static size_t frame_length;
	static size_t expected_frame_length;

	while (uart_poll_in(ti_uart, &byte) == 0) {
		if (frame_length > 0U || byte == CAR_SERIAL_SYNC_0) {
			if (frame_length == 0U) {
				frame[frame_length++] = byte;
				continue;
			}
			if (frame_length == 1U && byte != CAR_SERIAL_SYNC_1) {
				frame_length = byte == CAR_SERIAL_SYNC_0 ? 1U : 0U;
				expected_frame_length = 0U;
				continue;
			}
			if (frame_length < sizeof(frame)) {
				frame[frame_length++] = byte;
			} else {
				frame_length = 0U;
				expected_frame_length = 0U;
				continue;
			}
			if (frame_length == CAR_SERIAL_HEADER_SIZE) {
				if (frame[2] != CAR_SERIAL_VERSION ||
				    !car_serial_payload_size_valid(frame[3],
								   frame[4])) {
					frame_length = 0U;
					continue;
				}
				expected_frame_length =
					car_serial_frame_size(frame[4]);
			}
			if (expected_frame_length > 0U &&
			    frame_length == expected_frame_length) {
				handle_ti_frame(frame, frame_length);
				frame_length = 0U;
				expected_frame_length = 0U;
			}
			continue;
		}

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

	while (true) {
		struct car_command_event event;
		struct car_imu_sample imu_sample;

		read_ti_uart(line, &line_length, &overflow);

		while (k_msgq_get(&car_commands, &event, K_NO_WAIT) == 0) {
			send_command_to_ti(event.sequence,
					   (enum car_command)event.command,
					   event.speed);
		}
		while (k_msgq_get(&imu_samples, &imu_sample, K_NO_WAIT) == 0) {
			send_imu_to_ti(&imu_sample);
		}

		const int64_t now = k_uptime_get();

		if (command_received && current_command != CAR_COMMAND_STOP &&
		    now - last_command_time >= CONFIG_CAR_CONTROL_TIMEOUT_MS) {
			send_command_to_ti(current_sequence,
					   CAR_COMMAND_STOP, 0U);
			LOG_WRN("Control timeout: failsafe STOP");
		}

		if (now >= next_alive) {
			next_alive = now + CONFIG_TI_UART_STATUS_INTERVAL_MS;
		}

		k_sleep(K_MSEC(1));
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

void ti_uart_link_set_telemetry_handler(ti_telemetry_handler_t handler)
{
	telemetry_handler = handler;
}

int ti_uart_link_send_car_command(uint16_t sequence,
				  enum car_command command,
				  uint8_t speed)
{
	if (!link_started) {
		return -EAGAIN;
	}

	if (command > CAR_COMMAND_TRACK_OFF || speed > CAR_MAX_SPEED ||
	    (car_command_is_tracking(command) && speed != 0U)) {
		return -EINVAL;
	}

	const struct car_command_event event = {
		.sequence = sequence,
		.command = (uint8_t)command,
		.speed = command == CAR_COMMAND_STOP ? 0U : speed,
	};

	return k_msgq_put(&car_commands, &event, K_NO_WAIT);
}

int ti_uart_link_send_imu(const struct car_imu_sample *sample)
{
	if (!link_started) {
		return -EAGAIN;
	}
	if (sample == NULL) {
		return -EINVAL;
	}
	return k_msgq_put(&imu_samples, sample, K_NO_WAIT);
}
