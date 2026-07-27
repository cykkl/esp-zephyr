/*
 * WitMotion standard 0x55 UART protocol receiver.
 *
 * The car-side ESP decodes angular-velocity (0x52) and angle (0x53) frames,
 * then queues a compact CAR,IMU line on the existing UART1 link to the 3507.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "imu_uart_link.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "car_control_protocol.h"
#include "ti_uart_link.h"

LOG_MODULE_REGISTER(imu_uart_link, LOG_LEVEL_INF);

#define IMU_UART_NODE DT_ALIAS(imu_uart)
#define WIT_FRAME_SIZE 11U
#define WIT_FRAME_HEADER 0x55U
#define WIT_FRAME_GYRO 0x52U
#define WIT_FRAME_ANGLE 0x53U
#define IMU_THREAD_STACK_SIZE 1536
#define IMU_THREAD_PRIORITY 5
#define IMU_PUBLISH_MIN_PERIOD_MS 20
#define IMU_FAULT_REPORT_PERIOD_MS 200
#define WIT_FACTORY_BAUD_RATE 9600U
#define WIT_BAUD_REGISTER 0x04U
#define WIT_BAUD_115200 0x0006U
#define WIT_BAUD_PROBE_MS 350
#define WIT_BAUD_PROBE_FRAME_COUNT 2U
#define WIT_COMMAND_SETTLE_MS 100
#define WIT_OUTPUT_GYRO_AND_ANGLE 0x000cU
#define WIT_OUTPUT_RATE_50_HZ 0x0008U

static const struct device *const imu_uart = DEVICE_DT_GET(IMU_UART_NODE);
static uint8_t frame[WIT_FRAME_SIZE];
static size_t frame_length;
static int16_t gyro[3];
static int16_t angle[3];
static int64_t gyro_time_ms;
static int64_t angle_time_ms;
static int64_t last_publish_ms;
static uint16_t sequence;
static bool gyro_received;
static bool angle_received;
static bool sample_published;
static bool checksum_error_seen;

K_THREAD_STACK_DEFINE(imu_stack, IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;

static int configure_local_uart(uint32_t baudrate)
{
	const struct uart_config config = {
		.baudrate = baudrate,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};

	return uart_configure(imu_uart, &config);
}

static void write_wit_bytes(const uint8_t *data, size_t length)
{
	for (size_t index = 0; index < length; index++) {
		uart_poll_out(imu_uart, data[index]);
	}
}

static void write_wit_register(uint8_t address, uint16_t value)
{
	const uint8_t command[] = {
		0xffU, 0xaaU, address,
		(uint8_t)(value & 0xffU),
		(uint8_t)(value >> 8),
	};

	write_wit_bytes(command, sizeof(command));
	k_sleep(K_MSEC(3));
}

static bool frame_checksum_valid(const uint8_t *data);

static bool probe_wit_sensor(int32_t timeout_ms)
{
	uint8_t probe_frame[WIT_FRAME_SIZE];
	size_t probe_length = 0U;
	uint8_t valid_frames = 0U;
	const int64_t deadline = k_uptime_get() + timeout_ms;
	unsigned char byte;

	while (k_uptime_get() < deadline) {
		if (uart_poll_in(imu_uart, &byte) != 0) {
			k_sleep(K_MSEC(1));
			continue;
		}

		if (probe_length == 0U && byte != WIT_FRAME_HEADER) {
			continue;
		}

		probe_frame[probe_length++] = byte;
		if (probe_length != WIT_FRAME_SIZE) {
			continue;
		}

		if (frame_checksum_valid(probe_frame)) {
			valid_frames++;
			if (valid_frames >= WIT_BAUD_PROBE_FRAME_COUNT) {
				return true;
			}
		}
		probe_length = 0U;
	}

	return false;
}

static void flush_wit_uart(void)
{
	unsigned char byte;

	while (uart_poll_in(imu_uart, &byte) == 0) {
		/* Discard bytes captured while changing baud rate. */
	}
	frame_length = 0U;
}

static void configure_wit_stream(const uint8_t *unlock, size_t unlock_size)
{
	write_wit_bytes(unlock, unlock_size);
	k_sleep(K_MSEC(WIT_COMMAND_SETTLE_MS));
	write_wit_register(0x02U, WIT_OUTPUT_GYRO_AND_ANGLE);
	k_sleep(K_MSEC(WIT_COMMAND_SETTLE_MS));
	write_wit_register(0x03U, WIT_OUTPUT_RATE_50_HZ);
	k_sleep(K_MSEC(WIT_COMMAND_SETTLE_MS));
}

/*
 * Probe the configured baud first so an already-migrated sensor incurs no
 * flash writes. If no valid frames are found, configure a factory-default
 * JY61P at 9600, move both sides to 115200, and save the setting once.
 */
static int configure_wit_sensor(void)
{
	static const uint8_t unlock[] = {0xffU, 0xaaU, 0x69U, 0x88U, 0xb5U};
	static const uint8_t save[] = {0xffU, 0xaaU, 0x00U, 0x00U, 0x00U};
	int error = configure_local_uart(CONFIG_IMU_UART_BAUD_RATE);

	if (error != 0) {
		return error;
	}
	k_sleep(K_MSEC(20));
	flush_wit_uart();
	if (probe_wit_sensor(WIT_BAUD_PROBE_MS)) {
		LOG_INF("JY61P detected at %d baud",
			CONFIG_IMU_UART_BAUD_RATE);
		configure_wit_stream(unlock, sizeof(unlock));
		flush_wit_uart();
		return 0;
	}

	error = configure_local_uart(WIT_FACTORY_BAUD_RATE);
	if (error != 0) {
		return error;
	}
	k_sleep(K_MSEC(20));
	flush_wit_uart();
	if (!probe_wit_sensor(WIT_BAUD_PROBE_MS)) {
		LOG_WRN("JY61P not detected at 115200 or factory 9600 baud");
		return configure_local_uart(CONFIG_IMU_UART_BAUD_RATE);
	}

	configure_wit_stream(unlock, sizeof(unlock));
	write_wit_register(WIT_BAUD_REGISTER, WIT_BAUD_115200);
	k_sleep(K_MSEC(WIT_COMMAND_SETTLE_MS));

	/*
	 * Some WitMotion revisions apply BAUD immediately and others after
	 * SAVE. One SAVE at each speed safely covers both behaviours.
	 */
	write_wit_bytes(save, sizeof(save));
	k_sleep(K_MSEC(WIT_COMMAND_SETTLE_MS));
	error = configure_local_uart(CONFIG_IMU_UART_BAUD_RATE);
	if (error != 0) {
		return error;
	}
	k_sleep(K_MSEC(WIT_COMMAND_SETTLE_MS));
	write_wit_bytes(save, sizeof(save));
	k_sleep(K_MSEC(WIT_COMMAND_SETTLE_MS));
	flush_wit_uart();

	if (probe_wit_sensor(WIT_BAUD_PROBE_MS)) {
		LOG_INF("JY61P migrated from 9600 to %d baud",
			CONFIG_IMU_UART_BAUD_RATE);
	} else {
		LOG_WRN("JY61P did not answer after %d-baud migration",
			CONFIG_IMU_UART_BAUD_RATE);

		/*
		 * Keep IMU data available when the module's RX wire is absent
		 * or a firmware revision rejects the BAUD write. A later reboot
		 * retries the migration.
		 */
		error = configure_local_uart(WIT_FACTORY_BAUD_RATE);
		if (error != 0) {
			return error;
		}
		k_sleep(K_MSEC(20));
		flush_wit_uart();
		if (probe_wit_sensor(WIT_BAUD_PROBE_MS)) {
			LOG_WRN("JY61P remains at 9600 baud; using fallback");
			return 0;
		}

		return configure_local_uart(CONFIG_IMU_UART_BAUD_RATE);
	}

	return 0;
}

static int16_t scale_axis(const uint8_t *raw, int32_t full_scale)
{
	const int16_t value = (int16_t)sys_get_le16(raw);

	return (int16_t)(((int32_t)value * full_scale) / 32768);
}

static bool frame_checksum_valid(const uint8_t *data)
{
	uint8_t checksum = 0U;

	for (size_t index = 0; index < WIT_FRAME_SIZE - 1U; index++) {
		checksum = (uint8_t)(checksum + data[index]);
	}
	return checksum == data[WIT_FRAME_SIZE - 1U];
}

/*
 * 维特标准帧固定为 11 字节：55、类型、8 字节负载、累加校验。
 * 这里只接受闭环所需的 0x52 角速度和 0x53 欧拉角。
 */
static void decode_frame(const uint8_t *data, int64_t now)
{
	if (!frame_checksum_valid(data)) {
		checksum_error_seen = true;
		return;
	}

	switch (data[1]) {
	case WIT_FRAME_GYRO:
		for (size_t axis = 0; axis < ARRAY_SIZE(gyro); axis++) {
			gyro[axis] = scale_axis(&data[2U + axis * 2U], 2000);
		}
		gyro_time_ms = now;
		gyro_received = true;
		break;
	case WIT_FRAME_ANGLE:
		for (size_t axis = 0; axis < ARRAY_SIZE(angle); axis++) {
			angle[axis] = scale_axis(&data[2U + axis * 2U], 180);
		}
		angle_time_ms = now;
		angle_received = true;
		break;
	default:
		break;
	}
}

static void consume_byte(uint8_t byte, int64_t now)
{
	if (frame_length == 0U && byte != WIT_FRAME_HEADER) {
		return;
	}

	frame[frame_length++] = byte;
	if (frame_length == WIT_FRAME_SIZE) {
		decode_frame(frame, now);
		frame_length = 0U;
	}
}

static void read_uart(void)
{
	unsigned char byte;

	while (uart_poll_in(imu_uart, &byte) == 0) {
		consume_byte(byte, k_uptime_get());
	}
}

static void publish_sample(int64_t now)
{
	const bool gyro_fresh = gyro_received &&
		now - gyro_time_ms < CONFIG_IMU_UART_STALE_TIMEOUT_MS;
	const bool angle_fresh = angle_received &&
		now - angle_time_ms < CONFIG_IMU_UART_STALE_TIMEOUT_MS;
	struct car_imu_sample sample = {
		.sequence = sequence++,
	};

	if (gyro_fresh && angle_fresh) {
		sample.gyro_x_dps = gyro[0];
		sample.gyro_y_dps = gyro[1];
		sample.gyro_z_dps = gyro[2];
		sample.roll_deg = angle[0];
		sample.pitch_deg = angle[1];
		sample.yaw_deg = angle[2];
		sample.flags = CAR_TELEMETRY_IMU_VALID;
		checksum_error_seen = false;
	} else {
		const enum car_imu_fault fault = checksum_error_seen
			? CAR_IMU_FAULT_IO : CAR_IMU_FAULT_TIMEOUT;

		sample.flags =
			(uint8_t)fault << CAR_TELEMETRY_IMU_FAULT_SHIFT;
	}

	(void)ti_uart_link_send_imu(&sample);
	last_publish_ms = now;
	sample_published = true;
}

static void imu_thread(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	k_thread_name_set(k_current_get(), "imu_uart");

	while (true) {
		read_uart();

		const int64_t now = k_uptime_get();
		const bool valid = gyro_received && angle_received &&
			now - gyro_time_ms < CONFIG_IMU_UART_STALE_TIMEOUT_MS &&
			now - angle_time_ms < CONFIG_IMU_UART_STALE_TIMEOUT_MS;
		const int64_t period = valid ? IMU_PUBLISH_MIN_PERIOD_MS :
			IMU_FAULT_REPORT_PERIOD_MS;

		if (!sample_published || now - last_publish_ms >= period) {
			publish_sample(now);
		}
		k_sleep(K_MSEC(2));
	}
}

int imu_uart_link_start(void)
{
	if (!device_is_ready(imu_uart)) {
		LOG_ERR("IMU UART0 device is not ready");
		return -ENODEV;
	}

	const int error = configure_wit_sensor();

	if (error != 0) {
		LOG_ERR("IMU UART configure failed: %d", error);
		return error;
	}

	k_thread_create(&imu_thread_data, imu_stack,
			K_THREAD_STACK_SIZEOF(imu_stack),
			imu_thread, NULL, NULL, NULL,
			IMU_THREAD_PRIORITY, 0, K_NO_WAIT);
	LOG_INF("IMU UART ready: UART0 GPIO2 TX / GPIO3 RX, %d 8N1",
		CONFIG_IMU_UART_BAUD_RATE);
	return 0;
}
