/*
 * Binary car-control packet carried over ESP-NOW.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CAR_CONTROL_PROTOCOL_H
#define CAR_CONTROL_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#define CAR_PACKET_VERSION 1U
#define CAR_PACKET_TYPE_COMMAND 1U
#define CAR_PACKET_TYPE_TELEMETRY 2U
#define CAR_MAX_SPEED 100U
#define CAR_TELEMETRY_IMU_VALID BIT(0)
#define CAR_TELEMETRY_HEADING_ACTIVE BIT(1)
#define CAR_TELEMETRY_IMU_FAULT_SHIFT 2U
#define CAR_TELEMETRY_IMU_FAULT_MASK (0x3fU << CAR_TELEMETRY_IMU_FAULT_SHIFT)

enum car_imu_fault {
	CAR_IMU_FAULT_NONE = 0,
	CAR_IMU_FAULT_NOT_READY = 1,
	CAR_IMU_FAULT_NO_RESPONSE = 2,
	CAR_IMU_FAULT_TIMEOUT = 3,
	CAR_IMU_FAULT_STALE = 4,
	CAR_IMU_FAULT_IO = 5,
};

enum car_command {
	CAR_COMMAND_STOP = 0,
	CAR_COMMAND_FORWARD = 1,
	CAR_COMMAND_BACKWARD = 2,
	CAR_COMMAND_LEFT = 3,
	CAR_COMMAND_RIGHT = 4,
};

struct __packed car_control_packet {
	uint8_t magic[4];
	uint8_t version;
	uint8_t type;
	uint8_t sequence_le[2];
	uint8_t command;
	uint8_t speed;
	uint8_t crc_le[2];
};

struct car_telemetry_sample {
	uint16_t sequence;
	uint32_t uptime_ms;
	int16_t gyro_x_dps;
	int16_t gyro_y_dps;
	int16_t gyro_z_dps;
	int16_t roll_deg;
	int16_t pitch_deg;
	int16_t yaw_deg;
	uint8_t flags;
	int16_t heading_target_deg;
	int16_t heading_error_deg;
	int8_t heading_correction;
	int8_t left_duty;
	int8_t right_duty;
};

/* 车载 ESP 从维特 IMU 解码后，通过现有 ESP->3507 串口发送的姿态样本。 */
struct car_imu_sample {
	uint16_t sequence;
	int16_t gyro_x_dps;
	int16_t gyro_y_dps;
	int16_t gyro_z_dps;
	int16_t roll_deg;
	int16_t pitch_deg;
	int16_t yaw_deg;
	uint8_t flags;
};

struct __packed car_telemetry_packet {
	uint8_t magic[4];
	uint8_t version;
	uint8_t type;
	uint8_t sequence_le[2];
	uint8_t uptime_ms_le[4];
	uint8_t gyro_x_le[2];
	uint8_t gyro_y_le[2];
	uint8_t gyro_z_le[2];
	uint8_t roll_le[2];
	uint8_t pitch_le[2];
	uint8_t yaw_le[2];
	uint8_t flags;
	uint8_t heading_target_le[2];
	uint8_t heading_error_le[2];
	int8_t heading_correction;
	int8_t left_duty;
	int8_t right_duty;
	uint8_t crc_le[2];
};

static inline uint16_t car_crc16_ccitt(const uint8_t *data, size_t length)
{
	uint16_t crc = 0xffffU;

	for (size_t index = 0; index < length; index++) {
		crc ^= (uint16_t)data[index] << 8;
		for (int bit = 0; bit < 8; bit++) {
			crc = (crc & 0x8000U) != 0U
				      ? (uint16_t)((crc << 1) ^ 0x1021U)
				      : (uint16_t)(crc << 1);
		}
	}

	return crc;
}

static inline void car_control_packet_init(struct car_control_packet *packet,
					   uint16_t sequence,
					   enum car_command command,
					   uint8_t speed)
{
	static const uint8_t magic[4] = {'C', 'A', 'R', '1'};

	memset(packet, 0, sizeof(*packet));
	memcpy(packet->magic, magic, sizeof(packet->magic));
	packet->version = CAR_PACKET_VERSION;
	packet->type = CAR_PACKET_TYPE_COMMAND;
	sys_put_le16(sequence, packet->sequence_le);
	packet->command = (uint8_t)command;
	packet->speed = command == CAR_COMMAND_STOP ? 0U : MIN(speed, CAR_MAX_SPEED);

	const uint16_t crc =
		car_crc16_ccitt((const uint8_t *)packet,
				offsetof(struct car_control_packet, crc_le));

	sys_put_le16(crc, packet->crc_le);
}

static inline bool car_control_packet_is_valid(
	const struct car_control_packet *packet)
{
	static const uint8_t magic[4] = {'C', 'A', 'R', '1'};

	if (memcmp(packet->magic, magic, sizeof(packet->magic)) != 0 ||
	    packet->version != CAR_PACKET_VERSION ||
	    packet->type != CAR_PACKET_TYPE_COMMAND ||
	    packet->command > CAR_COMMAND_RIGHT ||
	    packet->speed > CAR_MAX_SPEED) {
		return false;
	}

	const uint16_t expected =
		car_crc16_ccitt((const uint8_t *)packet,
				offsetof(struct car_control_packet, crc_le));

	return sys_get_le16(packet->crc_le) == expected;
}

static inline uint16_t car_control_packet_sequence(
	const struct car_control_packet *packet)
{
	return sys_get_le16(packet->sequence_le);
}

static inline void car_telemetry_packet_init(
	struct car_telemetry_packet *packet,
	const struct car_telemetry_sample *sample)
{
	static const uint8_t magic[4] = {'C', 'A', 'R', '1'};

	memset(packet, 0, sizeof(*packet));
	memcpy(packet->magic, magic, sizeof(packet->magic));
	packet->version = CAR_PACKET_VERSION;
	packet->type = CAR_PACKET_TYPE_TELEMETRY;
	sys_put_le16(sample->sequence, packet->sequence_le);
	sys_put_le32(sample->uptime_ms, packet->uptime_ms_le);
	sys_put_le16((uint16_t)sample->gyro_x_dps, packet->gyro_x_le);
	sys_put_le16((uint16_t)sample->gyro_y_dps, packet->gyro_y_le);
	sys_put_le16((uint16_t)sample->gyro_z_dps, packet->gyro_z_le);
	sys_put_le16((uint16_t)sample->roll_deg, packet->roll_le);
	sys_put_le16((uint16_t)sample->pitch_deg, packet->pitch_le);
	sys_put_le16((uint16_t)sample->yaw_deg, packet->yaw_le);
	packet->flags = sample->flags;
	sys_put_le16((uint16_t)sample->heading_target_deg,
		     packet->heading_target_le);
	sys_put_le16((uint16_t)sample->heading_error_deg,
		     packet->heading_error_le);
	packet->heading_correction = sample->heading_correction;
	packet->left_duty = sample->left_duty;
	packet->right_duty = sample->right_duty;

	const uint16_t crc =
		car_crc16_ccitt((const uint8_t *)packet,
				offsetof(struct car_telemetry_packet, crc_le));
	sys_put_le16(crc, packet->crc_le);
}

static inline bool car_telemetry_packet_is_valid(
	const struct car_telemetry_packet *packet)
{
	static const uint8_t magic[4] = {'C', 'A', 'R', '1'};

	if (memcmp(packet->magic, magic, sizeof(packet->magic)) != 0 ||
	    packet->version != CAR_PACKET_VERSION ||
	    packet->type != CAR_PACKET_TYPE_TELEMETRY) {
		return false;
	}

	const uint16_t expected =
		car_crc16_ccitt((const uint8_t *)packet,
				offsetof(struct car_telemetry_packet, crc_le));
	return sys_get_le16(packet->crc_le) == expected;
}

static inline uint8_t car_telemetry_imu_fault(uint8_t flags)
{
	return (flags & CAR_TELEMETRY_IMU_FAULT_MASK) >>
	       CAR_TELEMETRY_IMU_FAULT_SHIFT;
}

static inline const char *car_imu_fault_name(uint8_t fault)
{
	switch (fault) {
	case CAR_IMU_FAULT_NONE:
		return "NONE";
	case CAR_IMU_FAULT_NOT_READY:
		return "I2C_NOT_READY";
	case CAR_IMU_FAULT_NO_RESPONSE:
		return "I2C_NACK";
	case CAR_IMU_FAULT_TIMEOUT:
		return "I2C_TIMEOUT";
	case CAR_IMU_FAULT_STALE:
		return "DATA_STALE";
	case CAR_IMU_FAULT_IO:
		return "I2C_IO";
	default:
		return "UNKNOWN";
	}
}

static inline void car_telemetry_packet_decode(
	const struct car_telemetry_packet *packet,
	struct car_telemetry_sample *sample)
{
	*sample = (struct car_telemetry_sample) {
		.sequence = sys_get_le16(packet->sequence_le),
		.uptime_ms = sys_get_le32(packet->uptime_ms_le),
		.gyro_x_dps = (int16_t)sys_get_le16(packet->gyro_x_le),
		.gyro_y_dps = (int16_t)sys_get_le16(packet->gyro_y_le),
		.gyro_z_dps = (int16_t)sys_get_le16(packet->gyro_z_le),
		.roll_deg = (int16_t)sys_get_le16(packet->roll_le),
		.pitch_deg = (int16_t)sys_get_le16(packet->pitch_le),
		.yaw_deg = (int16_t)sys_get_le16(packet->yaw_le),
		.flags = packet->flags,
		.heading_target_deg =
			(int16_t)sys_get_le16(packet->heading_target_le),
		.heading_error_deg =
			(int16_t)sys_get_le16(packet->heading_error_le),
		.heading_correction = packet->heading_correction,
		.left_duty = packet->left_duty,
		.right_duty = packet->right_duty,
	};
}

static inline const char *car_command_name(enum car_command command)
{
	switch (command) {
	case CAR_COMMAND_STOP:
		return "STOP";
	case CAR_COMMAND_FORWARD:
		return "FORWARD";
	case CAR_COMMAND_BACKWARD:
		return "BACKWARD";
	case CAR_COMMAND_LEFT:
		return "LEFT";
	case CAR_COMMAND_RIGHT:
		return "RIGHT";
	default:
		return "INVALID";
	}
}

#endif
