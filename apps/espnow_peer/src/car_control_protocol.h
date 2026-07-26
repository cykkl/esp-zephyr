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
#define CAR_MAX_SPEED 100U

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
