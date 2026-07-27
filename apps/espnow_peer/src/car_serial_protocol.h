/*
 * Framed binary protocol used on PC <-> base ESP and car ESP <-> TI UARTs.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CAR_SERIAL_PROTOCOL_H
#define CAR_SERIAL_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>

#define CAR_SERIAL_SYNC_0 0xa5U
#define CAR_SERIAL_SYNC_1 0x5aU
#define CAR_SERIAL_VERSION 2U
#define CAR_SERIAL_HEADER_SIZE 7U
#define CAR_SERIAL_CRC_SIZE 2U
#define CAR_SERIAL_MAX_PAYLOAD 25U
#define CAR_SERIAL_MAX_FRAME_SIZE \
	(CAR_SERIAL_HEADER_SIZE + CAR_SERIAL_MAX_PAYLOAD + CAR_SERIAL_CRC_SIZE)

#define CAR_SERIAL_TYPE_COMMAND 1U
#define CAR_SERIAL_TYPE_IMU 2U
#define CAR_SERIAL_TYPE_TELEMETRY 3U
#define CAR_SERIAL_TYPE_ACK 4U

#define CAR_SERIAL_COMMAND_PAYLOAD_SIZE 2U
#define CAR_SERIAL_IMU_PAYLOAD_SIZE 13U
#define CAR_SERIAL_TELEMETRY_PAYLOAD_SIZE 25U
#define CAR_SERIAL_ACK_PAYLOAD_SIZE 3U

#define CAR_SERIAL_ACK_OK 0U
#define CAR_SERIAL_ACK_BAD_FRAME 1U
#define CAR_SERIAL_ACK_BAD_COMMAND 2U
#define CAR_SERIAL_ACK_QUEUE_FULL 3U

/* IMU fixed-point units carried by IMU and TELEMETRY payloads. */
#define CAR_SERIAL_GYRO_SCALE 10
#define CAR_SERIAL_ANGLE_SCALE 100

static inline uint16_t car_serial_crc16(const uint8_t *data, size_t length)
{
	uint16_t crc = 0xffffU;

	for (size_t index = 0; index < length; ++index) {
		crc ^= (uint16_t)data[index] << 8;
		for (int bit = 0; bit < 8; ++bit) {
			crc = (crc & 0x8000U) != 0U
				      ? (uint16_t)((crc << 1) ^ 0x1021U)
				      : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

static inline bool car_serial_payload_size_valid(uint8_t type,
						 uint8_t payload_size)
{
	switch (type) {
	case CAR_SERIAL_TYPE_COMMAND:
		return payload_size == CAR_SERIAL_COMMAND_PAYLOAD_SIZE;
	case CAR_SERIAL_TYPE_IMU:
		return payload_size == CAR_SERIAL_IMU_PAYLOAD_SIZE;
	case CAR_SERIAL_TYPE_TELEMETRY:
		return payload_size == CAR_SERIAL_TELEMETRY_PAYLOAD_SIZE;
	case CAR_SERIAL_TYPE_ACK:
		return payload_size == CAR_SERIAL_ACK_PAYLOAD_SIZE;
	default:
		return false;
	}
}

static inline size_t car_serial_frame_size(uint8_t payload_size)
{
	return CAR_SERIAL_HEADER_SIZE + payload_size + CAR_SERIAL_CRC_SIZE;
}

static inline size_t car_serial_encode(uint8_t *frame, size_t capacity,
				      uint8_t type, uint16_t sequence,
				      const uint8_t *payload,
				      uint8_t payload_size)
{
	const size_t frame_size = car_serial_frame_size(payload_size);

	if (frame == NULL || payload == NULL || capacity < frame_size ||
	    !car_serial_payload_size_valid(type, payload_size)) {
		return 0U;
	}
	frame[0] = CAR_SERIAL_SYNC_0;
	frame[1] = CAR_SERIAL_SYNC_1;
	frame[2] = CAR_SERIAL_VERSION;
	frame[3] = type;
	frame[4] = payload_size;
	sys_put_le16(sequence, &frame[5]);
	memcpy(&frame[CAR_SERIAL_HEADER_SIZE], payload, payload_size);
	const uint16_t crc = car_serial_crc16(
		&frame[2], CAR_SERIAL_HEADER_SIZE - 2U + payload_size);
	sys_put_le16(crc, &frame[CAR_SERIAL_HEADER_SIZE + payload_size]);
	return frame_size;
}

static inline bool car_serial_frame_valid(const uint8_t *frame, size_t length)
{
	if (frame == NULL || length < CAR_SERIAL_HEADER_SIZE +
					   CAR_SERIAL_CRC_SIZE ||
	    frame[0] != CAR_SERIAL_SYNC_0 || frame[1] != CAR_SERIAL_SYNC_1 ||
	    frame[2] != CAR_SERIAL_VERSION ||
	    !car_serial_payload_size_valid(frame[3], frame[4]) ||
	    length != car_serial_frame_size(frame[4])) {
		return false;
	}
	const uint16_t expected = car_serial_crc16(
		&frame[2], CAR_SERIAL_HEADER_SIZE - 2U + frame[4]);
	return sys_get_le16(&frame[CAR_SERIAL_HEADER_SIZE + frame[4]]) ==
	       expected;
}

static inline uint16_t car_serial_sequence(const uint8_t *frame)
{
	return sys_get_le16(&frame[5]);
}

static inline uint8_t *car_serial_payload(uint8_t *frame)
{
	return &frame[CAR_SERIAL_HEADER_SIZE];
}

static inline const uint8_t *car_serial_const_payload(const uint8_t *frame)
{
	return &frame[CAR_SERIAL_HEADER_SIZE];
}

#endif
