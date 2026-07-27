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

#define CAR_PACKET_VERSION 3U
#define CAR_PACKET_TYPE_COMMAND 1U
#define CAR_PACKET_TYPE_TELEMETRY 2U
#define CAR_PACKET_TYPE_PARAMETER 3U
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
	CAR_COMMAND_TRACK_ON = 5,
	CAR_COMMAND_TRACK_OFF = 6,
};

/* 与 MSPM0G3507 ControlParam 的顺序保持一致。 */
enum car_parameter {
	CAR_PARAM_BASE = 0,
	CAR_PARAM_LIMIT,
	CAR_PARAM_MINIMUM,
	CAR_PARAM_SLEW,
	CAR_PARAM_KP,
	CAR_PARAM_KD,
	CAR_PARAM_GYRO,
	CAR_PARAM_OBSTACLE_STOP,
	CAR_PARAM_OBSTACLE_CLEAR,
	CAR_PARAM_SPEED_FULL_SCALE,
	CAR_PARAM_SPEED_KP,
	CAR_PARAM_SPEED_KI,
	CAR_PARAM_HEADING_KP,
	CAR_PARAM_HEADING_KI,
	CAR_PARAM_HEADING_KD,
	CAR_PARAM_HEADING_LIMIT,
	CAR_PARAM_HEADING_SIGN,
	CAR_PARAM_TURN_SPEED,
	CAR_PARAM_TURN_ANGLE,
	CAR_PARAM_TURN_TOLERANCE,
	CAR_PARAM_TURN_DETECT_CYCLES,
	CAR_PARAM_TURN_SIGN,
	CAR_PARAM_LINE_TRIM_LIMIT,
	CAR_PARAM_COUNT,
};

static inline bool car_command_is_tracking(enum car_command command)
{
	return command == CAR_COMMAND_TRACK_ON ||
	       command == CAR_COMMAND_TRACK_OFF;
}

struct __packed car_control_packet {
	uint8_t magic[4];
	uint8_t version;
	uint8_t type;
	uint8_t sequence_le[2];
	uint8_t command;
	uint8_t speed;
	uint8_t crc_le[2];
};

struct __packed car_parameter_packet {
	uint8_t magic[4];
	uint8_t version;
	uint8_t type;
	uint8_t sequence_le[2];
	uint8_t parameter;
	uint8_t value_le[4];
	uint8_t crc_le[2];
};

struct car_telemetry_sample {
	uint16_t sequence;
	uint32_t uptime_ms;
	int16_t gyro_x_dps_x10;
	int16_t gyro_y_dps_x10;
	int16_t gyro_z_dps_x10;
	int16_t roll_cdeg;
	int16_t pitch_cdeg;
	int16_t yaw_cdeg;
	uint8_t flags;
	int16_t heading_target_cdeg;
	int16_t heading_error_cdeg;
	int8_t heading_correction;
	int8_t left_duty;
	int8_t right_duty;
	uint8_t tracking_enabled;
	uint8_t line_bits;
	int8_t line_error;
	uint8_t line_active;
	uint8_t track_state;
	int8_t line_correction;
	int16_t target_left_cps;
	int16_t target_right_cps;
	int16_t measured_left_cps;
	int16_t measured_right_cps;
	uint8_t base_speed;
	uint8_t motor_limit;
	uint8_t motor_minimum;
	uint8_t output_slew;
	uint16_t kp_percent;
	uint16_t kd_percent;
	uint16_t gyro_percent;
	uint16_t speed_full_scale_cps;
	uint16_t speed_kp_percent;
	uint16_t speed_ki_percent;
	uint16_t heading_kp_percent;
	uint16_t heading_ki_percent;
	uint16_t heading_kd_percent;
	uint8_t heading_limit;
	uint8_t turn_speed;
	uint8_t turn_angle_deg;
	uint8_t turn_tolerance_deg;
	int8_t turn_sign;
	uint8_t turn_detect_cycles;
	uint16_t imu_age_ms;
	uint8_t line_trim_limit;
	int8_t heading_sign;
};

/* 车载 ESP 从维特 IMU 解码后，通过现有 ESP->3507 串口发送的姿态样本。 */
struct car_imu_sample {
	uint16_t sequence;
	int16_t gyro_x_dps_x10;
	int16_t gyro_y_dps_x10;
	int16_t gyro_z_dps_x10;
	int16_t roll_cdeg;
	int16_t pitch_cdeg;
	int16_t yaw_cdeg;
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
	uint8_t tracking_enabled;
	uint8_t line_bits;
	int8_t line_error;
	uint8_t line_active;
	uint8_t track_state;
	int8_t line_correction;
	uint8_t target_left_cps_le[2];
	uint8_t target_right_cps_le[2];
	uint8_t measured_left_cps_le[2];
	uint8_t measured_right_cps_le[2];
	uint8_t base_speed;
	uint8_t motor_limit;
	uint8_t motor_minimum;
	uint8_t output_slew;
	uint8_t kp_percent_le[2];
	uint8_t kd_percent_le[2];
	uint8_t gyro_percent_le[2];
	uint8_t speed_full_scale_cps_le[2];
	uint8_t speed_kp_percent_le[2];
	uint8_t speed_ki_percent_le[2];
	uint8_t heading_kp_percent_le[2];
	uint8_t heading_ki_percent_le[2];
	uint8_t heading_kd_percent_le[2];
	uint8_t heading_limit;
	uint8_t turn_speed;
	uint8_t turn_angle_deg;
	uint8_t turn_tolerance_deg;
	int8_t turn_sign;
	uint8_t turn_detect_cycles;
	uint8_t imu_age_ms_le[2];
	uint8_t line_trim_limit;
	int8_t heading_sign;
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
	packet->speed =
		command == CAR_COMMAND_STOP || car_command_is_tracking(command)
			? 0U
			: MIN(speed, CAR_MAX_SPEED);

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
	    packet->command > CAR_COMMAND_TRACK_OFF ||
	    packet->speed > CAR_MAX_SPEED ||
	    (car_command_is_tracking((enum car_command)packet->command) &&
	     packet->speed != 0U)) {
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

static inline void car_parameter_packet_init(
	struct car_parameter_packet *packet, uint16_t sequence,
	enum car_parameter parameter, int32_t value)
{
	static const uint8_t magic[4] = {'C', 'A', 'R', '1'};

	memset(packet, 0, sizeof(*packet));
	memcpy(packet->magic, magic, sizeof(packet->magic));
	packet->version = CAR_PACKET_VERSION;
	packet->type = CAR_PACKET_TYPE_PARAMETER;
	sys_put_le16(sequence, packet->sequence_le);
	packet->parameter = (uint8_t)parameter;
	sys_put_le32((uint32_t)value, packet->value_le);
	const uint16_t crc =
		car_crc16_ccitt((const uint8_t *)packet,
				offsetof(struct car_parameter_packet, crc_le));
	sys_put_le16(crc, packet->crc_le);
}

static inline bool car_parameter_packet_is_valid(
	const struct car_parameter_packet *packet)
{
	static const uint8_t magic[4] = {'C', 'A', 'R', '1'};

	if (memcmp(packet->magic, magic, sizeof(packet->magic)) != 0 ||
	    packet->version != CAR_PACKET_VERSION ||
	    packet->type != CAR_PACKET_TYPE_PARAMETER ||
	    packet->parameter >= CAR_PARAM_COUNT) {
		return false;
	}
	const uint16_t expected =
		car_crc16_ccitt((const uint8_t *)packet,
				offsetof(struct car_parameter_packet, crc_le));
	return sys_get_le16(packet->crc_le) == expected;
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
	sys_put_le16((uint16_t)sample->gyro_x_dps_x10, packet->gyro_x_le);
	sys_put_le16((uint16_t)sample->gyro_y_dps_x10, packet->gyro_y_le);
	sys_put_le16((uint16_t)sample->gyro_z_dps_x10, packet->gyro_z_le);
	sys_put_le16((uint16_t)sample->roll_cdeg, packet->roll_le);
	sys_put_le16((uint16_t)sample->pitch_cdeg, packet->pitch_le);
	sys_put_le16((uint16_t)sample->yaw_cdeg, packet->yaw_le);
	packet->flags = sample->flags;
	sys_put_le16((uint16_t)sample->heading_target_cdeg,
		     packet->heading_target_le);
	sys_put_le16((uint16_t)sample->heading_error_cdeg,
		     packet->heading_error_le);
	packet->heading_correction = sample->heading_correction;
	packet->left_duty = sample->left_duty;
	packet->right_duty = sample->right_duty;
	packet->tracking_enabled = sample->tracking_enabled != 0U ? 1U : 0U;
	packet->line_bits = sample->line_bits;
	packet->line_error = sample->line_error;
	packet->line_active = sample->line_active;
	packet->track_state = sample->track_state;
	packet->line_correction = sample->line_correction;
	sys_put_le16((uint16_t)sample->target_left_cps,
		     packet->target_left_cps_le);
	sys_put_le16((uint16_t)sample->target_right_cps,
		     packet->target_right_cps_le);
	sys_put_le16((uint16_t)sample->measured_left_cps,
		     packet->measured_left_cps_le);
	sys_put_le16((uint16_t)sample->measured_right_cps,
		     packet->measured_right_cps_le);
	packet->base_speed = sample->base_speed;
	packet->motor_limit = sample->motor_limit;
	packet->motor_minimum = sample->motor_minimum;
	packet->output_slew = sample->output_slew;
	sys_put_le16(sample->kp_percent, packet->kp_percent_le);
	sys_put_le16(sample->kd_percent, packet->kd_percent_le);
	sys_put_le16(sample->gyro_percent, packet->gyro_percent_le);
	sys_put_le16(sample->speed_full_scale_cps,
		     packet->speed_full_scale_cps_le);
	sys_put_le16(sample->speed_kp_percent, packet->speed_kp_percent_le);
	sys_put_le16(sample->speed_ki_percent, packet->speed_ki_percent_le);
	sys_put_le16(sample->heading_kp_percent, packet->heading_kp_percent_le);
	sys_put_le16(sample->heading_ki_percent, packet->heading_ki_percent_le);
	sys_put_le16(sample->heading_kd_percent, packet->heading_kd_percent_le);
	packet->heading_limit = sample->heading_limit;
	packet->turn_speed = sample->turn_speed;
	packet->turn_angle_deg = sample->turn_angle_deg;
	packet->turn_tolerance_deg = sample->turn_tolerance_deg;
	packet->turn_sign = sample->turn_sign;
	packet->turn_detect_cycles = sample->turn_detect_cycles;
	sys_put_le16(sample->imu_age_ms, packet->imu_age_ms_le);
	packet->line_trim_limit = sample->line_trim_limit;
	packet->heading_sign = sample->heading_sign;

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
	    packet->type != CAR_PACKET_TYPE_TELEMETRY ||
	    packet->tracking_enabled > 1U) {
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
		.gyro_x_dps_x10 = (int16_t)sys_get_le16(packet->gyro_x_le),
		.gyro_y_dps_x10 = (int16_t)sys_get_le16(packet->gyro_y_le),
		.gyro_z_dps_x10 = (int16_t)sys_get_le16(packet->gyro_z_le),
		.roll_cdeg = (int16_t)sys_get_le16(packet->roll_le),
		.pitch_cdeg = (int16_t)sys_get_le16(packet->pitch_le),
		.yaw_cdeg = (int16_t)sys_get_le16(packet->yaw_le),
		.flags = packet->flags,
		.heading_target_cdeg =
			(int16_t)sys_get_le16(packet->heading_target_le),
		.heading_error_cdeg =
			(int16_t)sys_get_le16(packet->heading_error_le),
		.heading_correction = packet->heading_correction,
		.left_duty = packet->left_duty,
		.right_duty = packet->right_duty,
		.tracking_enabled = packet->tracking_enabled,
		.line_bits = packet->line_bits,
		.line_error = packet->line_error,
		.line_active = packet->line_active,
		.track_state = packet->track_state,
		.line_correction = packet->line_correction,
		.target_left_cps =
			(int16_t)sys_get_le16(packet->target_left_cps_le),
		.target_right_cps =
			(int16_t)sys_get_le16(packet->target_right_cps_le),
		.measured_left_cps =
			(int16_t)sys_get_le16(packet->measured_left_cps_le),
		.measured_right_cps =
			(int16_t)sys_get_le16(packet->measured_right_cps_le),
		.base_speed = packet->base_speed,
		.motor_limit = packet->motor_limit,
		.motor_minimum = packet->motor_minimum,
		.output_slew = packet->output_slew,
		.kp_percent = sys_get_le16(packet->kp_percent_le),
		.kd_percent = sys_get_le16(packet->kd_percent_le),
		.gyro_percent = sys_get_le16(packet->gyro_percent_le),
		.speed_full_scale_cps =
			sys_get_le16(packet->speed_full_scale_cps_le),
		.speed_kp_percent =
			sys_get_le16(packet->speed_kp_percent_le),
		.speed_ki_percent =
			sys_get_le16(packet->speed_ki_percent_le),
		.heading_kp_percent =
			sys_get_le16(packet->heading_kp_percent_le),
		.heading_ki_percent =
			sys_get_le16(packet->heading_ki_percent_le),
		.heading_kd_percent =
			sys_get_le16(packet->heading_kd_percent_le),
		.heading_limit = packet->heading_limit,
		.turn_speed = packet->turn_speed,
		.turn_angle_deg = packet->turn_angle_deg,
		.turn_tolerance_deg = packet->turn_tolerance_deg,
		.turn_sign = packet->turn_sign,
		.turn_detect_cycles = packet->turn_detect_cycles,
		.imu_age_ms = sys_get_le16(packet->imu_age_ms_le),
		.line_trim_limit = packet->line_trim_limit,
		.heading_sign = packet->heading_sign,
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
	case CAR_COMMAND_TRACK_ON:
		return "TRACK_ON";
	case CAR_COMMAND_TRACK_OFF:
		return "TRACK_OFF";
	default:
		return "INVALID";
	}
}

#endif
