/*
 * PC serial command input for the base ESP.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PC_COMMAND_LINK_H
#define PC_COMMAND_LINK_H

#include <stdint.h>

#include "car_control_protocol.h"

typedef int (*pc_command_handler_t)(uint16_t sequence,
				    enum car_command command,
				    uint8_t speed);

int pc_command_link_start(pc_command_handler_t handler);

int pc_command_link_send_telemetry(
	const struct car_telemetry_sample *sample);

#endif
