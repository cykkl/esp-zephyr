/*
 * Car-side ESP <-> MSPM0G3507 UART link.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TI_UART_LINK_H
#define TI_UART_LINK_H

#include <stdint.h>

#include "car_control_protocol.h"

typedef int (*ti_telemetry_handler_t)(
	const struct car_telemetry_sample *sample);

int ti_uart_link_start(void);

void ti_uart_link_set_telemetry_handler(ti_telemetry_handler_t handler);

int ti_uart_link_send_car_command(uint16_t sequence,
				  enum car_command command,
				  uint8_t speed);

int ti_uart_link_send_imu(const struct car_imu_sample *sample);

#endif
