/*
 * Car-side ESP <-> MSPM0G3507 UART link.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TI_UART_LINK_H
#define TI_UART_LINK_H

#include <stdint.h>

#include "car_control_protocol.h"

int ti_uart_link_start(void);

int ti_uart_link_send_car_command(uint16_t sequence,
				  enum car_command command,
				  uint8_t speed);

#endif
