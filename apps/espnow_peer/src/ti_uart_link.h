/*
 * ESP master <-> TI MCU UART link.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TI_UART_LINK_H
#define TI_UART_LINK_H

#include <stdint.h>

int ti_uart_link_start(void);

int ti_uart_link_report_espnow(uint32_t sequence, int8_t rssi,
			      const uint8_t source_mac[6]);

#endif
