/*
 * RGB status indicator for ESP-NOW activity.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RGB_INDICATOR_H
#define RGB_INDICATOR_H

enum rgb_indicator_event {
	RGB_INDICATOR_TX,
	RGB_INDICATOR_RX,
};

int rgb_indicator_start(void);
void rgb_indicator_notify(enum rgb_indicator_event event);

#endif
