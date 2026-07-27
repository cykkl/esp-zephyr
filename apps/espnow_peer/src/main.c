/*
 * ESP-NOW bidirectional heartbeat test for two ESP32-C6 boards.
 *
 * Adapted from the Zephyr Project ESP-NOW sample.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <esp_now.h>
#include <esp_wifi.h>

#include "car_control_protocol.h"
#include "rgb_indicator.h"

#if defined(CONFIG_ESPNOW_NODE_ROLE_MASTER)
#include "pc_command_link.h"
#else
#include "imu_uart_link.h"
#include "ti_uart_link.h"
#endif

LOG_MODULE_REGISTER(espnow_peer, LOG_LEVEL_INF);

struct __packed espnow_heartbeat {
	uint32_t sequence;
	uint32_t uptime_ms;
	uint8_t source_mac[ESP_NOW_ETH_ALEN];
};

static const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

#if defined(CONFIG_ESPNOW_NODE_ROLE_MASTER)
static bool imu_state_known;
static bool last_imu_valid;
static uint8_t last_imu_fault;

static int forward_pc_command(uint16_t sequence, enum car_command command,
			      uint8_t speed)
{
	struct car_control_packet packet;

	car_control_packet_init(&packet, sequence, command, speed);

	const esp_err_t error =
		esp_now_send(broadcast_mac, (const uint8_t *)&packet,
			     sizeof(packet));

	if (error != ESP_OK) {
		LOG_ERR("Control send failed: 0x%x", error);
		return -EIO;
	}

	LOG_INF("Control TX seq=%u %s speed=%u",
		sequence, car_command_name(command), packet.speed);
	return 0;
}
#endif

#if defined(CONFIG_ESPNOW_NODE_ROLE_SLAVE)
static int forward_ti_telemetry(
	const struct car_telemetry_sample *sample)
{
	struct car_telemetry_packet packet;

	car_telemetry_packet_init(&packet, sample);
	const esp_err_t error =
		esp_now_send(broadcast_mac, (const uint8_t *)&packet,
			     sizeof(packet));

	return error == ESP_OK ? 0 : -EIO;
}
#endif

#if defined(CONFIG_ESPNOW_ROLE_RECEIVER) || defined(CONFIG_ESPNOW_ROLE_BIDIR)
static void receive_callback(const esp_now_recv_info_t *info, const uint8_t *data, int length)
{
	if (info == NULL || data == NULL || length <= 0 || info->rx_ctrl == NULL) {
		return;
	}

	rgb_indicator_notify(RGB_INDICATOR_RX);

	const uint8_t *source = info->src_addr;
	const uint8_t *destination = info->des_addr;
	const bool is_broadcast =
		memcmp(destination, broadcast_mac, ESP_NOW_ETH_ALEN) == 0;

	LOG_INF("RX [%s] from %02x:%02x:%02x:%02x:%02x:%02x RSSI=%d len=%d",
		is_broadcast ? "BCAST" : "UNICAST",
		source[0], source[1], source[2], source[3], source[4], source[5],
		info->rx_ctrl->rssi, length);

	if (length == sizeof(struct car_control_packet)) {
		const struct car_control_packet *packet =
			(const struct car_control_packet *)data;

		if (!car_control_packet_is_valid(packet)) {
			LOG_WRN("Rejected invalid car-control packet");
			return;
		}

		const uint16_t sequence =
			car_control_packet_sequence(packet);
		const enum car_command command =
			(enum car_command)packet->command;

		LOG_INF("Control RX seq=%u %s speed=%u",
			sequence, car_command_name(command), packet->speed);

#if defined(CONFIG_ESPNOW_NODE_ROLE_SLAVE)
		const int uart_error =
			ti_uart_link_send_car_command(sequence, command,
						      packet->speed);

		if (uart_error != 0) {
			LOG_ERR("Car UART queue failed: %d", uart_error);
		}
#endif
	} else if (length == sizeof(struct car_telemetry_packet)) {
		const struct car_telemetry_packet *packet =
			(const struct car_telemetry_packet *)data;
		struct car_telemetry_sample sample;

		if (!car_telemetry_packet_is_valid(packet)) {
			LOG_WRN("Rejected invalid car-telemetry packet");
			return;
		}
		car_telemetry_packet_decode(packet, &sample);

#if defined(CONFIG_ESPNOW_NODE_ROLE_MASTER)
		const uint8_t imu_fault =
			car_telemetry_imu_fault(sample.flags);
		const bool imu_valid =
			(sample.flags & CAR_TELEMETRY_IMU_VALID) != 0U;
		if (!imu_state_known || imu_valid != last_imu_valid ||
		    imu_fault != last_imu_fault) {
			if (imu_valid) {
				if (imu_state_known) {
					LOG_INF("Car IMU recovered");
				} else {
					LOG_INF("Car IMU ready");
				}
			} else {
				LOG_WRN("Car IMU fault: %s (%u)",
					car_imu_fault_name(imu_fault),
					imu_fault);
			}
			imu_state_known = true;
			last_imu_valid = imu_valid;
			last_imu_fault = imu_fault;
		}

		const int pc_error =
			pc_command_link_send_telemetry(&sample);
		if (pc_error != 0) {
			LOG_WRN("PC telemetry queue full: %d", pc_error);
		}
#endif
	} else if (length == sizeof(struct espnow_heartbeat)) {
		const struct espnow_heartbeat *heartbeat =
			(const struct espnow_heartbeat *)data;

		LOG_INF("RX heartbeat seq=%" PRIu32 " uptime=%" PRIu32
			" ms peer=%02x:%02x:%02x:%02x:%02x:%02x",
			heartbeat->sequence, heartbeat->uptime_ms,
			heartbeat->source_mac[0], heartbeat->source_mac[1],
			heartbeat->source_mac[2], heartbeat->source_mac[3],
			heartbeat->source_mac[4], heartbeat->source_mac[5]);
	} else {
		LOG_HEXDUMP_INF(data, MIN(length, 32), "RX payload");
	}
}
#endif

#if defined(CONFIG_ESPNOW_ROLE_SENDER) || defined(CONFIG_ESPNOW_ROLE_BIDIR)
static void send_callback(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
	if (info == NULL) {
		return;
	}

	const uint8_t *destination = info->des_addr;

	if (status == ESP_NOW_SEND_SUCCESS) {
		rgb_indicator_notify(RGB_INDICATOR_TX);
		LOG_INF("TX OK -> %02x:%02x:%02x:%02x:%02x:%02x",
			destination[0], destination[1], destination[2],
			destination[3], destination[4], destination[5]);
	} else {
		LOG_ERR("TX FAIL -> %02x:%02x:%02x:%02x:%02x:%02x",
			destination[0], destination[1], destination[2],
			destination[3], destination[4], destination[5]);
	}
}

#define BEACON_STACK_SIZE 2048
#define BEACON_PRIORITY 5

static void beacon_thread(void *unused1, void *unused2, void *unused3)
{
	uint32_t sequence = 0;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	k_thread_name_set(k_current_get(), "espnow_beacon");

	while (true) {
		struct espnow_heartbeat heartbeat = {
			.sequence = sequence++,
			.uptime_ms = k_uptime_get_32(),
		};
		esp_err_t error =
			esp_wifi_get_mac(WIFI_IF_STA, heartbeat.source_mac);

		if (error != ESP_OK) {
			LOG_ERR("esp_wifi_get_mac failed: 0x%x", error);
		} else {
			LOG_INF("TX heartbeat seq=%" PRIu32 " uptime=%" PRIu32
				" ms self=%02x:%02x:%02x:%02x:%02x:%02x",
				heartbeat.sequence, heartbeat.uptime_ms,
				heartbeat.source_mac[0], heartbeat.source_mac[1],
				heartbeat.source_mac[2], heartbeat.source_mac[3],
				heartbeat.source_mac[4], heartbeat.source_mac[5]);

			error = esp_now_send(broadcast_mac,
					     (const uint8_t *)&heartbeat,
					     sizeof(heartbeat));
			if (error != ESP_OK) {
				LOG_ERR("esp_now_send failed: 0x%x", error);
			}
		}

		k_sleep(K_SECONDS(CONFIG_ESPNOW_BEACON_INTERVAL_S));
	}
}

K_THREAD_STACK_DEFINE(beacon_stack, BEACON_STACK_SIZE);
static struct k_thread beacon_thread_data;
#endif

int main(void)
{
	wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
	esp_now_peer_info_t peer = {0};
	wifi_mode_t current_mode;
	uint8_t local_mac[ESP_NOW_ETH_ALEN];
	esp_err_t error;

	(void)rgb_indicator_start();

#if defined(CONFIG_ESPNOW_NODE_ROLE_SLAVE)
	const int uart_error = ti_uart_link_start();

	if (uart_error != 0) {
		LOG_WRN("Continuing without car UART link");
	}

	const int imu_error = imu_uart_link_start();

	if (imu_error != 0) {
		LOG_WRN("Continuing without IMU UART link");
	}
#endif

	error = esp_wifi_get_mode(&current_mode);
	if (error == ESP_ERR_WIFI_NOT_INIT) {
		error = esp_wifi_init(&wifi_config);
		if (error != ESP_OK) {
			LOG_ERR("esp_wifi_init failed: 0x%x", error);
			return -EIO;
		}
	}

	error = esp_wifi_set_storage(WIFI_STORAGE_RAM);
	if (error != ESP_OK) {
		LOG_ERR("esp_wifi_set_storage failed: 0x%x", error);
		return -EIO;
	}

	error = esp_wifi_set_mode(ESP32_WIFI_MODE_STA);
	if (error != ESP_OK) {
		LOG_ERR("esp_wifi_set_mode failed: 0x%x", error);
		return -EIO;
	}

	error = esp_wifi_start();
	if (error != ESP_OK) {
		LOG_ERR("esp_wifi_start failed: 0x%x", error);
		return -EIO;
	}

	error = esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL,
				     WIFI_SECOND_CHAN_NONE);
	if (error != ESP_OK) {
		LOG_ERR("esp_wifi_set_channel failed: 0x%x", error);
		return -EIO;
	}

	error = esp_now_init();
	if (error != ESP_OK) {
		LOG_ERR("esp_now_init failed: 0x%x", error);
		return -EIO;
	}

#if defined(CONFIG_ESPNOW_ROLE_RECEIVER) || defined(CONFIG_ESPNOW_ROLE_BIDIR)
	error = esp_now_register_recv_cb(receive_callback);
	if (error != ESP_OK) {
		LOG_ERR("esp_now_register_recv_cb failed: 0x%x", error);
		return -EIO;
	}
#endif

#if defined(CONFIG_ESPNOW_ROLE_SENDER) || defined(CONFIG_ESPNOW_ROLE_BIDIR)
	error = esp_now_register_send_cb(send_callback);
	if (error != ESP_OK) {
		LOG_ERR("esp_now_register_send_cb failed: 0x%x", error);
		return -EIO;
	}
#endif

	memcpy(peer.peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
	peer.channel = CONFIG_ESPNOW_CHANNEL;
	peer.ifidx = WIFI_IF_STA;
	peer.encrypt = false;

	error = esp_now_add_peer(&peer);
	if (error != ESP_OK) {
		LOG_ERR("esp_now_add_peer failed: 0x%x", error);
		return -EIO;
	}

#if defined(CONFIG_ESPNOW_NODE_ROLE_SLAVE)
	ti_uart_link_set_telemetry_handler(forward_ti_telemetry);
#endif

	error = esp_wifi_get_mac(WIFI_IF_STA, local_mac);
	if (error != ESP_OK) {
		LOG_ERR("esp_wifi_get_mac failed: 0x%x", error);
		return -EIO;
	}

	uint32_t version = 0;

	error = esp_now_get_version(&version);
	if (error != ESP_OK) {
		LOG_ERR("esp_now_get_version failed: 0x%x", error);
		return -EIO;
	}

	LOG_INF("ESP-NOW v%u ready channel=%u node=%s radio=%s",
		(unsigned int)version, CONFIG_ESPNOW_CHANNEL,
#if defined(CONFIG_ESPNOW_NODE_ROLE_MASTER)
		"MASTER",
#else
		"SLAVE",
#endif
#if defined(CONFIG_ESPNOW_ROLE_SENDER)
		"SENDER"
#elif defined(CONFIG_ESPNOW_ROLE_RECEIVER)
		"RECEIVER"
#else
		"BIDIR"
#endif
	);
	LOG_INF("Local MAC %02x:%02x:%02x:%02x:%02x:%02x",
		local_mac[0], local_mac[1], local_mac[2],
		local_mac[3], local_mac[4], local_mac[5]);

#if defined(CONFIG_ESPNOW_NODE_ROLE_MASTER)
	const int pc_error = pc_command_link_start(forward_pc_command);

	if (pc_error != 0) {
		LOG_ERR("PC control link failed: %d", pc_error);
		return pc_error;
	}
#endif

#if defined(CONFIG_ESPNOW_ROLE_SENDER) || defined(CONFIG_ESPNOW_ROLE_BIDIR)
	k_thread_create(&beacon_thread_data, beacon_stack,
			K_THREAD_STACK_SIZEOF(beacon_stack),
			beacon_thread, NULL, NULL, NULL,
			BEACON_PRIORITY, 0, K_NO_WAIT);
	LOG_INF("Beacon interval=%d seconds", CONFIG_ESPNOW_BEACON_INTERVAL_S);
#else
	LOG_INF("Receiver-only mode");
#endif

	return 0;
}
