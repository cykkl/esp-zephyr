# ESP master to MSPM0G3507 UART link

The ESP32-C6 master uses UART1 while UART0 remains available for Zephyr logs
through the CH343 USB serial port.

## Wiring

| ESP32-C6 master | MSPM0G3507 |
|---|---|
| GPIO5 / UART1 TX | Selected UART RX |
| GPIO4 / UART1 RX | Selected UART TX |
| GND | GND |

Use 3.3 V TTL logic and cross TX to RX. Do not connect either UART pin to an
RS-232 voltage-level interface.

Default format: `115200 baud, 8 data bits, no parity, 1 stop bit, no flow
control`.

## Text protocol

Messages are ASCII text terminated by `\r\n`.

ESP to TI:

```text
ESP,READY,MASTER,115200
ESP,ALIVE,<sequence>,<uptime_ms>
ESP,ESPNOW_RX,<sequence>,<rssi>,<source_mac>
ESP,PONG,<uptime_ms>
ESP,STATUS,MASTER,<uptime_ms>
ESP,ECHO,<text>
ESP,ERR,UNKNOWN_COMMAND
```

TI to ESP:

```text
PING
STATUS
ECHO,<text>
```

Examples:

```text
TI sends:  PING\r\n
ESP sends: ESP,PONG,15234\r\n

TI sends:  ECHO,hello\r\n
ESP sends: ESP,ECHO,hello\r\n
```

The implementation is in `src/ti_uart_link.c`. Change the baud rate through
`CONFIG_TI_UART_BAUD_RATE` in `Kconfig`; change pins in
`boards/esp32c6_devkitc_hpcore.overlay`.
