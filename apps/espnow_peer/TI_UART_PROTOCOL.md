# 小车端 ESP 与 MSPM0G3507 UART 协议

接小车的 ESP32-C6（从机）使用 UART1 与 MSPM0G3507 通信。UART0/CH343
仍用于烧录和 Zephyr 日志。

## 接线

| 小车端 ESP32-C6 | MSPM0G3507 |
|---|---|
| GPIO5 / UART1 TX | 所选 UART RX |
| GPIO4 / UART1 RX | 所选 UART TX |
| GND | GND |

使用 3.3 V TTL 电平并交叉连接 TX/RX，不要直接连接 RS-232 电平接口。

默认串口参数：`115200 baud, 8N1, no flow control`。

## ESP 发给 TI

所有消息均为 ASCII 文本，以 `\r\n` 结尾。

控制命令：

```text
CAR,CMD,<sequence>,<command>,<speed>
```

其中：

- `sequence`：0–65535 的递增序号
- `command`：`STOP`、`FORWARD`、`BACKWARD`、`LEFT`、`RIGHT`
- `speed`：0–100；STOP 固定为 0

示例：

```text
CAR,CMD,10,FORWARD,35\r\n
CAR,CMD,11,LEFT,25\r\n
CAR,CMD,14,STOP,0\r\n
```

状态消息：

```text
ESP,READY,CAR_NODE,115200
ESP,ALIVE,<uptime_ms>
ESP,PONG,<uptime_ms>
ESP,STATUS,CAR_NODE,<sequence>,<command>,<speed>
ESP,ECHO,<text>
ESP,ERR,UNKNOWN_COMMAND
```

## TI 可发给 ESP

```text
PING
STATUS
ECHO,<text>
```

## 自动停车

运动命令需要持续刷新。小车端 ESP 在运动状态下连续
`CONFIG_CAR_CONTROL_TIMEOUT_MS`（默认 700 ms）没有收到有效控制帧，会自动向
TI 发送：

```text
CAR,CMD,<last_sequence>,STOP,0\r\n
```

代码位于 `src/ti_uart_link.c`。波特率通过
`CONFIG_TI_UART_BAUD_RATE` 修改，引脚在
`boards/esp32c6_devkitc_hpcore.overlay` 中修改。
