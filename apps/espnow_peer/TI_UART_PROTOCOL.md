# 小车端 ESP 与 MSPM0G3507 UART 协议

车载 ESP32-C6（从机）使用 UART1 与 MSPM0G3507 通信，原有接口不变。
从机 UART0 在应用启动后重映射到 GPIO2/GPIO3，专门接收 WitMotion IMU。
板载 CH343 的 GPIO16/GPIO17 不与 IMU 共线，仍可用于 ROM 下载；从机应用日志
不再输出到 UART0。

## 接线

| 车载 ESP32-C6 | MSPM0G3507 |
| --- | --- |
| GPIO5 / UART1 TX | PA9 / UART1 RX |
| GPIO4 / UART1 RX | PA8 / UART1 TX |
| GND | GND |

使用交叉连接的 3.3 V TTL UART，参数为 `460800 8N1`，无流控。

IMU 与车载 ESP 的接线：

| WitMotion IMU | 车载 ESP32-C6 |
| --- | --- |
| RX | GPIO2 / UART0 TX |
| TX | GPIO3 / UART0 RX |
| GND | GND |
| VCC | 按模块规格接电源 |

从机启动时兼容模块出厂 `9600 8N1`，随后把标准 WitMotion 模块切换成
`9600 8N1`、只输出角速度/姿态角、20 Hz。模块配置不写入永久存储，
每次从机启动都会重新设置，避免反复擦写传感器配置区。

## ESP 下发运动命令

所有消息使用 ASCII，并以 `\r\n` 结束：

```text
CAR,CMD,<sequence>,<command>,<speed>
```

- `sequence`：0–65535。
- `command`：`STOP`、`FORWARD`、`BACKWARD`、`LEFT`、`RIGHT`。
- `speed`：0–100；`STOP` 必须为 0。

示例：

```text
CAR,CMD,10,FORWARD,35\r\n
CAR,CMD,11,LEFT,25\r\n
CAR,CMD,14,STOP,0\r\n
```

MSPM0 成功入队后返回：

```text
CAR,ACK,<sequence>,<command>,<speed>\r\n
```

失败返回：

```text
CAR,ERR,<sequence>,BAD_FORMAT\r\n
CAR,ERR,<sequence>,UNKNOWN_COMMAND\r\n
CAR,ERR,<sequence>,STOP_SPEED_MUST_BE_ZERO\r\n
CAR,ERR,<sequence>,RANGE\r\n
CAR,ERR,<sequence>,QUEUE_FULL\r\n
```

MSPM0 启动串口服务时发送：

```text
CAR,READY,TMX,115200,<mspm_timeout_ms>\r\n
```

ESP 只记录 `CAR,ACK`、`CAR,ERR`、`CAR,READY` 和 TI 遥测，不再对未知 TI
输出自动回复，防止两端错误消息循环。

## ESP 下发 IMU 数据

车载 ESP 校验 WitMotion 的 11 字节二进制帧后，通过现有 UART1 向 3507 发送：

```text
CAR,IMU,<seq>,<gx>,<gy>,<gz>,<roll>,<pitch>,<yaw>,<flags>\r\n
```

- `gx/gy/gz` 来自 `0x55 0x52` 帧，单位 °/s。
- `roll/pitch/yaw` 来自 `0x55 0x53` 帧，单位 °。
- `flags` bit0 表示数据有效，`flags[7:2]` 表示故障码。
- 3507 超过 250 ms 没收到有效 `CAR,IMU` 时会立即退出航向闭环。

该消息与运动命令共用 GPIO5/GPIO4 对应的 UART1，不增加 3507 引脚。

## 3507 回传 IMU 与航向遥测

MSPM0 每 200 ms 向车载 ESP 发送：

```text
CAR,TEL,<seq>,<ms>,<gx>,<gy>,<gz>,<roll>,<pitch>,<yaw>,<flags>,
        <target>,<error>,<correction>,<left>,<right>
```

- `gx/gy/gz`：JY61P 三轴角速度，单位 °/s。
- `roll/pitch/yaw`：三轴姿态角，单位 °。
- `flags` bit0：IMU 数据有效；bit1：远程直行航向环已锁定。
- `flags[7:2]`：IMU 故障码，0=无故障、1=串口设备未就绪、
  2=设备无响应、3=接收超时、4=数据过期、5=校验或其他I/O错误。
- `target/error/correction`：目标航向、航向误差和左右轮差速修正。
- `left/right`：MSPM0 最终输出的左右轮 PWM 百分比。

车载 ESP 校验并解析该行，转换为带 CRC16 的二进制 ESP-NOW 遥测包。基站
ESP 校验无线包后再以同样的 `CAR,TEL` 文本行发给 PC，PC 控制界面实时显示
YAW、GYRO Z、IMU故障原因和航向锁定状态。即使 IMU 读取失败，MSPM0
仍会发送遥测；主机日志和 PC 界面只在故障状态发生变化时报告一次，
避免每 200 ms 重复刷屏。

## ESP 状态消息

车载 ESP 会发送：

```text
ESP,READY,CAR_NODE,460800
ESP,ALIVE,<uptime_ms>
ESP,PONG,<uptime_ms>
ESP,STATUS,CAR_NODE,<sequence>,<command>,<speed>
ESP,ECHO,<text>
```

MSPM0 将所有 `ESP,` 开头的状态消息作为信息处理，不回送错误。

MSPM0 仍可向 ESP 发送以下调试命令：

```text
PING
STATUS
ECHO,<text>
```

## 双层失联停车

- PC 控制程序运动期间每 200 ms 刷新一次命令。
- 车载 ESP 在 700 ms 没有新无线控制命令时向 MSPM0 发送 `STOP`。
- MSPM0 在默认 1000 ms 没有收到新的 `CAR,CMD` 运动命令时，自行把左右 PWM
  清零并取消电机使能。

因此 PC、无线链路、ESP 程序或 UART 任一环节失联，最终都会停车。参数分别由
ESP 的 `CONFIG_CAR_CONTROL_TIMEOUT_MS` 和 MSPM0 的
`CONFIG_TMX_REMOTE_COMMAND_TIMEOUT_MS` 配置。
