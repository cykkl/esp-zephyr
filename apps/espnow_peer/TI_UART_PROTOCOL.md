# 小车通信二进制协议

PC、基站 ESP、车载 ESP 与 MSPM0G3507 使用统一的二进制串口帧。ESP-NOW
无线段继续使用原有固定结构和 CRC16；串口接收器支持任意分包、连续粘包、
错误字节自动重新同步和 CRC 错包丢弃。

## 接线

车载 ESP 与 MSPM0G3507 使用交叉连接的 3.3 V TTL UART，参数为
`460800 8N1`、无流控：

| 车载 ESP32-C6 | MSPM0G3507 |
| --- | --- |
| GPIO5 / UART1 TX | PA9 / UART1 RX |
| GPIO4 / UART1 RX | PA8 / UART1 TX |
| GND | GND |

WitMotion JY61P 使用车载 ESP 的 UART0，参数为 `115200 8N1`。从机 ESP
会在首次启动时自动把仍处于出厂 `9600` 的传感器迁移到 `115200` 并保存：

| WitMotion IMU | 车载 ESP32-C6 |
| --- | --- |
| RX | GPIO2 / UART0 TX |
| TX | GPIO3 / UART0 RX |
| GND | GND |

## 帧格式

所有多字节整数均为小端序。

| 偏移 | 长度 | 字段 |
| ---: | ---: | --- |
| 0 | 2 | 同步头 `A5 5A` |
| 2 | 1 | 协议版本，当前为 `2` |
| 3 | 1 | 消息类型 |
| 4 | 1 | 负载长度 |
| 5 | 2 | 16 位序号 |
| 7 | N | 负载 |
| 7+N | 2 | CRC16-CCITT |

CRC 初值为 `0xFFFF`、多项式为 `0x1021`，计算范围从版本字段到负载末尾，
不包含同步头和 CRC 字段。最大负载 25 字节，最大完整帧 34 字节。

## 消息类型

### `0x01` COMMAND，负载 2 字节

| 偏移 | 字段 |
| ---: | --- |
| 0 | 命令：0 STOP、1 FORWARD、2 BACKWARD、3 LEFT、4 RIGHT、5 TRACK_ON、6 TRACK_OFF |
| 1 | 速度 0–100；STOP/TRACK_ON/TRACK_OFF 必须为 0 |

`TRACK_ON` 退出远程手动模式、使能电机并进入自动循迹；`TRACK_OFF` 立即清零
左右轮输出并取消使能。运动命令仍受 ESP 700 ms 和 TI 1000 ms 两层失联停车
保护。

### `0x02` IMU，负载 13 字节

依次为 `gx/gy/gz/roll/pitch/yaw` 六个有符号 16 位定点整数，以及 1 字节
flags。角速度单位为 `0.1°/s`，姿态角单位为 `0.01°`；flags bit0 表示有效，
flags[7:2] 为故障码。车载 ESP 在 80 ms 内没有同时收到新角速度帧和姿态角
帧时撤销有效位，避免重复转发冻结数据。

### `0x03` TELEMETRY，负载 25 字节

依次为：

- `uptime_ms`：无符号 32 位；
- `gx/gy/gz`：三个有符号 16 位，单位 `0.1°/s`；
- `roll/pitch/yaw`：三个有符号 16 位，单位 `0.01°`；
- `flags`：1 字节；
- `heading_target/heading_error`：两个有符号 16 位，单位 `0.01°`；
- `correction/left/right`：三个有符号 8 位；
- `tracking`：0 或 1。

MSPM0 每 20 ms 发送一帧。车载 ESP 校验后转换为 ESP-NOW 遥测包，基站 ESP
再将同格式二进制串口帧交给 PC。

### `0x04` ACK，负载 3 字节

负载依次为命令、速度和状态：0=成功、1=坏帧、2=非法命令、3=队列满。

## 兼容性

ESP 和 MSPM0 暂时保留旧 ASCII 命令解析，便于串口手工调试；新版 PC 程序与
固件默认使用二进制帧。升级时必须同时更新 PC 程序、主 ESP、从 ESP 和
MSPM0G3507，不能混用新旧固件。
