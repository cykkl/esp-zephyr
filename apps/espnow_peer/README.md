# ESP-NOW 双向通信测试

两块 Waveshare ESP32-C6 分别烧录主机和从机 BIDIR 固件，在 Wi-Fi 信道 6
上每两秒广播一次心跳，同时接收另一块板的心跳。无需路由器。

每个心跳包含：

- 递增序号
- 设备启动时间
- 发送方 MAC 地址

接收日志还会显示 RSSI。

板载 RGB 使用同一种蓝色显示节点身份，避免颜色差异与状态含义混淆：

- 待机：蓝色；主机亮度 `8/255`，从机亮度 `2/255`
- ESP-NOW 发送成功：红色闪烁
- ESP-NOW 收到数据：绿色闪烁

亮度值可通过 `CONFIG_ESPNOW_MASTER_LED_BRIGHTNESS` 和
`CONFIG_ESPNOW_SLAVE_LED_BRIGHTNESS` 修改。通信颜色默认保持 180 ms，可通过
`CONFIG_ESPNOW_LED_EVENT_DURATION_MS` 修改。发送和接收事件使用队列依次显示。

本应用参考 Zephyr 官方 ESP-NOW 示例，并针对当前两块 16 MiB Flash 的
Waveshare 板增加了 Devicetree 覆盖。

## 使用

在项目根目录执行：

```powershell
.\scripts\build-espnow.ps1 -Pristine
.\scripts\flash-espnow-pair.ps1 -FirstPort COM17 -SecondPort COM19
F:\zephyr\.venv\Scripts\python.exe .\scripts\monitor-pair.py COM17 COM19 --duration 12
```

当前固件为双向广播模式，固定使用信道 6，每两秒发送一次链路心跳。
编译脚本默认分别生成 `build-espnow-master` 和 `build-espnow-slave`。

## 小车控制链路

控制数据路径：

```text
PC GUI → COM17/UART0 → 主 ESP → ESP-NOW → 从 ESP → UART1 → MSPM0G3507
```

IMU 数据先由车载 ESP 解码，再复用现有 UART1 交给 MSPM0G3507 做闭环：

```text
WitMotion IMU → 从 ESP UART0 → 从 ESP UART1 → MSPM0G3507
MSPM0G3507 → UART1 → 从 ESP → ESP-NOW → 主 ESP → COM17 → PC GUI
```

遥测更新率为 20 Hz，包含三轴角速度、三轴姿态角、航向锁定目标/误差/修正量
以及左右轮实际输出。

- PC 与主 ESP、从 ESP 与 TI 均使用带同步头、长度、序号和 CRC16 的紧凑
  二进制串口帧，支持分包/粘包和错误后自动重同步。
- 主 ESP 从 PC 接收 STOP/FORWARD/BACKWARD/LEFT/RIGHT、循迹开关和速度。
- ESP-NOW 使用带 CRC16 的二进制控制帧。
- 从 ESP 校验控制帧，再从 GPIO5 TX / GPIO4 RX 转发给 MSPM0G3507。
- 从 ESP 使用重映射后的 UART0 GPIO2 TX / GPIO3 RX 接收 IMU，
  解析 `0x55 0x52` 角速度帧和 `0x55 0x53` 姿态角帧。
- 从机启动时把标准 WitMotion 模块配置成只输出角速度/姿态角、20 Hz、
  9600 8N1；向 3507 的 IMU 转发周期为 50 ms。
- 运动中 700 ms 没有新命令，从 ESP 自动向 TI 发送 STOP。

PC 程序见 `../../pc/README.md`，TI 串口接线与协议见
`TI_UART_PROTOCOL.md`。
