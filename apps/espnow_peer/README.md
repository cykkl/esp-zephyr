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

当前测试固件为双向广播模式，固定使用信道 6，每两秒发送一次心跳。
编译脚本默认分别生成 `build-espnow-master` 和 `build-espnow-slave`。

## 主机与 MSPM0G3507 串口

主机额外启用 UART1 与 TI MSPM0G3507 通信：

- ESP GPIO5 / UART1 TX 接 TI UART RX
- ESP GPIO4 / UART1 RX 接 TI UART TX
- ESP GND 接 TI GND
- 默认 `115200, 8N1`

UART0/COM17 继续用于 Zephyr 日志。详细文本协议见
`TI_UART_PROTOCOL.md`，代码位于 `src/ti_uart_link.c`。
