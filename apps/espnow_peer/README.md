# ESP-NOW 双向通信测试

两块 Waveshare ESP32-C6 烧录同一份 BIDIR 固件，在 Wi-Fi 信道 6 上每两秒
广播一次心跳，同时接收另一块板的心跳。无需路由器。

每个心跳包含：

- 递增序号
- 设备启动时间
- 发送方 MAC 地址

接收日志还会显示 RSSI。

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
