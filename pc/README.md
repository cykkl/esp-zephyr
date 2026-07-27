# PC 小车控制程序

## 启动

确保连接 PC 的基站 ESP 是 COM17，然后在 PowerShell 中执行：

```powershell
cd F:\dsai26\zephyresp
.\scripts\start-car-controller.ps1 -Port COM17
```

点击“连接”后，可以：

- 按住 `W` 或上方向键前进
- 按住 `S` 或下方向键后退
- 按住 `A` 或左方向键左转
- 按住 `D` 或右方向键右转
- 松开方向键自动发送 STOP
- 空格键立即停止
- 使用滑块调整速度 0–100

程序在运动期间每 200 ms 刷新一次命令。关闭窗口或断开串口前会发送 STOP；
即使 PC 异常退出，小车端 ESP 也会在默认 700 ms 后自动停车。

点击“循迹关闭 // 点击开启”可进入自动循迹。再次点击、按空格、窗口失焦或
使用方向键接管时会立即退出循迹并停车。

界面会显示 TI3507 遥测回传的真实循迹状态，而不是只显示本地按钮状态。

## PC 到基站 ESP 协议

PC 通过 COM17/CH343 以 `115200, 8N1` 发送二进制帧。帧包含 `A5 5A`
同步头、版本、类型、负载长度、16 位序号、负载和 CRC16-CCITT。控制帧共
11 字节，遥测帧共 34 字节；接收器可处理分包、粘包、错位和 CRC 错包。

命令编号为 0 STOP、1 FORWARD、2 BACKWARD、3 LEFT、4 RIGHT、
5 TRACK_ON、6 TRACK_OFF。基站以二进制 ACK 回复。详细字节布局见
`apps/espnow_peer/TI_UART_PROTOCOL.md`。

固件仍接受旧的 `CAR,<sequence>,<command>,<speed>` 文本行用于手工调试，
但新版 GUI 默认只发送二进制帧。

## 命令行测试

无需打开 GUI，发送一秒前进命令并自动停车：

```powershell
F:\zephyr\.venv\Scripts\python.exe .\pc\car_controller.py `
    --port COM17 --command forward --speed 30 --duration 1
```

端到端自动测试：

```powershell
F:\zephyr\.venv\Scripts\python.exe .\scripts\test-car-control.py
```
