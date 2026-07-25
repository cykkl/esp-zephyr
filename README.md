# Waveshare ESP32-C6 Zephyr 工程

这个目录是 Waveshare ESP32-C6-DEV-KIT 的独立应用工程。Zephyr 源码、Python
虚拟环境和 SDK 继续共用 `F:\zephyr`，本项目的源码、板级覆盖、脚本、构建结果
和硬件资料全部保存在本目录。

## 已确认的硬件

- SoC：ESP32-C6 rev 0.2，RISC-V HP Core
- 实测 SPI Flash：16 MiB
- 板载串口：CH343，当前为 `COM17`
- 原生 USB 串口：当前为 `COM18`
- UART0：GPIO16 TX、GPIO17 RX，115200 波特率
- RGB LED：单颗 WS2812B，GPIO8
- BOOT 键：GPIO9，低电平有效
- 原生 USB D-/D+：GPIO12/GPIO13

Zephyr 使用的目标为：

```text
esp32c6_devkitc/esp32c6/hpcore
```

`boards/esp32c6_devkitc_hpcore.overlay` 将上游默认的 8 MiB Flash 覆盖成实测
的 16 MiB，并声明板载 WS2812。

## 快速使用

在 PowerShell 中进入本目录：

```powershell
cd F:\dsai26\zephyresp
```

首次或需要完全清理时编译：

```powershell
.\scripts\build.ps1 -Pristine
```

增量编译：

```powershell
.\scripts\build.ps1
```

烧录到 CH343 串口：

```powershell
.\scripts\flash.ps1 -Port COM17
```

查看串口日志：

```powershell
.\scripts\monitor.ps1 -Port COM17
```

退出串口监视器使用 `Ctrl+]`。

检测设备端口：

```powershell
.\scripts\detect-ports.ps1
```

## 示例程序

`src/main.c` 会：

1. 在 UART0 打印开发板配置。
2. 让板载 RGB LED 每秒切换红、绿、蓝、白。
3. 检测 BOOT 按键，按下时输出 `BOOT button pressed`。

## 目录结构

```text
zephyresp/
├── boards/       Devicetree 板级覆盖
├── docs/         厂商引脚图和原理图
├── scripts/      编译、烧录、串口和端口检测脚本
├── src/          C 源码
├── build/        本地构建结果（由脚本生成）
├── CMakeLists.txt
├── prj.conf
└── README.md
```

## 硬件资料

- 产品页面：https://docs.waveshare.net/ESP32-C6-DEV-KIT-N8/
- 原理图：https://www.waveshare.net/w/upload/5/5a/ESP32-C6-DEV-KIT-N8.pdf

