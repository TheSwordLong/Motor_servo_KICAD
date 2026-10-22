# USB Device 参考源码说明

当前往返自检使用 Modbus/CAN，查询字段见[自检结果](../../docs/Robot_电机自检.md#roundtrip-results)；本 USB 参考代码仍未启用。

首次装机按[四路检测、配置减速比并保存到板上](../../docs/Robot_Modbus与CAN使用手册.md#commissioning)流程使用 Modbus 或 CAN。USB 参考代码不承担当前配置下发、保存或同步；驱动板 Flash 保存四路配置，上位机按 UID 备份。

本目录保留 GD32F30x USB Device 库和 CDC 参考实现，**当前机器人电机固件不启用 USB CDC**。电机通信入口是 USART0/CH340 Modbus 或 CAN，见[通信使用手册](../../docs/Robot_Modbus与CAN使用手册.md)。

## 当前构建与运行状态

| 位置 | 当前用途 |
|---|---|
| `Class/CDC` | CDC 描述符、类请求和数据回调参考源码 |
| `Core` | 枚举、控制传输、端点和电源处理源码 |
| `LL` | USBD 寄存器与中断处理源码 |
| [App/USB](../../App/USB) | 保留的板级移植封装，当前未由主循环初始化 |
| [USB_App/Reference_CDC](../../USB_App/Reference_CDC) | 独立参考主程序和中断，不能整套直接加入当前工程 |

[CMakeLists.txt](../../CMakeLists.txt) 未列入 USB 移植/库源文件；[EIDE 清单](../../.eide/eide.yml)仍列入相关编译项。文件参与编译不代表 USB 已运行。当前 [main.c](../../Core/Src/main.c) 未调用 usb_cdc_init，共享中断入口未接入 usbd_isr。

## 可从保留源码核对的配置

以下仅为未启用代码的配置事实，不是枚举或硬件验收结果：

- [usbd_conf.h](../../App/USB/Inc/usbd_conf.h)：USBD_CFG_MAX_NUM=1、USBD_ITF_MAX_NUM=2、EP_COUNT=4；CDC 数据包 64 字节、通知包 8 字节。
- 同一头文件定义 USB_DP_PULLUP_FIXED；固定上拉的板上实际情况须以硬件资料和测量核对。
- [cdc_acm_core.c](Class/CDC/Source/cdc_acm_core.c) 的默认线路编码为 1000000、8N1；不是当前电机通信波特率。
- [usb_cdc_port.c](../../App/USB/Src/usb_cdc_port.c) 保留初始化、枚举状态查询和单包发送接口，但当前固件不调用它们。

旧“USB 已集成运行”“CMake 自动收录 USB”“按键发送 USB 文本”的操作步骤已清除。若以后要启用 USB，应作为独立功能变更，重新核对引脚/上拉、48 MHz 时钟、共享中断、构建入口和电机实时性；不能照旧文档替换当前主循环。本次没有改动任何 USB 库源码或执行 USB 测试。
