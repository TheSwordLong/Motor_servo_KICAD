# GD32F303 四路电机控制板 · V2.0

本分支保存 GD32F303VET6 四路 24 V 电机控制板的 KiCad 设计，以及配套控制软件的源码快照。硬件与软件均以本次提交中的文件为准。

> **状态：开发版本，不是已完成生产验收的发布版。** 源码已收录不等于与当前 PCB 完全适配；打板、贴片及电机运行前必须完成下文检查。旧版生产压缩包、历史测试记录不能作为 V2.0 的验收依据。

## 目录与入口

| 文件或目录 | 用途 |
| --- | --- |
| `GD32F303VET6_4Motor_Control.kicad_pro` | KiCad 主工程入口 |
| `GD32F303VET6_4Motor_Control.kicad_sch` | 顶层原理图 |
| `GD32F303VET6_4Motor_Control.kicad_pcb` | 六层 PCB |
| `POWER.kicad_sch` | 电源及保护电路 |
| `GD32F303VET6.kicad_sch` | MCU、时钟、复位及存储接口 |
| `Motor_Comm.kicad_sch` | 四路电机 PWM、DIR、BRAKE、FG 接口 |
| `RS485.kicad_sch`、`TTL_CH340.kicad_sch` | RS485 与 USB 转串口 |
| `CAN.kicad_sch`、`RS232.kicad_sch` | CAN、RS232 电路，需按当前装配标记确认是否启用 |
| `kicad_api_libs/` | 项目符号、封装与部分 3D 模型 |
| `sym-lib-table`、`fp-lib-table` | 项目级库路径配置 |
| `firmware/` | MCU 固件、主机工具、ROS2 源码与说明 |

根目录的独立 `*_Module.kicad_sch` 文件包含参考模块；主工程实际使用哪些子页，以顶层原理图的层次结构为准。

## 硬件概况

- 主控：GD32F303VET6。
- 用途：四路电机接口控制与 FG 转速反馈，不是直接驱动电机绕组的三相逆变功率板。
- PCB：六层，当前文件板厚为 **1.4 mm**，板框约 **100 × 60 mm**。实际制造叠层需与板厂确认。
- 电源：24 V 输入，经板载降压得到 5 V 和逻辑电源；主逻辑网络统一命名为 **`+3V3`**。
- 铜层：`F.Cu`、`In1.Cu`、`In2.Cu`、`In3.Cu`、`In4.Cu`、`B.Cu`。
- `+3V3S`、`+5VS`、`GNDS` 是不同的备用电源域名称，不应因名称相近而与主电源直接合并；实际是否启用取决于本版装配与连接。
- 当前 RS485 原理图包含 TX 控制晶体管实现自动方向切换的方案，DI 接地、TX 高电平释放总线。必须结合偏置、终端、线长和实际波特率验证，不能按传统 GPIO-DE 接法直接验收。

### 打开 KiCad

使用 **KiCad 10.0 或能兼容当前文件格式的版本**打开根目录 `.kicad_pro`。保留项目目录结构，使库表中的 `${KIPRJMOD}` 相对路径可用。

若文件在 KiCad 窗口外更新，请先处理窗口中的未保存修改，再重新载入；不要用旧窗口覆盖磁盘上的新版本。

## 通道命名与 FG4 更新（2026-10-26）

- 第一组四路通道统一使用 `PWM1～PWM4`、`FG1～FG4`、`DIR1～DIR4`、`BRAKE1～BRAKE4`；总线对应 `PWM[1..4]` 等。
- 第二组 DNP 备用接口顺延为 5～8，例如 `PWM5～PWM8`；5 V 输出为 `PWM_5V5～PWM_5V8`，防止与第一组同名短接。
- 5 V PWM 输出使用 `PWM_5V1～PWM_5V4`；`MOTOR_DIR1～MOTOR_DIR4` 与 `MOTOR_BRAKE1～MOTOR_BRAKE4` 保留 MOTOR 前缀，以区分缓冲器两侧网络。ADC_0～ADC_3 的零起始命名保留。
- FG4 从 PA0 / 23 脚迁至 PE0 / 97 脚，符号显示 `PE0/TIMER3_ETI`；PA0 作为未连接备用。连接器和上拉电阻端点不变。
- PCB 同步网络名、相关通道文字及 MCU 两个焊盘连接，保留全部布局、走线和规则。
- **固件本次未修改。** 后续须同时适配 TIMER7 PWM，以及 PE0 / TIMER3_ETI 的 FG4 测速方式；固件中的旧通道名暂时保留。下方早期记录中的名称为历史名称。

## MCU 符号与原理图整理（2026-10-25）

- 当前使用 `GD32F303_VET6_100Pin_4Side`：U7A 为供电单元（15 脚），U7B 为时钟、启动、调试单元（9 脚），U7C 为其余 76 脚；合计 100 个脚号，各出现一次。原 `GD32F303VET6_Mulit` 符号保留。
- U7C 按 LQFP100 顶视图排列：左 1～25 向下、下 26～50 向右、右 51～75 向上、上 76～100 向左；跳过 U7A / U7B 已使用脚号。
- U7C 的引脚名称和网络标签统一为 1.27 mm；显示 GPIO 及当前功能，完整旧名称和复用选项保留为备用引脚功能。FG 的既有复用选择保留功能，仅为显示名称补上 GPIO。
- 本次整理仅修改原理图、符号库及说明；电气连接、PCB、固件与 MCU-01 / MCU-02 保持原样。图中 Wi-Fi / OTA 为停用的备用设计。

## 四路 PWM 改接 TIMER7（2026-10-25）

四路 PWM 的 MCU 端点由 TIMER3 重映射的 PD12～PD15 改接至 TIMER7 的 PC6～PC9。网络名、通道顺序、下游电平转换及电机接口保持不变。

| 网络 | MCU 引脚 | LQFP100 脚号 | 定时器通道 |
| --- | --- | --- | --- |
| PWM1_1 | PC6 | 63 | TIMER7_CH0 |
| PWM1_2 | PC7 | 64 | TIMER7_CH1 |
| PWM1_3 | PC8 | 65 | TIMER7_CH2 |
| PWM1_4 | PC9 | 66 | TIMER7_CH3 |

- 项目符号库与原理图缓存同步更新：PC6～PC9 与备用未连接的 PD12～PD15 均位于新符号的 U7C 单元；MCU-01 / U7A 与 MCU-02 / U7B 保留。
- PCB 已同步上述焊盘网络，保留原有布局和走线；四路 PWM 尚未布线。
- **固件尚未适配此映射。** 当前 `firmware/` 仍使用 TIMER3 / PD12～PD15，不能直接作为本版四路 PWM 控制固件使用。后续需修改为 TIMER7 / GPIOC PC6～PC9，取消旧 TIMER3 重映射配置，核对 APB2 定时器时钟及 PWM 周期，并配置高级定时器主输出使能和安全空闲电平。保持现有低有效 PWM 及高电平停止语义。
- 本次仅修改硬件文件与说明，未修改、编译或烧录固件。

## Wi-Fi / OTA 停用（2026-10-25）

- 当前 V2.0 不装配 Wi-Fi / OTA 电路；保留 `MCU_Core → WiFi_OTA` 原理图及原有连接作为备用设计。
- 子页及 U26、J3、SW2～SW3、R79～R82、R87～R90、C68～C70 共 15 个实体元件均设置为“不作装配”“从电路板排除”“从 BOM 排除”。
- 对应 15 个封装已从 PCB 移除，PCB 保留 156 个封装；Wi-Fi 元件不再出现在 PCB、3D 视图及贴片坐标中。其他电路、布局和固件保持原样。
- 恢复此功能时，需同时取消子页和元件的上述排除标记，再从原理图更新 PCB，重新完成布局、布线和验收。
- 下方 2026-10-23 的说明、原理图预览与检查记录为停用前的历史资料，不代表当前装配配置。

## Wi-Fi / OTA 硬件更新（2026-10-23）

- 新增 `MCU_Core → WiFi_OTA` 子页，使用 ESP32-C3-WROOM-02-N4；精简后连模块共 15 个元件。
- Type-C 原生 USB：GPIO19 接 D+，GPIO18 接 D−；R87～R90 为 0402。USB 不给板卡供电，需先接原有板卡电源；精简版未配置外置 USB ESD 保护。
- ESP GPIO4 TX → GD32 PD9 RX，ESP GPIO5 RX ← GD32 PD8 TX；GD32 固件需启用 USART2 全重映射。
- ESP GPIO10 直连 GD32 NRST，必须使用开漏：低电平复位、高阻释放，启用开漏前先将输出锁存置 1；禁止推挽输出高电平。
- OTA/IAP 目前完成硬件和方案，尚未实现升级固件；现有应用链接地址及串口配置仍需调整。
- 原理图精简时 ERC 为 0 错误、554 条原有警告，无新增项；PCB 保存为当前工作版本，尚未完成与 Wi-Fi 原理图的同步及布线验收。

[原理图预览](docs/wifi_ota/WiFi_OTA.svg) · [OTA / IAP 设计说明](docs/wifi_ota/WiFi_OTA_IAP_design.html) · [原理图检查记录](docs/wifi_ota/verification.json)

## 固件与软件

源码入口：[firmware/README.md](firmware/README.md)；详细说明：[firmware/docs/README.md](firmware/docs/README.md)。

`firmware/` 是 `GD32F303VET6_MotorControl` 工作目录于 **2026-10-22** 收录的源码快照，包含当时尚未提交的 BSP 修改。来源仓库的基准提交为 `25c6ce84a96e671c2c6e35e70acc02d851ee985c`；该提交号不能单独代表本快照全部内容。

本仓库不嵌套原固件仓库的 `.git`，不包含构建缓存、旧二进制、调试设备包压缩档、临时备份及历史验证截图/日志。原固件说明中指向 `docs/verification/` 的历史证据链接可能不可用；它们也不代表本次上传重新完成了验证。

主要内容：

- `Core/`、`BSP/`：MCU 入口、板级驱动、电机控制、FG、通信和配置保存。
- `Drivers/`、`startup/`、`linker/`：芯片支持库、启动及链接文件。
- `tools/`、`tests/`：编译辅助、主机通信及离线测试。
- `ros2/`：ROS2 控制、接口及相关工具源码。
- `App/`、`USB_App/`：保留的 USB 相关源码；是否参与构建以具体构建清单为准。

### 编译

需要 ARM GNU Toolchain、CMake（项目要求至少 3.22；使用当前 version 6 preset 需 CMake 3.25 或更高）及 Ninja。在 `firmware/` 目录运行：

```sh
cmake --preset debug
cmake --build --preset debug
```

当前快照保留来源工程配置：`CMakePresets.json` 的 Ninja 路径、`cmake/arm-none-eabi.cmake` 的 ARM 工具链路径包含本机绝对路径。换电脑需先改为本机安装路径，不能假定克隆后无需配置即可编译。

编译结果位于 `firmware/build/debug/`，包含 ELF、HEX、BIN 和 MAP。更多步骤见[编译与下载手册](firmware/docs/编译与下载操作手册.md)。本次上传不会烧录 MCU，也不会运行电机。

### 离线测试

```sh
cd firmware
python -m unittest discover -s tests -p test_robot_host.py -v
```

该测试检查主机工具逻辑，不替代实物通信、FG 精度、刹车及运动安全验证。

### 本次归档验证（2026-10-22）

- 在 `firmware/` 快照目录完成 CMake/Ninja 配置与编译，生成 ELF、HEX、BIN、MAP。
- `test_robot_host.py` 的 10 项离线单元测试全部通过。
- 编译存在 `syscalls.c` 未使用参数警告和链接器 RWX LOAD 段警告；不是零警告构建。
- 未烧录、未运行电机，未执行本版软硬件联合验收或完整 ERC/DRC 验收。

## 软硬件适配注意事项

1. **默认通信入口不能混淆。** 当前机器人代码默认通过 USART0 / CH340 使用 Modbus；保留的 RS485 驱动使用 USART1，不能仅凭源码中存在 `rs485.c` 就认为主程序已启用 RS485。
2. **RS485 方向控制尚需适配核对。** `firmware/BSP/board_config.h` 与 `rs485.c` 保留 PA1 控制 DE 的旧隔离版本逻辑；新版硬件已采用 TX 自动方向电路。启用前应核对 PA2/PA3、PA1 的实际用途、回波处理和波形，不能直接套用旧注释。
3. **电机控制极性须逐路验证。** 原理图的 MOS/电平转换可能改变 DIR、BRAKE、PWM 的电平含义；以 MCU 引脚、电机端实测和最新网表共同确认。首次测试应限流、空载或解除机械负载。
4. **不作装配不等于从 PCB 删除。** DNP 器件可能仍有封装；应确认板外备用器件、BOM、贴片坐标和实际装配一致。
5. **历史文档可能过时。** 根目录 `使用文档.md` 含旧四层板等描述。当前六层板参数以本 README 和当前 PCB 文件为准，不按旧说明直接生产。

## 打板与装配前检查

- 保存工程并重新填充铜区，执行 ERC、DRC，复查短路、未连接和铜到板边距离。
- 核对主电源、备用电源、GND/GNDS/Earth 的连接与实际装配方案。
- 清理不需要的板外封装，检查 DNP 与“排除贴片位置文件”标记。
- 依据当前 PCB 重新导出六个铜层、阻焊、丝印、`Edge.Cuts` 及 PTH/NPTH 钻孔文件。
- 重新生成 BOM 与贴片坐标，并用 Gerber 查看器/板厂 DFM 检查。
- 不直接使用仓库历史中留下的旧 Gerber、BOM、坐标或固件二进制。

本次提交的目的为源码归档与版本同步；不承诺所有 ERC/DRC 已清零或已通过整机验收。

第三方驱动和库保留各自源文件中的版权与许可声明，本 README 不改变其许可条件。
