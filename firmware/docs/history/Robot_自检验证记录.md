# 历史参考：Robot_自检验证记录

> 保留当时方案、观测及验证结果，不是现行操作步骤。历史“当前”仅指记录当天；请使用 [文档索引](../../README.md)。原报文与测量值不改写。

> 2026-10-08 故障4规则更新：按已生效减速比计算累计非预期活动量，≤10°不报警、>10°报警；516/290/82分别在第87/49/14个脉冲触发，未配置有效减速比不触发。本文原固定10/11脉冲说明属于旧版，当前规则、运行逻辑和验证见[固件功能与运行逻辑](../Robot_固件功能与运行逻辑.md)。

> 当前判据更新：残差仅记录，不再因残差大小失败；本文此前原因10与±1阈值记录均为历史版本。

# 电机响应自检验证记录

> 2026-09-27 后续调整：当前自检去程、回程各最多 2000 ms，最大低电平占空比仍为10%。下方300 ms数据属于历史版本；新验证记录见文末。

> 本页原有记录为历史版本验证，不是当前发送步骤。单帧协议版本 2 的本次验证另见[迁移与验证记录](Robot_单帧命令迁移.md)，不得照发历史 A55A 提交帧。

## 多次角度操作文档同步（2026-09-25）

操作入口：[M2 +20°、+20°、−20°、−20°，含 Modbus/CAN 帧](../Robot_免序号操作.md#repeated-angle)。本次只更新文档，未修改固件/构建配置，没有重新构建固件、烧录、连接串口/CAN 或启动电机；下方原有测试记录保持对应版本含义。连续四次角度运动的实体结果仍待用户执行并记录。

用户提供的实际串口记录：17:07:07 M2 CLEAR_FAULT 暂存收到正常 0x10 应答，17:07:14 提交收到 0x06 原帧回显，可确认该命令执行返回成功。该记录未包含后续状态/FG 累计读回、板上固件身份或角度运动测量，不能据此证明无故障复发、10/11 阈值实测通过或已经转动回位。

本次实际文档验证：

- `python build/verify_repeated_angle_docs.py`：14 份 UTF-8 文档的代码块、文件链接和章节跳转通过；新增连续运动章节的 13 条 Modbus 帧 CRC/字段、11 条 CAN 帧字段通过。[校验日志](../../build/repeated-angle-doc-check.log)。该工作区检查脚本位于 build，清理构建目录后不保证保留。
- `python tools/test_robot_auto_frames.py`：原有 22 种免序号命令变体及 16 条读取在实际协议/业务解析器的主机模拟中通过。[回归日志](../../build/repeated-angle-existing-frames.log)。这个现有检查器不模拟新增四步连续运动的实体过程，不能将其结果当作硬件回位证据。
- Firmware build / Flash / Monitor / hardware manual checks：**not actually run**，此次为文档同步，不重复固件构建或操作硬件。新增连续四步流程仍待按[完整测试流程](../Robot_完整测试流程.md)人工验收。

最新 FG 故障阈值增量：每路累计 0～10 个非预期 FG 不报警，第 11 个触发故障 4；已同步主循环和自检中断，构建与回归结果见 [FG 调试与阈值验证](../Robot_FG调试观察.md)。以下记录保持其对应历史版本含义。

后续启动保护调整：用户报告上电短转后，已把停止 GPIO 设置提前到 SystemInit 之前。最新两套构建、反汇编调用顺序与待实测事项见[上电短转排查](../Robot_上电短转排查.md)。本页以下哈希属于启动保护前移前的相应版本，不覆盖当前最新产物。

## 最新增量：序号 0000 自动分配（2026-09-25）

手动免序号报文见[操作入口](../Robot_免序号操作.md)。`robot_command` 在执行前将零转换为新的非零序号，避开缓存编号；每次零提交是新请求，非零请求保留去重/冲突规则。Flash image 136 字节、schema 1、寄存器和 CAN 应答布局不变；CAN 回显原零请求，0036 和自检 +27 提供实际编号。旧客户端必须跳过显式零序号；Python 自动重试工具已同步跳零并拒绝零请求。

### Verification

Build:

- `cmake --build --preset debug`：通过；首次因沙箱禁止 Ninja 失败，经授权在沙箱外重跑成功。[日志](../../build/autoseq-cmake.log)。
- `powershell -ExecutionPolicy Bypass -File tools/build_eide.ps1`：通过，53 个对象，未启动 GUI。[日志](../../build/autoseq-eide.log)。
- 两套 size：text=22756、data=20、bss=10588；EIDE FLASH=22776 B/508 KiB，RAM=10600 B/64 KiB。与此前往返版相比 text 增加 64 字节，Flash 保留区不变。
- 当前构建有 RWX 链接警告、EIDE 厂商 USB 未使用变量警告；CMake 本轮为增量构建，未重新编译 Newlib，不能据此宣称其历史警告已消失。
- CMake ELF SHA-256：`83378541A7F60B1F2C8FF3043F68C65CB4CFBFF71611BF91E01226813C69A8A8`。
- EIDE ELF SHA-256：`1A455292B50CCFB46530FEEA5744F90FA1EF573CC72A7BE857A50B558D481887`。

Software checks:

- `powershell -ExecutionPolicy Bypass -File tools/test_robot.ps1`：10 组 PASS。新增零请求连续执行、非零缓存重放/冲突、回绕跳零并避开旧缓存、持久化/重启、旧 Flash 零序号记录、MOVE 忙拒绝及完成后再次位移、故障/存储互锁、Modbus 重复提交与 CAN 零回显、自检内部编号及取消；原回归通过。[日志](../../build/autoseq-native.log)。
- `python tools/test_robot_port_selftest.py`：端口保护模拟通过。[日志](../../build/autoseq-port.log)。
- `python -m unittest discover -s tests -p test_robot_host.py -v`：10 个方法通过，包括 30 个自检 CLI 子场景回绕到 1，以及两种自动重试客户端和 CLI 拒绝零请求。[日志](../../build/autoseq-host.log)。
- `python tools/test_robot_auto_frames.py`：手册 22 种命令变体分别经实际 Modbus 暂存/提交和 CAN 解析器验证；16 条 RTU 读取、CRC 和字段一致性通过。[日志](../../build/autoseq-frames.log)。所有模拟动作使用内存反馈/输出，不操作电机。
- `python build/verify_protocol_guide.py`：旧非零协议示例回归通过，含 60 处命名 RTU、85 个 CAN 示例、±20°及全部操作码。[日志](../../build/autoseq-protocol.log)。
- `python build/verify_recovery_docs.py`：12 份文档 UTF-8/代码块/文件及锚点检查通过；24 条旧恢复 RTU 和 16 种非零操作的两协议回归通过。[日志](../../build/autoseq-recovery.log)。

Flash: **not actually run**。未下载或复位目标板。

Monitor: **not actually run**。未打开串口/CAN 或发送实体报文。

Manual checks: **not actually run**。免序号状态、实际运动和回位仍需板上验证。已有用户串口日志属于更新前状态，不是本版硬件验收证据。

Next verification step: 人工下载本版，先只读确认 UID 和停止/故障/存储状态。排查并清除 M2 既有非预期 FG 故障后，按免序号报文启动四路检测，核对 0036 实际编号与 +27；正式非零模式再测同号不重复。零模式重复动作测试仅在前次完成且机械空间允许的测试台进行，不用丢包重试模拟来盲目触发。

以下为免序号增量前的历史记录；体积、哈希和测试组数对应当时版本。

## 重新测试（2026-09-25）

已重新执行 C 回归 9 组、端口保护模拟、Python 9 个方法、协议与恢复报文检查，以及 CMake clean-first 和 EIDE 源清单构建，均通过。完整命令、独立日志、产物哈希和后续硬件步骤见[完整测试流程与本轮记录](../Robot_完整测试流程.md)。本轮未修改固件源码，未烧录或操作电机。

本轮 CMake 全量重编译还显示 Newlib `syscalls.c` 未使用参数警告，除既有 USB 未使用变量与 RWX 链接段警告外一并记录。实体测试仍为 **not actually run**，不能由软件通过推定机械回位合格。

## 操作及错误说明补全（文档验证）

补充 11 个功能作用/前置条件/完成判据、通信异常/业务结果/普通故障/往返原因的恢复步骤，以及完整查询和恢复 HEX。此次没有修改固件、重新构建 ARM 产物或连接硬件。

- `python build/verify_recovery_docs.py`：10 份文档 UTF-8、代码块、链接和锚点通过；24 条生成 RTU 帧 CRC/长度及解析通过；16 种恢复/功能命令分别在配置合法、已静止的模拟板上通过 Modbus 暂存/提交和 CAN 实际解析器检查。[日志](../../build/recovery-doc-validation.log)。
- 检查器初次把统一提交帧放在没有命令暂存的模拟状态中，按代码正确被拒绝；补上有效暂存前置条件后检查通过。这不是实体板报错或硬件测试。
- 原通信手册校验也已运行通过，见[协议示例日志](../../build/protocol-guide-validation.log)。故障原因的硬件排查项是待执行操作指导，不能视为已定位现场硬件原因。

## 最新版本：往返采集并返回（2026-09-25）

实现：去程 3 脉冲停止、余转累计为 N、静止确认后返程以 N 为目标，最终残差 ≤1 脉冲才通过；每程 10%/300 ms。增加 +28～+2F 只读阶段/返程时间/往返计数/残差，原因 9/10 表示返程超时/偏差超限，Flash image 仍为 136 字节。

### Verification

Build:
- `cmake --build --preset debug`：成功，首次 Ninja 被沙箱阻止后获准重跑。[日志](../../build/roundtrip-cmake.log)。
- `powershell -ExecutionPolicy Bypass -File tools/build_eide.ps1`：成功，使用 EIDE 源清单脚本，未启动 GUI。[日志](../../build/roundtrip-eide.log)。
- 两套 size：text=22692、data=20、bss=10588。已有 RWX 链接段和 EIDE 厂商 USB 未使用变量警告仍存在。
- CMake ELF SHA-256：`6EF92E652003586FD2FC71A249F0D7FD4ABDC03E7C19100A749CCCE70CB47C71`。
- EIDE ELF SHA-256：`606295CD646D643625D0F6F60F31743B1626B3D81ABC720E8530247E0E705069`。

软件模拟：
- `powershell -ExecutionPolicy Bypass -File tools/test_robot.ps1`：9 组通过。往返方向与 inverted、非零起点、去程余转加入返程目标、返程余转残差 0/±1/超限、未达目标即超时、四阶段取消、两类反馈异常、位置越界、停止锁、时基回绕，以及原运动/存储/协议回归。
- `python tools/test_robot_port_selftest.py`：通过。抽取当前端口源码中实际 ISR/输出/轮询函数，以模拟 MCU IO 运行；验证返程目标 5 不会在第 3 个脉冲截断、截止快照先于余转处理、返程独立 300 ms、其他通道及捕获错误立即停止。这不能验证真实中断延迟或电平。
- `python -m unittest discover -s tests -p test_robot_host.py -v`：9 个方法通过，覆盖新增结果及负残差解析。[日志](../../build/roundtrip-python.log)。
- `python build/verify_protocol_guide.py`：原 60 处命名 RTU 示例、四路新增往返结果读取及 85 个 CAN 示例通过校验/实际解析器模拟。[日志](../../build/protocol-guide-validation.log)。首次检查把新增八字节 RTU 帧误归为 CAN，修正检查器识别规则后通过；不是实体通信测试。

Flash / Monitor / Manual checks：**not actually run**。未烧录、复位、连接串口/CAN 或启动电机。代码已经更新不代表实体板已更新。

Next verification step：人工下载并核对 UID 后，在机械空间允许时测试四路往返、去程余转目标、返程方向、停止距离、真实回位误差及每程截止时序；覆盖缺电机/缺反馈/负载无法起动/取消/掉电。通过计数不能代替机械量测。启动报文格式不变，使用有效新序号；新增查询报文见[自检文档](../Robot_电机自检.md#roundtrip-results)。

## 以下为历史记录（单向自检及批量扫描）

以下原体积、哈希、测试行为和后续建议均对应往返功能加入前的版本，不代表当前功能。


当前推荐[四路检测 → 配置减速比 → CONFIG 保存到驱动板 → 读回 → 手动设零](../Robot_Modbus与CAN使用手册.md#commissioning)。已对照 robot.c 的 CONFIG/SAVE、位置失效和自检结果清除行为，以及 robot_host.py 的四路暂存后一次提交实现。此次仅更新文档，没有新增固件测试或构建；下文构建和回归结果属于对应固件变更。

后续人工验收（not actually run）：空白配置扫描并记录四路结果，按真实规格填写四路参数，一次 CONFIG 后核对四路已应用值与保存状态；安全停止后重启核对配置保留、UID 绑定。验证保存失败不误报成功、未使用通道禁用、配置变更清自检结果和使相应位置失效。重新设零后独立量测 +20°/−20°，上位机不得重复乘减速比。

## 最新增量：四路接入响应扫描（2026-09-25）

操作码 11 现在不要求通道启用、关节标签、减速比或零点，固定测试全部 M1→M4。无脉冲/脉冲不足在静止确认后继续下一路，安全异常或取消仍终止整批。操作码 10 的配置要求不变；协议编号、寄存器和 Flash image 136 字节不变。未配置通道不自动启用、设零或获得有效角度。

### Verification

Build:
- `cmake --build --preset debug`：成功；首次 Ninja 被沙箱阻止，获准后重跑成功。[日志](../../build/discovery-cmake.log)。
- `powershell -ExecutionPolicy Bypass -File tools/build_eide.ps1`：成功，EIDE 源清单脚本构建，未启动 GUI。[日志](../../build/discovery-eide.log)。
- 两套 size 均为 text=22032、data=20、bss=10516 字节。已有 RWX 链接段警告和 EIDE 厂商 USB 未使用变量警告仍存在。
- CMake ELF SHA-256：`BD313BA64A49461A171339E480B927EEAA185811A437CE51A7BB82399D377BB6`。
- EIDE ELF SHA-256：`7D793C02F95AD33153383E6986675325555BB0B57EA61B1F07C08036EED28A44`。

软件检查：
- `powershell -ExecutionPolicy Bypass -File tools/test_robot.ps1`：8 组通过。新增空白配置下四路全部无响应，以及 0/1/2/3 脉冲混合扫描；顺序、单通道输出、100‰/300 ms 限制、500 ms 停止确认、起止两次保存、配置/零点不变；无响应余转期取消、捕获错误、其他通道活动、停止超时；空白配置的 Modbus/CAN 批量启动及重放去重。原运动、存储断写与异常报文回归继续通过。
- `python -m unittest discover -s tests -p test_robot_host.py -v`：9 个方法通过。[日志](../../build/discovery-python.log)。
- `python build/verify_protocol_guide.py`：60 处命名 RTU 帧 CRC/长度、69 个 CAN 八字节示例、实际协议解析与正负角度生成检查通过。[日志](../../build/protocol-guide-validation.log)。

Flash / Monitor / Manual checks：**not actually run**。没有下载、复位、连接串口/CAN 或启动电机；以上是主机模拟和编译结果，不是机械或通信硬件实测。

Next verification step：人工下载新固件后，确认四路均有机械活动空间；空白配置发送一个新的 op 11 请求，测试全部未接、仅 M2 接入、混合接入和 FG 断线场景，读回四路同序号结果及板级保存标志。核验 10%/300 ms 输出、停止与余转，以及异常发生时后续通道不启动。再次扫描前处理并清除已有故障。单路及正常角度运动仍按原配置要求验收。

## 历史记录：原始自检版本（以下不是最新产物）

下文“禁用跳过、任一路失败中止、未配置拒绝”等是原版测试行为；最新批量语义和产物以上述增量为准，保留原文用于追溯当时验证。

日期：2026-09-25。对应[自检使用与协议](../Robot_电机自检.md)。本记录覆盖本次自检增量；[之前的验证记录](Robot_验证记录.md)保留原固件快照，不作为本版产物哈希。

按 [build-test-verify 技能](../../.agents/skills/embedded-codex-workflow-skills-main/build-test-verify/SKILL.md)区分构建、模拟测试和真实硬件。本次没有烧录、打开串口/CAN 或运行电机，也没有修改引脚映射、Flash 记录结构、链接脚本或构建源文件清单。

## Verification

### Build

| 项目 | 实际命令 | 结果与证据 |
|---|---|---|
| CMake 配置 | `cmake --preset debug` | 成功；初次被沙箱阻止启动 Ninja，获得执行权限后重跑成功 |
| 最终 CMake 增量构建 | `cmake --build --preset debug` | 退出码 0，重新编译修改单元并链接；[日志](../../build/selftest-cmake.log) |
| 最终 EIDE 清单构建 | `powershell -ExecutionPolicy Bypass -File tools/build_eide.ps1` | 退出码 0，编译 53 个对象并链接；[日志](../../build/selftest-eide.log) |
| 主机 C 模拟与回归 | `powershell -ExecutionPolicy Bypass -File tools/test_robot.ps1` | 退出码 0，7 组全部通过；[日志](../../build/selftest-native.log) |
| Python 上位机测试 | `python -m unittest discover -s tests -p test_robot_host.py -v` | 退出码 0，9 个测试方法通过；[日志](../../build/selftest-host.log) |
| CLI 帮助 | `python tools/robot_host.py selftest --help` | 退出码 0；[日志](../../build/selftest-help.log) |
| 文档及产物核对 | Python 静态检查 UTF-8、代码块、相对文件链接、章节跳转、HEX CRC 和 ELF SHA-256 | 通过；10 条示例帧 CRC 正确，两份 ELF 哈希匹配；[日志](../../build/selftest-doc-check.log) |
| 可选按键编译分支 | ARM GCC 编译临时副本，副本中置 `BUTTON_ESTOP_ENABLE=1`，`-Wall -Wextra -Werror` | 退出码 0；没有修改实际默认宏，也没有替换正式产物；完整命令见[日志](../../build/selftest-button-compile.log) |

ARM 工具链沿用工程已有 `15.3.rel1`；主机 C 使用已有 Zig 0.16.0，实际业务/协议/存储源码配合模拟回调运行。EIDE 结果来自读取 `.eide/eide.yml` 的脚本，未启动 EIDE GUI。

最终两套 ARM ELF 的 size 均为 `text=21992，data=20，bss=10516` 字节。EIDE 链接报告 FLASH 使用 22012 B / 508 KiB、RAM 使用 10528 B / 64 KiB。映射表中 FLASH 长度仍为 `0x7F000`，保留末尾两页存储区；`robot_selftest_safety_tick`、`robot_is_busy`、`robot_stop_all` 和 `TIMER0_Channel_IRQHandler` 已链接。

最终构建并非零警告：两套均有既有 LOAD 段 RWX 警告；EIDE 的厂商 USB 参考代码 `usbd_lld_int.c` 有未使用变量 `epcs` 警告。最后增量构建未出现本次 robot 模块编译警告。这里未声称重新审计或消除了历史工程全部警告。

### 固件 SHA-256

| 文件 | SHA-256 |
|---|---|
| `build/debug/GD32F303VET6_MotorControl.elf` | `8869B11874D8616B0C0C819556C187F8DDF8AF5C10348F782895887E4A5B4678` |
| `build-eide/Debug/GD32F303VET6_MotorControl.elf` | `823B6A900EE3C8B2D3B302C7CCCFFFF7C74C8ECF74A23A758FF68B0E54943015` |

两套构建的 ELF 调试信息/路径和构建选项可不同，因此这里分别记录哈希，没有声称 ELF 逐字节相同。后续再次构建应重新核对哈希。

### 主机测试覆盖

[C 测试](../../tests/robot_test.c)新增实际执行的场景：

- 未配置、无可测通道、非法通道/方向、未静止、其他电机运动和未清故障拒绝；无需零点即可测试。
- 已 dirty 仍写本次请求序号；最大 100‰、每级 10‰、300 ms 截止；三脉冲提前停止；零、一、二脉冲失败。
- 余转加入总数及有符号位置；截止后新脉冲不能挽救原失败；已知零点保留，未知零点不自动恢复。
- 负方向与 inverted 组合、负位置累计、int32 越界、捕获错误、其他通道活动、两秒停止超时和停止锁。
- 停止确认需同时满足静止窗口；物理停机时刻领先反馈快照一毫秒时，不误报停止超时；uint32 时间回绕。
- MOVE、方向、释放刹车、设零、配置提交、SAVE、清故障、新自检在批次中被拒绝；任一路 STOP、STOP_ALL、BRAKE=1 和统一按键取消入口中止整批。
- 模拟按键持续有效时拒绝新运动/自检/释放刹车，同时允许读取和停止。
- dirty 写失败不释放刹车且保留原位置有效性；完成保存失败维持存储锁；显式 SAVE 恢复；整批只有起止两次保存，通道切换无 Flash 操作。
- 批量固定顺序、禁用跳过、失败后续未执行；停止超时后即使配置变化清结果也不能绕过运动锁，静止清故障后方可解锁。
- 自检结果 RAM 重启清空，dirty 重启四路位置失效，旧序号拒绝；结果只读、旧状态寄存器兼容、改 RPM 保留结果、改 inverted 清对应结果。
- Modbus 单路启动和结果读取、CAN 同号重放去重、两协议结果一致；CAN 批量和停止；Flash image 大小仍为 136 字节。

原有五组回归继续执行，包括四路独立运动、正常运动故障与位置规则、Flash 日志 **129 个断写注入位置**、**20000 个异常 Modbus ADU**、RTU 分帧/溢出/回绕。

[Python 测试](../../tests/test_robot_host.py)包含原有五项，加上自检无符号字段解析、UID 必填、status 四路结果、Modbus/CAN × 五种通道选择 × 三种方向参数形式的 **30 个 CLI 子场景**。使用模拟客户端，未创建真实串口或 CAN 连接。

实现期间首轮新增协议测试因测试请求的 CAN 保留字节未清零而失败，修正测试帧后通过；该失败没有算作成功。随后补入停机时刻跨 tick 用例，最终全部回归通过。

### Flash

- Command：无。
- Result：**not actually run**；未烧录、擦除或写入实体板 Flash。
- Evidence：执行范围为本地编译、模拟和静态检查。构建脚本末尾仅打印烧录命令，没有执行该命令。

### Monitor

- Command：无。
- Result：**not actually run**；未打开 CH340 串口、CAN 或串口监视器。
- Evidence：上位机实际执行仅为帮助和模拟客户端单元测试。

### Manual checks / Not actually run

GPIO 电平、PWM 占空比与通电时长、SysTick 截止保护的实际中断延迟、FG 捕获电气行为、余转机械位移、真实连接/断线/掉电、三种减速比和负载、掉电写入及按键电气路径均为 **not actually run**。

核心状态机有模拟覆盖；MCU 中断保护绑定仅完成源码检查和目标架构编译，未用主机测试假装验证外设时序。原有电机转向、刹车保持能力和 ±5° 机械精度仍须独立测量。

Next verification step：按[自检硬件验收表](../Robot_电机自检.md#7-软件验证与硬件验收)，先测电机端输出电平，再在有机械活动空间的条件下人工触发单路自检，记录 UID、固件哈希、FG、PWM/BRAKE 波形和位移。


## 2026-09-27：每程通电上限调整为 2000 ms

本次修改 `BSP/robot.h` 的 `ROBOT_SELFTEST_POWER_MS`，从 300U 改为 2000U。主循环和 1 ms 中断保护共用该常量，去程与回程分别计时；最大低电平占空比仍为 10%，满足脉冲目标仍提前停止。

实际执行结果：

- `powershell -NoProfile -ExecutionPolicy Bypass -File tools/test_robot.ps1`：12 组回归通过，覆盖延长后的截止时间、往返、批量检查及时间回绕。
- `python tools/test_robot_port_selftest.py`：通过；验证去程超过旧 300 ms 时限仍保持使能，2000 ms 截止，回程截止及脉冲提前停止。此项是主机模拟，不代表硬件时序实测。
- `python tools/test_robot_docs.py`：12 份文档、266 个本地链接与章节跳转、255 条 RTU 示例检查通过。
- `python tools/test_robot_auto_frames.py`：22 条命令分别经 Modbus/CAN 模拟验证、16 条读取验证通过。
- `python -m unittest discover -s tests -p test_robot_host.py`：10 项测试通过。
- `cmake --build --preset debug`：通过，text=23040、data=20、bss=10788 字节；仍有已有的 LOAD 段 RWX 链接警告。日志：`build/selftest-2s-cmake.log`。
- `powershell -NoProfile -ExecutionPolicy Bypass -File tools/build_eide.ps1`：通过，FLASH=23060、RAM=10800 字节。日志：`build/selftest-2s-eide.log`。

本次未烧录、未发送运动命令，未进行串口、CAN 或电机硬件测试。延长通电不能证明 M4 反馈问题已解决；更新固件后仍需核对 PWM/BRAKE 输出、FG 捕获、去程与回程计数及实际位移。


<a id="m4-return-error-20260927"></a>
## 2026-09-27 20:29：M4 回位残差超限（用户串口实测）

来源：用户提供电脑串口上位机收发记录，非 SSH 测试、非本助手实际操作。接线按用户说明为 M4 左大臂、减速比 516；本次日志没有 UID、固件哈希及配置回读，不能单凭这些短时返回确认烧录的具体版本或实际机械方向。

```text
20:29:22.057 启动 M4 单路自检
TX: 01 10 00 30 00 05 0A 00 00 00 0A 00 03 00 00 00 00 DF C8
RX: 01 10 00 30 00 05 00 05
20:29:52.385 自检结果
TX: 01 03 01 E0 00 08 44 06
RX: 01 03 10 00 05 00 0A 00 00 00 0D 00 06 00 04 B6 5B 00 33 E1 8E
20:29:58.014 往返详情
TX: 01 03 01 E8 00 08 C5 C4
RX: 01 03 10 00 05 00 0A 00 00 00 05 00 00 00 08 FF FF FF FD 67 94
```

| 字段 | 原始值 | 解码 |
|---|---|---|
| 状态 / 原因 | 0005 / 000A | 失败 / ST_RETURN_ERROR，回位残差超限 |
| 总 FG | 0000000D | 13 个，含余转 |
| 去程记录时间 | 0006 | 6 ms |
| 完成时运行时间 | 0004B65B | 308827 ms，非电脑时间戳 |
| 请求序号 | 0033 | 51 |
| 阶段 | 0005 | 已结束，不代表通过 |
| 回程记录时间 | 000A | 10 ms |
| 去程 / 回程 FG | 00000005 / 00000008 | 5 / 8 个，含各程余转 |
| 有符号残差 | FFFFFFFD | -3 个脉冲 |

代码依据：[枚举和容差](../../BSP/robot.h)中的 `ST_RETURN_ERROR=10`、`ROBOT_SELFTEST_RETURN_TOLERANCE=1`；[状态机](../../BSP/robot.c)累加每程及余转脉冲，按命令方向更新位置，停稳后检查残差是否超出 ±1。正向去程下 5-8=-3，故本次报告失败；不是无反馈、不是返程超时，也不是固定转满 2 秒。命令正常应答只表示接受，固件已进入回程阶段，但不能仅凭单线 FG 证明机械实际反转回位。

按每电机转 6 个 FG、减速比 516 计算，-3 个脉冲约为输出轴 -0.349°；这是代码计数换算，不是实测角度。自检使用 ±1 脉冲判据，与正常角度运动的 ±5°要求不同，不应据此改判通过。

后续操作：

1. 读取 M4 状态 `01 03 01 D0 00 07 04 0D`，以及保存状态 `01 03 00 05 00 03 15 CA`。源码失败路径使该路位置无效并置自检故障；本次未提供这两条回读，不能声称已核实板上最终保存状态。
2. 记录实际正反转、PWM/BRAKE/DIR 与 FG 波形。去程 6 ms 即截止，早于从零每 10 ms 增加占空比的首个正常步进，应重点核对 FG 是否由刹车释放后的被动运动、信号干扰、映射或时序问题造成；这些均为待验证可能性，不是已确认原因。
3. 区分达到目标时的计数与停止后的余转，不能将最终多出的 3 个直接全部认定为刹车余转。核对反馈可信后，再决定是否需要调整制动/控制逻辑；本次不放宽容差、不自动补转。
4. 再次自检前，停止并静止后清除 M4 故障：`01 10 00 30 00 05 0A 00 00 00 09 00 03 00 00 00 00 EC C8`。清故障不恢复位置有效，也不证明问题解决；不得用设零掩盖反馈问题后直接进行角度运动。

本次文档更新仅做源码对照和报文解码，没有重新构建、烧录或主动操作硬件。


## 2026-09-27：取消残差超限失败的软件验证

变更：删除停止确认后的残差阈值失败分支；保留有符号残差寄存器、实际位置累计、原位置有效性规则以及所有其他保护。原因10保留为历史编号。每程上限仍为2000ms、最大低电平占空比10%。没有自动补转或强制归零。

实际执行：

- `powershell -NoProfile -ExecutionPolicy Bypass -File tools/test_robot.ps1`：12组通过。正负方向和方向反相组合下，残差0到±5均记录并通过，批量进入下一路，原有效位置保留且完成保存；寄存器保留int32补码。返回超时、捕获错误、位置越界、取消和停止锁等原回归仍通过。日志 `build/residual-tests.log`。
- `python tools/test_robot_port_selftest.py`：通过，实际ISR提取模型中残差-4仍通过；脉冲截止、2000ms截止及其他活动保护仍通过。这是模拟，不是硬件实测。
- `python tools/test_robot_auto_frames.py`：22条命令分别经Modbus/CAN解析与16条RTU读取通过。
- `python -m unittest discover -s tests -p test_robot_host.py`：10项通过。
- `python tools/test_robot_docs.py`：UTF-8、链接、章节、代码块和RTU示例通过。
- `cmake --build --preset debug` 与 `powershell -NoProfile -ExecutionPolicy Bypass -File tools/build_eide.ps1`：均通过。text=22976、data=20、bss=10788字节；EIDE FLASH=22996、RAM=10800字节。已有LOAD段RWX警告仍存在。日志 `build/residual-cmake.log`、`build/residual-eide.log`。

Flash / Monitor / Manual checks：本次新逻辑未烧录、未执行串口或CAN硬件试转（not actually run）。此前远程旧固件自检M2为3/3、M3为3/3且通过，M4为6/10、残差-4而失败；该历史结果不能充当新逻辑验证。

下一步：用户烧录后重测单路及批量，核对非零残差仍显示且不因残差失败、脉冲到数仍停止、其他故障继续报警。协议版本仍为2，须用构建产物身份确认版本，不能只读000F区分新旧判据。


## 2026-09-27：远程板自检及运动实测补充

以下为软件验证之后实际通过远程主机串口执行的结果；不覆盖前文当时“未烧录/未实测”的历史事实。板子UID为 C23B378115B092083830364A，协议版本2，M2左大臂516、M3左小臂290、M4左手腕82（标签6）。用户已确认停止电平和机械活动空间。没有读取固件哈希，因此不能仅凭这些结果证明板上二进制与本地ELF完全一致。

| 通道 | 自检序号 | 去程/回程FG | 残差 | 去程/回程时间 | 结果 |
|---|---|---|---|---|---|
| M2 | 81 | 3/3 | 0 | 104/98ms | 通过 |
| M3 | 82 | 3/3 | 0 | 99/117ms | 通过 |
| M4 | 83 | 6/10 | -4 | 8/12ms | 通过 |

M4非零残差保留且未导致失败，符合新判据。各路最终PWM为0、刹车启用、转速反馈0、无故障、保存三标志全0。此实测没有触发2000ms截止，不能据此声称时限已做硬件验证。

用户确认后，三路当前姿态分别设零并保存，各执行一次+20°：M2反馈20.0°，M3反馈20.0°，M4反馈23.4°（脉冲量化目标19.7°）。均完成且保存正常。随后M2返回0.0°，M4返回-3.6°，M3在其当时姿态重新设零。角度均来自FG换算，不是外部机械测量。

M2另完成+40°至40.1°、再+20°至60.2°；后一次完成后短暂有低速FG反馈，监测程序补发STOP_ALL，复查静止、无故障及保存正常，原因未独立确认。

M2使用临时600RPM（减速前）完成约10秒往返，每次完成均恢复并保存8000RPM：

| 动作 | 首次观察到刹车停止 | 报告完成 | 最终反馈 |
|---|---|---|---|
| 60.2°返回0° | 9.95s | 10.38s | 0.0° |
| 0°到60° | 9.98s | 10.84s | 60.2° |
| 60.2°返回0° | 9.74s | 10.39s | 0.0° |

时间由主机轮询观察，约0.2秒采样间隔，非固件定时轨迹或精准机械到位计时。最终M2反馈0°、有效、无故障、PWM0、刹车启用、配置8000RPM、保存标志全0。全过程没有自动循环或追加位置补偿。


<a id="local-ports-20260928"></a>
## 2026-09-28：单电机在 M1、M2、M4 的本地测试

来源：用户提供的 00:19～00:36 本地上位机 Modbus 收发日志；本次文档整理没有连接串口、SSH、烧录或启动电机。00:19 UID 回读为 `C23B37C715B092083830364A`，协议版本 2。后续记录按用户连续测试归于此板，未在每次换口后重新读取 UID，也没有固件哈希。用户说明只有一台减速比 516 的电机，依次测试 M1、M2、M4，避开有硬件疑问的 M3；每次配置为左大臂（标签 2）。

| 通道及时间 | 自检结果 | 去程/回程 FG | 残差 | 去程/回程时间 | 后续证据 |
|---|---|---|---|---|---|
| M1 00:23～00:24 | 失败，原因 10，序号 4 | 5/8 | -3 | 6/10 ms | ZERO 后空闲、无故障、位置有效、角度与计数归零、保存三标志全 0；用户随后反馈“M1可以”，未提供运动后角度回包 |
| M2 00:29 | 失败，原因 10，序号 16 | 5/8 | -3 | 7/10 ms | 状态 6、故障 7、位置无效 |
| M2 00:33 | 通过，原因 0，序号 20 | 5/8 | -3 | 6/10 ms | ZERO 后空闲、无故障、位置有效、角度/计数/目标为 0、保存三标志全 0；未提供本轮 +20° 运动结果 |
| M4 00:36 | 通过，原因 0，序号 26 | 5/8 | -3 | 7/10 ms | ZERO 后有效；+20° 接受，随后状态 5、故障 0、位置有效、刹车 1、PWM/转速 0，保存三标志全 0；缺运动后角度回读 |

关键原始回包：

```text
M2 00:33:34 自检结果（0160，8 个寄存器）
01 03 10 00 04 00 00 00 00 00 0D 00 06 00 01 6B 06 00 14 1F 20
M2 00:33:39 往返详情（0168，8 个寄存器）
01 03 10 00 05 00 0A 00 00 00 05 00 00 00 08 FF FF FF FD 67 94
M4 00:36:21 自检结果（01E0，8 个寄存器）
01 03 10 00 04 00 00 00 00 00 0D 00 07 00 04 0C F1 00 1A EC A2
M4 00:36:27 往返详情（01E8，8 个寄存器）
01 03 10 00 05 00 0A 00 00 00 05 00 00 00 08 FF FF FF FD 67 94
M4 00:36:46 运动后状态（01D0，7 个寄存器）
01 03 0E 00 05 00 00 00 01 00 01 00 00 00 00 00 00 FE 40
M4 00:36:50 保存状态（0005，3 个寄存器）
01 03 06 00 00 00 00 00 00 21 75
```

解码注意：自检结果第二个寄存器是原因，往返详情第二个寄存器是返程毫秒；详情中的 `000A` 表示 10 ms，不是原因 10。阶段 5 表示结束，只有结果状态 4、原因 0 才表示通过。-3 个脉冲按 516×6 换算约为 -0.349°，不是外部测角值。

当前源码没有残差阈值失败分支，`ST_RETURN_ERROR=10` 仅保留历史编号。前两次原因 10 表明运行行为与当前源码不一致，可能来自旧构建/烧录产物；不能仅凭日志断言具体二进制版本。后两次行为符合新策略，也不能代替固件哈希核验。000F=2 只区分单帧命令协议，不能区分残差策略版本。

待验证：M4 读取 `01 03 01 D8 00 06 44 0F` 确认当前角度及目标；M1/M2 补充运动后状态与角度；实际机械方向、角度精度、各接口 PWM/BRAKE/FG 波形、2 秒硬件截止、断电恢复仍需独立验证。M3 本轮未测，不能宣称其电气问题已修复。6～7 ms 即出现足够 FG 仍需波形解释，不能仅凭“通过”认定全部反馈来自预期主动试转。

本次文档更新的 Build / Flash / Monitor：not actually run（仅整理已有日志，未改固件，不需要重新构建）。实际执行 `python tools/test_robot_docs.py`：12 份 UTF-8 文档、274 个本地链接/章节跳转、271 条完整 RTU 报文校验通过，代码围栏闭合。


## 2026-09-29：相对 MOVE 与绝对零点有效性解耦

变更：只移除 MOVE 准入条件中的 valid 检查；目标仍为当前保留计数加相对增量，零点无效不会阻止相对运动，成功也不将 valid 恢复为 1。计数、协议命令/寄存器/Flash 格式不变；故障需显式清除，ZERO 仅用于建立新零点。无绝对定位命令、无自动清故障、无自动重试或追补运动。

### Verification

Build：实际执行 `cmake --build --preset debug` 及 `powershell -NoProfile -ExecutionPolicy Bypass -File tools/build_eide.ps1`，均通过。CMake/EIDE text=22956、data=20、bss=10788；EIDE FLASH=22976、RAM=10800 字节。仍有已有 USB 未使用变量与 LOAD RWX 警告。

实际离线验证：

- `powershell -NoProfile -ExecutionPolicy Bypass -File tools/test_robot.ps1`：13 组通过。新增无 ZERO 的非零计数正向运动、反向/反相、正常与 dirty 重启、有效位保持、故障拒绝/显式清除、存储失败、溢出、反馈/捕获/到位偏差和其他互锁，以及 Modbus/CAN 与重放检查。
- `python tools/test_robot_port_selftest.py`：实际端口保护函数的主机模拟通过。
- `python tools/test_robot_auto_frames.py`：每协议 22 种命令变体、16 条 RTU 查询通过。
- `python -m unittest discover -s tests -p test_robot_host.py`：10 项通过。
- `python tools/test_robot_docs.py`：编码、链接、章节、完整 RTU CRC 和代码围栏检查通过。

Flash / Monitor / Manual checks：not actually run；本次未烧录、未使用新版连接串口/CAN 试转，也未清除远程板故障。不能把旧固件已执行的运动当作本变更的硬件验证。

Next verification step：烧录本次构建产物，在安全测试台按[无零点相对运动](../Robot_Modbus与CAN使用手册.md#unreferenced-relative)验证 configured/valid=0 下的显式相对命令、故障恢复与重启，不执行 ZERO。寄存器协议版本 2 未变，不能用于识别此次变更。


<a id="relative-hardware-20260929"></a>
## 2026-09-29：无零点相对运动的远程串口验证及 DIR 未解决问题

这是上述软件构建之后的补充证据，不改写当时尚未烧录的历史记录。用户报告已烧录，助手经 xinhua@192.168.1.113 实际执行 Modbus；串口重新枚举为 ttyUSB2，稳定路径仍为 platform-fc800000.usb-usb-0:1.2:1.0-port0，板 UID C23B378115B092083830364A。没有读取板上固件哈希，因此仅确认行为与新逻辑一致，不证明二进制身份。

先清除 M4 故障4（序号118），约5秒内未复发；valid仍为0，未设零、未试转。随后实际测试头部 M1，减速比516、inverted=0：

| 动作 | 命令序号 | 位置有效位 | 计数变化 | 观察到刹车/完成时间 | 结果 |
|---|---|---|---|---|---|
| 临时400RPM、相对+10° | CONFIG119、MOVE120、恢复CONFIG121 | 始终0 | -96→-10，+86 FG | 2.14/2.77秒 | DONE，无故障，刹车1，PWM/转速0，保存三标志0 |
| 临时400RPM、相对-10° | CONFIG122、MOVE123、恢复CONFIG124 | 始终0 | -10→-96，-86 FG | 2.34/2.76秒 | DONE，无故障，刹车1，PWM/转速0，保存三标志0 |

两次没有 ZERO，停止后恢复8000RPM配置，状态回到0但fault=0、valid=0。证明新语义可在零点无效时接受并完成相对计数运动，且不自动恢复零点。时间是主机轮询观察值；FG符号来自命令，不代表独立机械测角。

用户确认+10°确实向后约10°，但明确否认-10°向前，故实际正反转问题未解决。软件运行时DIR分别为0/1，不能据此宣称电机实际换向正确。随后M1再次出现故障4；用户授权后清故障125、释放刹车126，保持PWM0，短时读回无故障。再经用户授权STOP127、ZERO128，三次读回角度/计数/目标0、valid1、刹车1及保存正常。这次ZERO是用户明确重新定义参考，不是相对运动的前提。

为静态测电压，助手发送DIRECTION=0（序号129），读回DIR0、刹车1、PWM/转速0。用户报告板上PE15对板GND为0V、电机方向线对电机GND为5V，并称线路与地连通。尚缺同时测量的两端压差、带载接触电阻、原理图及芯片GPIO寄存器实读，不能确定是断路、参考地、电平转换或GPIO损坏；没有修改推挽模式/引脚映射，也没有确认电机端低电平成功。

此后SSH会话断开，再次连接未获批准，重新发送DIR的尝试没有执行成功。最后成功核对的软件状态是DIR0、刹车1、PWM0；不能作为断开后实时状态。头部实际方向、非预期FG来源、两端电压差、M2异常余转/反馈仍待验证。本轮没有验证CAN实物或新版掉电恢复。

<a id="remote-readonly-20260929"></a>
## 2026-09-29（晚）：远程只读复核与 ROS 2 驱动离线验证（未烧录、未运动）

本节全部是**只读**证据：经 `ssh xinhua@192.168.1.113` 用 `id_ed25519` 私钥（口令保护）登录，向板卡只发送功能码 `03` 读请求，**没有发送任何写命令、没有运动、没有清故障、没有设零、没有自检、没有烧录**。没有读取板上固件哈希，因此不能据此声称板上二进制与任何本地构建一致。

### 主机与串口环境（实测）

| 项 | 实测值 |
|---|---|
| 主机 | `ubuntu20`，Ubuntu 20.04.6，Linux 5.10.209，**aarch64**，内存 15 GiB，load average ≈10，10 个登录会话 |
| 用户 | `xinhua`，组含 `dialout` 与 `sudo` |
| 磁盘 | `/`（含 `/home`）已用 **97%**，仅剩约 478 MB；`/data` 剩 19 GB |
| 本板串口 | `platform-fc800000.usb-usb-0:1.2:1.0-port0` → **ttyUSB0**，1a86:7523 HL-340，驱动为自编译 `/lib/modules/5.10.209/extra/ch341.ko`，探测时**空闲无占用** |
| 另一串口 | ttyUSB1 = 10c4:ea60 CP2102 = `/dev/door_lock_uart`，被 `python3 -m gateway.kafka_server`（PID 256299）占用，**与本板无关，全程未触碰** |
| 在跑服务 | pm2 `robot-gateway`、`robot-h5`；ROS 图上有 `/controller_manager`、`/joint_state_broadcaster`、`/diff_drive_controller`、多个 `/robot_state_publisher` |
| 工具链 | `python3` + pyserial 3.5、pyyaml 5.3.1、cmake 3.25.2、gcc 9.4.0、git；**无 arm-none-eabi-gcc / openocd / probe-rs / gdb / python-can** |

同一稳定路径在 2026-09-29 白天记录为 ttyUSB2、上一节记录为 ttyUSB1，本次为 **ttyUSB0**：编号会继续漂移，脚本必须用 `/dev/serial/by-path/`。

### 只读回包（115200 8N1，站号 1，全部 CRC 校验通过）

| 项目 | 请求 | 应答 | 解码 |
|---|---|---|---|
| 板级配置 0001～0004 | `01 03 00 01 00 04 15 C9` | `01 03 08 00 00 00 01 00 01 61 A8 D1 F9` | protocol=0(Modbus)、address=1、node=1、pwm=25000 |
| 保存标志 0005～0007 | `01 03 00 05 00 03 15 CA` | `01 03 06 00 00 00 00 00 00 21 75` | dirty=0、storage_error=0、pending_save=0 |
| UID 0008～000D | `01 03 00 08 00 06 44 0A` | `01 03 0C C2 3B 37 81 15 B0 92 08 38 30 36 4A 78 12` | `C23B378115B092083830364A`，与历史记录同一块板 |
| 命令协议版本 000F | `01 03 00 0F 00 01 B4 09` | `01 03 02 00 02 39 85` | 版本 2（单帧命令语义可用） |
| 最近序号/结果 0036～0037 | `01 03 00 36 00 02 24 05` | `01 03 04 00 90 00 00 FA 1E` | 序号 144、业务结果 0（成功） |
| M1 配置 | `01 03 01 00 00 05 84 35` | `01 03 0A 00 01 00 02 02 04 00 00 1F 40 F3 04` | enabled=1、role=2、gear=516、inverted=0、rpm=8000 |
| M2 配置 | `01 03 01 40 00 05 85 E1` | `01 03 0A 00 01 00 01 02 04 00 00 1F 40 C0 04` | enabled=1、role=1、gear=516、inverted=0、rpm=8000 |
| M3 配置 | `01 03 01 80 00 05 85 DD` | `01 03 0A 00 01 00 04 01 22 00 00 1F 40 9C F0` | enabled=1、role=4、gear=290、inverted=0、rpm=8000 |
| M4 配置 | `01 03 01 C0 00 05 84 09` | `01 03 0A 00 01 00 06 00 52 00 00 1F 40 FF 2A` | enabled=1、role=6、gear=82、inverted=0、rpm=8000 |
| M1 状态 | `01 03 01 10 00 07 04 31` | `01 03 0E 00 00 00 00 00 00 00 01 00 00 00 00 00 00 FF D5` | state=0、fault=0、valid=0、brake=1、dir=0、duty=0、rpm=0 |
| M2 状态 | `01 03 01 50 00 07 05 E5` | `01 03 0E 00 00 00 00 00 00 00 01 00 01 00 00 00 00 C2 15` | state=0、fault=0、valid=0、brake=1、**dir=1**、duty=0、rpm=0 |
| M3 状态 | `01 03 01 90 00 07 05 D9` | `01 03 0E 00 01 00 00 00 01 00 01 00 00 00 00 00 00 F0 C4` | **state=1(空闲)**、fault=0、**valid=1**、brake=1、dir=0 |
| M4 状态 | `01 03 01 D0 00 07 04 0D` | `01 03 0E 00 00 00 00 00 00 00 01 00 00 00 00 00 00 FF D5` | state=0、fault=0、valid=0、brake=1、dir=0 |
| M2 角度 | `01 03 01 58 00 06 45 E7` | `01 03 0C 00 00 04 37 00 00 03 A0 00 00 04 37 63 A8` | 角度 107.9°、FG 计数 928、目标 107.9° |
| M4 角度 | `01 03 01 D8 00 06 44 0F` | `01 03 0C FF FF FB F1 FF FF FF 72 FF FF FB F1 64 5F` | 角度 −103.9°、FG 计数 −142、目标 −103.9° |

M1/M3 角度块与四路自检块（`+0x20`、`+0x28`）本次全部读回 0，说明本次上电后未做过自检。M4 角度回包与 `pulses*3600/(gear*6)` 公式一致：`-142*3600/(82*6) = -1039`（向零截断）。

### 只读结论与边界

- 四路 `fault` 均为 0：上一节记录的“插拔后故障4”本次**读不到**，但这只说明当前寄存器值，不说明根因已消除，也不能当作非预期 FG 问题已解决。
- 只有 M3 有有效零点（valid=1、state=1 空闲）；M1/M2/M4 为 state=0（未建立零点）且 valid=0，与“相对运动不依赖 valid、绝对运动依赖 valid”的语义一致。
- M2 在 valid=0 下 `dir=1`，而 M1/M3/M4 为 0；这与 M1 方向未解决问题同族，仍缺 PE15 与电机端方向线的实测电压，不能据此判断接线或极性。
- 减速比与角色（516/516/290/82，role 2/1/4/6）与 [ROS 2 执行方案](ROS2_头部与双臂控制_代码生成执行方案.md) 第 2 节的关节映射逐项吻合，因此该映射按“与在板配置一致”对待；机械正方向与限位仍未确认。
- 保存三标志全 0、最近序号 144 且结果 0：板子处于干净状态，没有未落盘的pending。

### ROS 2 驱动的生成与验证

按 [ROS 2 执行方案](ROS2_头部与双臂控制_代码生成执行方案.md) 生成三个新包，源码在本仓库 `ros2/`，构建与测试在上述远程主机执行；审查结论见 [compatibility_audit.md](compatibility_audit.md)，协议出处见 [protocol_contract.md](protocol_contract.md)，用法与回滚见 [ros2/README.md](../../ros2/README.md)。

- 工作区实体在 `/data/ros2/robot_motor_ros2_ws`（因 `/` 只剩数百 MB），`~/robot_motor_ros2_ws` 是指向它的符号链接；旧工作区 `/opt/ros/motor_control_ws`（与 `/data/ros2/motor_control_ws` 同一 inode）**未被修改**，其 git 仓库 `src/modbus-motor-ctl` 仍为 `master` / HEAD `b7881bd` / `git status --short` 为空。
- 因该机 Humble 是 colcon 源码安装且缺 `ament_python`，三个包改用 `ament_cmake` + `ament_cmake_python`（测试用已存在的 `ament_cmake_pytest`）；`diagnostic_msgs` 缺失，诊断话题运行时降级为 `/robot/diagnostics_json`。
- 先用一个一次性最小包实测该机能否生成 action（`/data/ros2/dsh_iface_probe`，`COLCON_BUILD_EXIT=0`、`ros2 interface show` 正常），确认后才按方案用 Action 而没有退化成 Service。
- `colcon build --symlink-install`：3 个包全部成功。
- `colcon test --parallel-workers 1`：**356 项测试，0 失败 0 错误 0 跳过**（protocol 73、config 88、node 43、fake_board 37、board_session 36、scheduler 22、CAN 编解码 20、transport 19、launch 9）。
- `ros2 interface package robot_motor_interfaces` 列出 10 个接口；`ros2 pkg executables robot_motor_driver` 输出 `robot_motor_node`。
- 仓库既有测试未受影响：`tools/test_robot_docs.py` PASS（13 docs / 297 links / 313 RTU examples）、`tests/test_robot_host.py` 10 tests OK、`tools/test_robot_auto_frames.py` PASS、`tools/test_robot_port_selftest.py` PASS。
- 离线测试全部运行在**模拟板卡**上：`transport=fake` 不打开任何串口，节点测试强制 `ROS_DOMAIN_ID=71` 以免进入该机 domain 0 上在跑的实机控制栈。

### 阶段 D：模拟后端的动作验收（`transport=fake`，不开串口）

`ros2/robot_motor_bringup/scripts/stage_d_fake_acceptance.sh` 的实际输出（证据留在远程 `/data/ros2/stage_d_evidence/`）：

| 场景 | 实际结果 |
|---|---|
| 默认 bring-up（`allow_motion:=false`） | 话题 `/robot/motor_states`、`/robot/diagnostics_json`；服务 7 个（stop / clear_fault / set_zero / set_brake / apply_config / acknowledge_unknown / board_identity）；动作 2 个（move_joint / selftest）。节点日志 `TX ` 计数为 0 |
| 只读模式下的 MOVE 目标 | `Goal was rejected`，日志 `move goal rejected: allow_motion is false` |
| 只读模式下的 `/robot/stop` | `accepted=False, stop_confirmed=False`，消息点名"Use the hardware e-stop, or restart with maintenance:=true" |
| 相对 +10°（`left_forearm_joint`，gear 290，`allow_unreferenced_relative:=true`） | `SUCCEEDED`：`motion_completed=true, persisted=true, position_valid=false, error_code=0`，`final_angle_rad=0.17278759594743864`（目标 0.17453292519943295）。差值来自 FG 量化：48 步 → `48*3600/(290*6)=99` 个 0.1°，即 9.9°，不是 10.0°。反馈流带 `command_sequence=1`、`state=5` |
| 绝对运动且 `position_valid=false` | `ABORTED`，`error_code=104`，消息说明不会自动设零 |
| `/robot/set_zero` | `accepted=True, sequence=1, result_code=0, position_valid=True, persisted=True` |
| 设零后的绝对运动 0.2 rad | `SUCCEEDED`：`motion_completed=true, persisted=true, position_valid=true, final_angle_rad=0.2007128639793479` |
| 量化为 0 步的目标（gear 82，0.0001 rad） | `SUCCEEDED`，`error_code=106`（NO_OP），消息明确"nothing was sent and no new DONE is expected" |
| 运动中调用 `/robot/stop` | `accepted=True, stop_confirmed=True, unknown=False, sequence=3`，消息"stop confirmed: channel settled with duty 0" |
| `/robot/selftest`（单路，正向） | `SUCCEEDED`：`completed=true, passed=true, selftest_state=4, reason=0, phase=5, outbound=5, return=5, residual=0`，消息附带"passing only proves at least 3 feedback pulses were captured, not a mechanical connection or a direction" |
| 诊断 | `{"level": "OK", "uid": "C23B378115B092083830364A", "read_only": false, "unknown": false, "problems": []}` |

### 阶段 C 末步：只读连实板（真实串口，仅功能码 03）

`ros2/robot_motor_bringup/scripts/stage_c_realboard_readonly.sh`，`transport:=modbus_rtu allow_motion:=false maintenance:=false`，端口用稳定路径，脚本前置检查端口存在且未被占用，全程不触碰 ttyUSB1（门锁网关）。实际结果：

- 节点日志：`board identified uid=C23B378115B092083830364A protocol=2 schema=1 dirty=False`、`ready: transport=modbus_rtu port=/dev/serial/by-path/platform-fc800000.usb-usb-0:1.2:1.0-port0 allow_motion=False maintenance=False read_only=True polling=5.0 Hz`。
- `/robot/board_identity` → `success=True, uid=C23B378115B092083830364A command_protocol=2 expected=C23B378115B092083830364A error=none`。
- `/robot/motor_states` 发布真实通道数据（如 channel 1：role=1、gear=516、enabled=true、configured_rpm=8000、state=0、fault=0、position_valid=false、brake=true），与上一节裸串口只读回包一致。
- `/robot/stop` → `accepted=False`，消息为只读拒绝并指向硬件急停。
- **`TX_COUNT=0`**：整段运行没有发出任何命令帧，证明只读模式确实拦住了包括 STOP 在内的全部写操作。

### 本轮在验证中发现并修掉的缺陷（均有测试或实测复现）

1. `0x0017` 是寄存器空洞，把状态与位置合成一次 14 寄存器读会被整帧拒绝；已拆成两次读并加了回归测试。
2. 模拟板卡的 FC16 成功应答多回显了字节数字段（9 字节而非 8 字节），传输层判为 CRC 错并重发同帧——意外验证了"同帧重发 + `0x0036/0x0037` 对账"路径可用。
3. 序号被拒时固件**不更新** `0x0036`（`robot.c:480-485`），模拟板卡原先更新了，会让对账判据失效；已按源码修正，并据此把"异常 0x03 + `0x0036` 未变"确定性地判为 SEQUENCE 拒绝而不是 UNKNOWN。
4. `ServerGoalHandle.is_canceling` 在该源码构建的 rclpy 上**不存在**，execute 回调抛 AttributeError → 动作服务器用**空结果**把目标标记 aborted，而此时命令帧**已经发出**。实板上就是"电机在动、ROS 报失败"。已改为按 `status` 判断并兼容多种拼写，加了用缺失该属性的假 goal handle 跑完整回调的回归测试。
5. 维护命令（如 `/robot/set_zero`）回读的是可能早于命令的旧快照，会把 `position_valid` 报错；已改为命令后强制取新快照。
6. 运动进行中 worker 忙于等待，快照不刷新，诊断报 `data stale`；已在等待循环内按轮询周期刷新。
7. 快照尚无数据时 `status is None` 会**短路跳过**无零点相对运动的守卫，命令照发；已改为 fail-closed（先取新读数，仍无数据则拒绝并报未连接）。
8. YAML 的 `driver.allow_motion / allow_unreferenced_relative / polling_hz` 曾被节点忽略（只认 ROS 参数）；已改为"YAML 提供默认值、launch 参数覆盖"，并加了双向测试。
9. `--symlink-install` 直接链接源文件，而 `launch_ros` 用 `os.access(X_OK)` 过滤 libexec 目录：从 Windows 打包同步过去的入口脚本没有可执行位，`ros2 launch` 报"executable not found"。已在 CMake 里用 `add_custom_target(... ALL)` 每次构建补可执行位（只在 configure 期 `execute_process` 会在重新同步后失效）。
10. 实板 5 Hz 轮询出现 `age 1.46 s`。实测归因：每次交易恒定 50.5 ms，正好等于 pyserial 的 `timeout=0.05`——因为 `read(64)` 请求的字节数远大于实际应答（9～37 B），pyserial 只有在**读满请求字节数**时才提前返回，于是每帧都等满超时；一轮 18 次交易 911 ms。先试过 `inter_byte_timeout=5 ms`，实测**无效**（仍是 50.5 ms/帧），已改为**按需精确读取**（先读 2～3 字节定出应答长度，再只读剩余字节），实测降到 **5.6～8.1 ms/帧**，一轮约 126 ms，可支撑约 7.9 Hz。`inter_byte_timeout` 保留（无害且对分片到达有利），但不是起作用的那一项。

远程主机上本轮新增的内容限于：`/data/ros2/robot_motor_ros2_ws`（新工作区，46 MB，含 build/install/log 与 `stage_c_realboard_evidence`）、`~/robot_motor_ros2_ws` 符号链接、`/data/ros2/dsh_iface_probe`（action 生成探测包，12 MB，可删）、`/data/ros2/stage_d_evidence`（模拟后端证据）、`/data/dsh_cfgcheck`（配置模块验证副本，可删）、`/data/dsh_old_ws.tar.gz`（旧工程审查快照，43 KB）、`/tmp/dsh_probe/`（只读探测与延迟测量脚本）。旧工作区、门锁网关、domain 0 上在跑的控制栈均未触碰。

### 会话期间实板状态发生了外部变化（重要，勿与本节软件工作混淆）

同一天 18:30 的实板只读运行（以及 18:40 的独立裸帧复核，均只发功能码 03）读到的板状态与本节开头 14:23 的探测**不一致**：

| 项目 | 14:23 探测 | 18:30 / 18:40 复核 |
|---|---|---|
| 最近序号 `0x0036` | 144 | **178**（期间板子受理了 34 条命令） |
| M4 状态/故障 | state=0、fault=0 | **state=6(FAULT)、fault=4(F_UNEXPECTED)** |
| M4 角度 / FG 计数 | −103.9° / −142 | **+33.6° / 46** |
| M4 valid | 0 | 0 |
| M1 / M2 / M3 | state 0/0/1，fault 全 0，M2 角度 107.9°、M3 valid=1 | **未变**（M2 仍 107.9°/928，M3 仍 valid=1、state=1） |
| 保存标志 `0x0005~0x0007` | 0/0/0 | 0/0/0 |
| UID / `0x000F` | `C23B378115B092083830364A` / 2 | 相同 |
| 四路自检块 | 全 0 | 全 0 |

- 另外 `/dev/serial/by-path/platform-fc800000.usb-usb-0:1.2:1.0-port0` 符号链接的时间戳从 13:55 变为 **17:38**，说明期间发生过一次 USB 重新枚举（仍指向 ttyUSB0）。
- **这 34 条命令不是本轮软件发出的**：两次实板运行的节点日志 `TX ` 计数均为 0（`/data/ros2/stage_c_realboard_evidence/tx_count.txt`），脚本只发功能码 03 读；而 `0x0036` 只在功能码 16 写 `0x0030` 的命令块被受理时才前进。阶段 D 的全部动作都在模拟板卡上。
- 因此本节开头"四路 fault=0"是 14:23 的准确快照，**现在已被超越**：M4 处于 fault 4。按契约，`F_UNEXPECTED` 由累计超过 10 个非预期 FG 边沿触发（`BSP/robot.c:31-37`），只有上电、`CLEAR_FAULT` 或 `ZERO` 会清零；手动转动关节即可造成。本轮**没有**清除该故障、没有设零、没有运动，也没有查证是谁在何时发的这 34 条命令——需要用户确认这段时间是否有人在板上操作。
- 这条变化同时是一个正面的实测证据：驱动的诊断在只读模式下正确报出了 `channel 3 fault 4 (unexpected_fg)`，`/robot/motor_states` 的 `state`/`fault` 字段与独立裸帧复核逐字一致。

### Flash / Monitor / Manual checks

- **Flash：not actually run**。本轮没有烧录任何固件，没有读取板上固件哈希，因此不能声称板上二进制与任何本地构建一致。
- **Monitor：已执行，但仅限只读**。实板只读运行通过 ROS 节点监听真实串口，只发功能码 03；`TX_COUNT=0` 证明没有写命令。阶段 D 的全部动作、自检、设零都发生在**模拟板卡**上，不是实板。
- **Manual checks：not actually run**。没有清故障、没有设零、没有自检、没有运动、没有测量电压或波形、没有核对机械方向或限位。实板上的 `position_valid`、故障与角度仍是本节只读回包所示状态。

## 2026-10-08：驱动五项缺陷修复（仅模拟板验证，未连电机、未烧录）

用户审查 ROS 2 驱动后指出五个缺陷。本节记录**改了什么、依据哪条固件代码、怎么验证的、还剩什么必须在实板上做**。本轮没有连接或驱动真实电机，没有烧录固件，没有对任何板卡发写命令。

### 缺陷与修复

| # | 缺陷（用户指出，已逐条在代码中核实） | 固件依据 | 修复 |
|---|---|---|---|
| 1 | `scheduler.submit()` 放行 STOP，但 `board_session._guard_write()` 在 UNKNOWN 期间拒绝**所有**写，包括 STOP → 异常后无法软件停机 | `robot.c:144-148`（`robot_stop_all` 无条件执行） | `_guard_write(what, allow_while_unknown)`；`send_command(..., allow_while_unknown=True)` **只对 `STOP_OPCODES=(STOP, STOP_ALL)` 生效**，其它 opcode 即使请求放行也抛错。`BRAKE(1)` 固件里等效于停，但**不在**放行名单。停机成功**不**清 UNKNOWN；只读模式仍拒绝 STOP；UID 校验与 STOP 优先级不变。附带修掉同族缺陷：停机期间 `poll` 也被拦，导致 `refresh_now()` 返回旧快照、连"停没停"都无法确认 |
| 2 | 整板自检在 M1 结束就报完成；`passed` 只反映一条记录 | `robot.c:294-296`（逐路推进下一个 `ST_QUEUED`）、`robot.c:102-106`（被丢弃的通道置 `ST_UNTESTED`）、`robot.c:130-139`（scan_all 下 no_pulses/too_few 不中止批次，其余失败中止）、`robot.c:179`（记录带本次序号）、`robot.c:488`（`last_seq` 在 dispatch 前写入，故记录序号＝本次命令序号） | 结束条件改为"无通道处于 queued/running/settling 且无电机在 `S_SELFTEST`"（`UNTESTED` 不是终态，等"四路都终态"会在取消后永久挂住）；按序号过滤记录；新增 `msg/SelftestChannel.msg`，`RunSelfTest.Result` 增加 `batch_completed`/`all_passed`/`channels[]`，Feedback 增加 `active_channel`；超时、取消、异常中止分别处理；假板卡改为**串行队列**（原来四路并行，无法暴露该缺陷） |
| 3 | `MoveJoint.motor_rpm` 只写日志，实际不下发；固件用板上保存的每路 rpm | `robot.c:334`（`desired = r->config.channel[i].rpm`），MOVE 帧只有角度 | 发送前读板上该路生效 rpm：`motor_rpm=0` 用板上值；非 0 且不一致 → **拒绝**，`error_code=111`，一帧不发，消息指向 `/robot/apply_config`。YAML 字段改名 `expected_motor_rpm`（旧名 `default_motor_rpm` 仍解析为别名），`robot.yaml` 由 400 改为实测的 **8000** 并注明这是"期望值"不是指令 |
| 4 | `ApplyConfig.srv` 写明空 `channels` 保留板上配置，实现却用 YAML 的 role/gear/rpm 重建已映射通道 | `robot.c:11-23`（`robot_config_valid`） | 空列表＝以**板上生效值**为基线；部分列表只改列出的通道；与板上完全一致则**不发 CONFIG**（`changed=false`），避免无谓 Flash 写。新增 `protocol.validate_board_image()` 逐条转写 `robot_config_valid`，含两条易错规则：**rpm/gear 范围对禁用通道同样生效**、**禁用通道的非零 role 也参与唯一性检查**（原实现按 `enabled` 过滤，会暂存出板子必拒的镜像）。提交前完整回读暂存区、提交后回读生效区，拒绝或保存失败也报实际状态 |
| 5 | `refresh_now()` 超时仍返回旧快照供绝对运动算位移；读与发之间存在并发窗口；相对运动不核对终点；`limits_rad: null` 时没有任何保护 | — | "读位置→算位移→核对限位与 rpm→发 MOVE"整体移入 worker 线程（持有该路 slot 期间完成），杜绝两个目标基于同一起点各算一遍；`refresh_now()` 失败返回 **`None`** 而不是旧快照，读不到位置直接拒绝（`error_code=113`）；**相对运动也核对预计终点**；`limits_rad: null` 默认**拒绝运动**（`error_code=112`），只有 `commissioning:=true` 才放行，且单条命令被 `max_commissioning_step_deg`（默认 10°）封顶，日志/`motor_states.commissioning`/`soft_limits_enforced`/诊断四处标明无软限位保护；`position_valid=false` 时不声称核对过终点 |

新增驱动结果码：**111** rpm 不一致、**112** 限位未确认、**113** 位置不新鲜、**114** 请求本身不合法。均 ≥100，与固件业务码 0～6 不冲突。

### 验证（实际执行结果）

远程 `xinhua@192.168.1.113`，`/data/ros2/robot_motor_ros2_ws`，Ubuntu 20.04.6 aarch64、源码编译的 ROS 2 Humble、Python 3.8.10、pytest 8.3.5、`ROS_DOMAIN_ID=71`：

- `colcon build`：**3 个包全部成功**（`robot_motor_interfaces` 因新增 `SelftestChannel.msg` 与 `RunSelfTest.action` 字段重新生成接口，耗时约 1 min 28 s；仅有 CMake policy CMP0148 警告，来自 `python_cmake_module`，与本次改动无关）。
- `colcon test`：首轮 **407 个用例 / 15 项失败**，全部定位为**测试自身**问题而非产品缺陷，修正后复跑（分包：launch 9、can_codec 19、config 89、fake_board 42、node 55、protocol 81、scheduler 43、session 49、transport 20）。
- 15 项失败的归因：6 个调度器用例在新增"限位未确认默认拒绝"后需显式走 commissioning 路径（已加 `commissioning()` 助手并在注释写明原因）；2 个用例期望板级 UNREADY(3)/STORAGE(4) 却被更早的 104 拦下（同上）；1 个 protocol 用例把"禁用通道重复 role"构造错了（改为 channel1 禁用且 role=2 与 channel0 相撞）；2 个"停机无应答"用例只丢了一个回包，而传输层会**按原序号原字节重发**，重发即成功（改为丢弃全部命令回包）；1 个 node 取消用例的 `motor_rpm: 400` 现在会被 111 拒绝（改为 0）。
- 新增用例覆盖用户点名的场景：UNKNOWN 后可发 STOP_ALL、其它写命令仍被拒、停机无应答不得报成功；四路依次完成、其中一路失败、后续通道未执行（`UNTESTED`）、保存失败、超时、取消；rpm 的 0/匹配/不匹配/板值变更后；空列表不改配置、部分通道修改不影响其余、重复 role 被拒；旧快照、轮询超时、并发目标、相对运动越界。

### 尚未解决的风险

- **全部结论只来自模拟板卡**。假板卡现在按固件语义串行推进自检队列，但它仍是模型：`all_quiet` 门限、`ROBOT_QUIET_MS`、Flash 写延迟、FG 抖动都与实板有差距。
- **限位仍是 `limits_rad: null`**，所以实板上任何运动都必须 `commissioning:=true`，即**没有软限位保护**，只有 10°/条 的步长上限。机械限位量出后应写进 YAML 并关闭 commissioning。
- **`expected_motor_rpm: 8000`** 只是 2026-09-29 只读探测到的板值；若现场用 `apply_config` 改成 400，YAML 必须同步改，否则诊断会一直报不一致（这是设计意图，不是 bug）。
- 右臂（板 B）三路减速比**仍未确认**，`config/robot_right_arm_UNCONFIRMED_GEARS.yaml` 只能只读用；驱动仍是**一节点一块板**（`_select_board()` 在板数≠1 时抛 ConfigError），双臂同节点属阶段 2。
- 批量自检的 `batch_completed` 判据依赖"没有通道处于 queued/running/settling 且没有电机在 S_SELFTEST"。实板上若有外部扰动使某路反复进入 settling，理论上会拖到超时；超时会转 UNKNOWN 停机，不会误报通过。

### 需要在硬件上验证的具体步骤

均为**只读或单条命令**，逐项授权后执行；任何一步异常就停下记录，不要继续。

1. **停机可用性（缺陷 1，最高优先）**：`allow_motion:=true` 启动 → 发一条会超时的 MOVE（`timeout_sec` 小于实际所需）→ 确认 `unknown_command_state=true` → 调 `/robot/stop`（空 `joint_name`）。判据：`accepted=true`、`stop_confirmed=true`、`unknown_command_state` **仍为 true**（停机不等于确认）；再发一条 MOVE 判据为被拒 `error_code=109`；调 `/robot/acknowledge_unknown` 后才恢复。
2. **只读模式仍拒绝 STOP**：`allow_motion:=false maintenance:=false` 启动 → `/robot/stop` 判据 `accepted=false`、消息含 `read-only`、`TX_COUNT=0`。
3. **整板自检逐路结果（缺陷 2）**：机械空间确认后 `scope: 1`（ALL）。判据：`channels[]` 长度 4、四个 `state` 均为终态、`batch_completed=true`、`all_passed` 与实际一致；**观察确认四路是一个接一个动**，不是同时。中途 Ctrl-C 取消一次，判据：被取消那路 `state=6(cancelled)`，其余 `state=0(untested)` 且 `executed=true`、`batch_completed=false`、消息含 `NEVER ENERGISED`。
4. **rpm 断言（缺陷 3）**：只读回读四路 rpm → 发 `motor_rpm` 等于该值的 MOVE（判据放行）与 `motor_rpm: 400`（若板值是 8000，判据 `error_code=111`、`TX` 不增加）。
5. **配置事务（缺陷 4）**：先 `commit: false` 且 `channels: []` → 判据 `changed=false`、消息含 `CONFIG was NOT sent`、`staged` 与 `applied` 逐字段等于板上现值、Flash 无写入。再构造"禁用通道带非零 role 且与另一路相同"的请求 → 判据 `accepted=false`、消息含 `duplicates` 与 `robot_config_valid`、板上配置未变。
6. **新鲜度与限位（缺陷 5）**：拔掉串口或让端口忙 → 发 MOVE，判据 `error_code=113` 且 `TX_COUNT` 不增加。量出某路机械限位后写入 YAML（去掉 null）→ 设零 → 发一个越界的**相对** MOVE，判据 `error_code=105`、消息给出预计终点与区间、未发帧。
7. **commissioning 上限**：`commissioning:=true max_commissioning_step_deg:=5.0` → 发 10° 目标判据 `error_code=105`；发 3° 目标判据放行，且 `/robot/motor_states.commissioning=true`、`soft_limits_enforced=false`、诊断含"NO soft-limit protection"。
