# 历史参考：Robot_验证记录

> 保留当时方案、观测及验证结果，不是现行操作步骤。历史“当前”仅指记录当天；请使用 [文档索引](../../README.md)。原报文与测量值不改写。

# 机器人固件验证记录

> 本页原有记录为历史版本验证，不是当前发送步骤。单帧协议版本 2 的本次验证另见[迁移与验证记录](Robot_单帧命令迁移.md)，不得照发历史 A55A 提交帧。

当前操作补充：[多次相对角度 Modbus/CAN 完整步骤](../Robot_免序号操作.md#repeated-angle)，以及[连续四次运动待验收项目](../Robot_完整测试流程.md)。本页保留基础版本历史测试、尺寸和哈希；文档补充不表示新的多次运动已完成硬件验收。

历史版本往返自检：去程采集含余转脉冲 N，停稳后反向以 N 为目标返回，最终残差 ≤1 脉冲才通过；两程各限 10%/300 ms。新增阶段和回位结果见[往返结果寄存器](../Robot_电机自检.md#roundtrip-results)。换向仍由主循环在静止确认后执行，中断只截止当前行程输出。旧单向版本验证记录仅供历史追溯。

当前推荐流程已统一为[先四路检测，再配置减速比并保存到板上，读回后设零](../Robot_Modbus与CAN使用手册.md#commissioning)。本文件保留基础版本历史证据，不表示已完成新流程的硬件验收；不因文档更新改写当时结果。

日期：2026-09-25。对象：本次固定四路、角度控制、Modbus/CAN、UID 和 Flash 保存实现。配套说明：[协议与验收](../Robot_固件协议与验收.md)。

这是增加电机响应自检前的基础版本记录，保留当时的实际验证证据。当前自检版本请同时阅读[自检验证记录](Robot_自检验证记录.md)；本文中的产物、大小和测试数量不代表最新版本。

本记录按 [build-test-verify 技能](../../.agents/skills/embedded-codex-workflow-skills-main/build-test-verify/SKILL.md)区分构建、主机测试与硬件验证。工程没有 `docs/ai_context` 持久上下文，本次依据现有构建配置和用户确认的需求实施。没有烧录或连接串口/CAN。

## Verification

### Build

| 项目 | 实际命令 | 结果 | 证据 |
|---|---|---|---|
| CMake 配置 | `cmake --preset debug` | 成功 | 配置输出及 build/debug |
| CMake 完整重编译 | `cmake --build --preset debug --clean-first` | 退出码 0；44 个编译单元并链接 | [构建日志](../../build/robot-cmake.log) |
| 最后串口并发修正 | `cmake --build --preset debug` | 退出码 0；重新编译 USART0 并链接；EIDE 随后也完整重编译通过 | [最终增量日志](../../build/robot-cmake-final.log) |
| EIDE 清单脚本 | `powershell -ExecutionPolicy Bypass -File tools/build_eide.ps1` | 退出码 0；53 个对象并链接 | [构建日志](../../build/robot-eide.log) |
| 主机 C 测试 | `powershell -ExecutionPolicy Bypass -File tools/test_robot.ps1` | 退出码 0，五组通过 | [测试日志](../../build/robot-host-tests.log) |
| Python 工具测试 | `python -m unittest discover -s tests -p test_robot_host.py -v` | 退出码 0，5 个测试通过 | [测试日志](../../build/robot-python-tests.log) |
| CLI 入口 | `python tools/robot_host.py --help` | 正常输出帮助，不打开端口 | 本次命令输出 |

ARM 工具链：工程已有 Arm GNU Toolchain `15.3.rel1`。CMake 保留 `-Wall -Wextra`；主机 C 测试用 `-Wall -Wextra -Werror`，编译实际的 robot/协议/日志/分帧源码，替换硬件回调。为了运行主机测试，在 `build/test-tools` 内安装 `ziglang==0.16.0`，没有修改全局编译器；脚本也接受 `-Compiler` 指定已有的主机 GCC/Clang 路径。

最终两套 ARM 产物的 size 输出均为：`text=19136，data=20，bss=10324` 字节。链接内存报告 FLASH 使用 19156 B / 508 KiB，RAM 使用 10336 B / 64 KiB；4 KiB 栈和 1 KiB 堆包含在链接保留中。映射表的 FLASH 长度为 `0x7F000`，未覆盖日志保留区 `0x0807F000～0x0807FFFF`；包含 `TIMER0_Channel_IRQHandler` 和 robot 主流程。

构建有已记录警告，不能称为零警告：CMake 报旧 Newlib syscalls 的未使用参数，EIDE 报 USB 参考源 `usbd_lld_int.c` 的未使用变量，两者均有链接 LOAD 段 RWX 警告。本次新增 robot 模块没有编译警告。EIDE GUI 本身未启动；上述 EIDE 结果来自读取相同源清单的脚本。

曾发生的环境/测试问题：沙箱最初阻止修改 EIDE 清单及执行 Ninja，获得工具权限后配置/编译成功。第一次主机测试编译因 Zig 优化模式关闭 assert 而触发未使用变量错误，已在测试文件显式启用断言后重新运行成功。PowerShell 将标准错误流的警告/测试输出封装成错误对象，随后用 Python subprocess 直接捕获退出码与合并输出，确认 EIDE、C 测试、Python 测试最终均为 0。日志保存了最终结果，没有把前面的失败算作成功。

### 主机测试覆盖

[C 测试](../../tests/robot_test.c)实际执行了：

- 未配置拒绝运动、配置提交、逐路设零，四路同时运动，固定通道的目标脉冲计算，正负方向、忙拒绝、刹车取消和释放不启动。
- 同序号重复命令不重复移动，同序号不同内容拒绝；缓存淘汰后的旧号拒绝，uint16 回绕，重启后旧运动序号拒绝。
- 已保存位置恢复；dirty 记录恢复后四路全部无效；存储失败阻止起动，失败配置不暴露为已应用配置。
- 反馈超时、总行程超时、漏捕获、静止意外脉冲、位置 int32 越界、超调误差、配置改变使零点失效、关节标签冲突。
- Flash 模拟只允许 1→0 编程。64 个“clean→dirty”中断位置，以及包含擦页和提交字的 65 个“dirty→clean 翻页”中断位置，共 **129 个故障注入用例**；还覆盖持续翻页、CRC 损坏回退。
- Modbus CRC/UID/暂存写、跨非法地址写入的原子拒绝、CAN 编解码/重试，**20000 个带有效 CRC 的异常 ADU**解析。
- RTU 1750 μs 边界、帧内超时、256 字节上限、错误帧丢弃、32 位时间戳回绕。

[Python 测试](../../tests/test_robot_host.py)实际执行了 CRC/有符号值、跨板重复标签/节点、UID 重新绑定拒绝、Modbus 命令字节布局、CAN 命令字节布局。示例整机清单另经校验，默认全部禁用。

这些测试验证软件状态/协议/存储算法，不模拟电机惯性、减速器回差、MCU 外设实际时序、Flash 欠压电气行为或 CAN 收发器。

### Flash

- Command：无。
- Result：**not actually run**，没有烧录或修改板上 Flash。
- Evidence：本次只运行构建、主机模拟测试及静态读取工具；烧录脚本没有执行。

### Monitor

- Command：无。
- Result：**not actually run**，未打开 CH340、CAN 或串口监视器。
- Evidence：Python 只执行帮助和离线单元测试；未实例化真实传输连接。

### Manual checks

- 静态检查：固定引脚与原工程一致；M2 使用 TIMER0_Channel；按 APB 分频计算定时器时钟；PWM 反极性使 compare=0 对应停止高电平；故障入口强制 PWM 高/刹车低；新主循环不运行旧命令行。
- 文件检查：新协议文档和本记录以严格 UTF-8 解码，Markdown 围栏成对，本地文件链接存在；历史交接文档加了改造前快照说明，旧哈希未宣称为当前哈希。
- 实体测量结果：**not actually run**。未声称 PWM 引脚已经测得指定波形，也未声称输出轴已达到 ±5°。

### Not actually run

| 项目 | 原因/待确认内容 |
|---|---|
| 电气检查、上电/复位波形 | 需要实际板与示波器，尤其 DIR/BRAKE 通路极性、FG 上拉电压及复位默认电平 |
| FG 定时器/中断实测 | 需要已知频率脉冲源或实际电机；软件捕获路径已修正，但尚无板上证据 |
| 位置精度与速度环调参 | 需要独立角度测量、实际负载和不同减速器；当前参数是起始值 |
| 实际掉电恢复/Flash 寿命 | 主机故障注入不能替代 MCU 欠压、擦写时长和寿命试验 |
| CH340/CAN 连通、丢帧/总线关闭 | 没有连接实际适配器；CAN 适配器类型仍由上位机使用时指定 |
| 独立 build_verify 脚本运行 | 源文件清单已同步，但本次以 CMake 完整构建和 EIDE 清单脚本为证据 |
| 连续运行、满载多轴安全 | 未实测执行时长、ISR 最坏延迟、栈最高水位或机械安全功能 |

### Next verification step

按[协议文档第 7 节](../Robot_固件协议与验收.md#hardware-acceptance)，先断开电机动力电源，测 PWM 停止电平、BRAKE、方向转换及 FG 输入电压。确认后以低速、卸载方式逐路核对通道和转向，再测试手动零点、小角度、保存与断电；最后调到实际目标转速并验证 ±5°。未预设关节实际接线与减速比，请在上位机配置中据实填写。

## 产物快照

以下 SHA-256 为本次最终读取的 ELF 文件；不同调试信息和构建路径会使两套 ELF 哈希不同，并不表示要把两者混合烧录。

| 文件 | SHA-256 |
|---|---|
| [CMake ELF](../../build/debug/GD32F303VET6_MotorControl.elf) | `1B8988CD99C077C74F6309881A554F1EA84B1F2E0A3AF2C0C42DB2075B72D1BB` |
| [EIDE ELF](../../build-eide/Debug/GD32F303VET6_MotorControl.elf) | `0FCFCB53C1733FEB856D13C58E6B5B31D837C313D205729CABE7DD75946639AA` |

同目录已生成 `.hex/.bin/.map`。当前文件系统没有可用 Git 工作树，未创建提交。生成物和主机编译工具位于 build/build-eide 目录，不属于固件源码。
