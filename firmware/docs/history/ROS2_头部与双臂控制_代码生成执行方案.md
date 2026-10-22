# 历史参考：ROS2_头部与双臂控制_代码生成执行方案

> 保留当时方案、观测及验证结果，不是现行操作步骤。历史“当前”仅指记录当天；请使用 [文档索引](../../README.md)。原报文与测量值不改写。

# ROS 2 头部与双臂控制：代码生成执行方案

更新日期：2026-09-29。适用于 DeepSeek、Qwen 等代码生成模型，不依赖特定模型版本。

本文是可逐阶段执行的开发任务书，不是已实现的 ROS 软件。本文生成过程中未安装软件、未修改固件、未部署节点、未启动电机。下文区分现有事实、实测记录和建议接口。

## 1. 使用方法与目标位置

1. 将本文、必读源码、协议文档和远程已有 ROS 源码交给模型。
2. 模型先执行阶段 A 的只读审查，提交实际阅读清单、差异和实现文件清单。
3. 按 B～F 逐阶段生成代码、构建、验收。离线工作可连续推进；实物配置、自检、设零或运动需本轮明确指令。
4. 每阶段交付完整文件或可应用补丁、运行命令、实际测试结果、剩余问题。未执行不能写“已通过”。
5. 仅阅读本文不等于核对过源码，缺少必要文件必须索取。

实际工程路径：`E:\Date_save\embedded_code\GD32Projects\GD32F303\GD32F303VET6_MotorControl`。
用户消息中的旧路径 `E:\Date\_save\embedded\_code\GD32Projects\GD32F303\GD32F303VET6\_MotorControl` 本次检查不存在；本文保存在当前工程 docs 下，不创建重复工程。

### 必读材料

| 文件 | 用途 |
|---|---|
| [robot.c](../../BSP/robot.c)、[robot.h](../../BSP/robot.h) | 命令、状态、序号、互锁、位置、自检 |
| [robot_protocol.c](../../BSP/robot_protocol.c) | Modbus/CAN 格式与异常映射 |
| [robot_store.c](../../BSP/robot_store.c)、[robot_port.c](../../BSP/robot_port.c) | 保存、恢复、硬件绑定 |
| [robot_host.py](../../tools/robot_host.py) | 客户端参考，需评估兼容后复用 |
| [robot_test.c](../../tests/robot_test.c)、[test_robot_host.py](../../tests/test_robot_host.py) | 现有测试与边界样例 |
| [Modbus/CAN使用手册](../Robot_Modbus与CAN使用手册.md) | 寄存器、命令与错误 |
| [单帧命令迁移](Robot_单帧命令迁移.md) | 协议版本2，旧提交禁写 |
| [无零点相对运动](../Robot_Modbus与CAN使用手册.md#unreferenced-relative) | valid=0 的实际语义 |
| [电机自检](../Robot_电机自检.md)、[协议与验收](../Robot_固件协议与验收.md) | 自检边界、引脚、验收 |
| [历史验证记录](Robot_自检验证记录.md)、[待确认事项](../ai_context/OPEN_QUESTIONS.md) | 按日期理解，旧接线和状态不代表当前 |

远程还须完整审查 `/opt/ros/motor_control_ws/src/modbus-motor-ctl` 的 Python_Ctrl、motor_node、配置、接口、package.xml、setup.py、launch；先检查Git状态并备份改动，不覆盖已有部署。

## 2. 已核对环境与实机映射

本轮通过 SSH 只读确认：

- 主机 `xinhua@192.168.1.113`，Ubuntu 20.04.6，aarch64，Python 3.8.10。
- ROS_DISTRO=humble，rclpy可从 `/opt/ros/humble` 导入，pyserial=3.5。
- colcon：`/home/xinhua/.local/bin/colcon`。
- 已有 `/opt/ros/motor_control_ws`，含 build/install/log/src。
- `ros2_motor`目录的真实ROS包名是`motor_controller`，另有`motor_controller_interfaces`。
- 只验证了环境、基础导入和包清单，尚未全面审查旧ROS实现，不能声称已兼容当前固件。

首版适配现有环境，不擅自升级系统或重装ROS。Python3.8代码避免无保护使用`list[int]`、`X | None`等较新注解行为。新工作区建议`~/robot_motor_ros2_ws`，不覆盖`/opt/ros`中的现有工作区。

板 UID：`C23B378115B092083830364A`，Modbus站号1。

| ROS逻辑关节 | 通道 | 协议编号 | 板上角色 | 减速比 |
|---|---|---:|---:|---:|
| head_joint | M2 | 1 | 1 | 516 |
| left_upper_arm_joint | M1 | 0 | 2 | 516 |
| left_forearm_joint | M3 | 2 | 4 | 290 |
| left_wrist_joint | M4 | 3 | 6 | 82 |

角色6在协议中称左手掌，当前界面称左手腕。四路inverted当前均为0。头部用户确认正角度向前、负角度向后；所有关节完整机械坐标和限位仍需确认。历史文档的“M1头部、M2左大臂”不可直接沿用。

右臂未来角色3/5/7，用新板UID、实际通道配置扩展；不凭左臂推断右臂接线、减速比或正方向。

串口稳定路径：`/dev/serial/by-path/platform-fc800000.usb-usb-0:1.2:1.0-port0`，本轮映射ttyUSB0。换USB口可能改变路径，连接时始终核对UID，不只认ttyUSB编号。

### 最新实测边界

左大臂和左小臂最近返回计数0后，左大臂valid=0，左小臂valid=1。随后左手腕清故障，按1500RPM分别执行+40°、+90°，FG换算位移约+43.9°、+93.7°；完成时无故障、已刹车、保存标志全0，valid仍为0。

左手腕启动时读到约5737/5847RPM，明显高于设定，须排查测速/调速；不能认定根因或宣称恒速验收通过。所有角度来自单线FG，不是独立仪器测量。以上是历史快照，节点启动必须重新读取。

## 3. 范围与包结构

首版实现离散角度运动、查询、停止、自检、故障确认、人工设零、配置管理，预留右臂、多板和CAN。固件不变。没有同步轨迹、连续目标更新、逆运动学或固定到位时间承诺。

```text
~/robot_motor_ros2_ws/src/
  robot_motor_interfaces/      # ament_cmake，msg/srv/action
  robot_motor_driver/          # ament_python，rclpy
    robot_motor_driver/
      protocol.py             # 编解码、枚举、单位
      transport_modbus.py     # pyserial、交易超时、CRC
      transport_can.py        # 后续；未实现时明确拒绝
      board_session.py        # UID、序号、重连、请求日志
      scheduler.py            # 关节锁、整板锁、STOP优先
      motor_node.py           # ROS适配
      fake_board.py           # 无硬件模拟
    test/
  robot_motor_bringup/         # launch、YAML、命令示例
```

先审查旧代码再决定迁移内容，保留归属说明；不直接用subprocess反复运行现有CLI代替协议库。每条串口仅一个工作线程或异步调度器拥有，不在ROS定时回调阻塞整个运动。

暂不把`joint_trajectory_controller`的周期输出转换为MOVE：标准控制器按时间插值并向硬件转发目标，当前固件同一路忙时拒绝新MOVE，无轨迹缓冲。见[Humble官方控制器说明](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)。

## 4. 协议契约

### 4.1 通信和命令

- Modbus RTU：115200、8N1、站号1，二进制，不附CR/LF。
- 先读UID 0x0008～0x000D，协议版本0x000F必须为2；协议版本不是固件构建哈希。
- 功能码0x10完整写0x0030～0x0034：`seq,op,ch,arg_hi,arg_lo`，立即尝试执行。
- 禁止写0x0035，不发送A55A提交帧。参数有符号int32、寄存器大端、CRC低字节先发。
- CAN是自定义协议，不是CANopen；500kbit/s、标准帧、DLC8，请求ID=0x600+node，应答ID=0x580+node。
- CAN执行数据：`03 seq_hi seq_lo op ch arg_hi arg_lo 00`，arg为int16；每次读1～2寄存器。
- 固件选择Modbus或CAN之一，不假定同时启用。首版Modbus实装，CAN若只做离线测试须明确未实测。

| 操作码 | 功能 | 参数/限制 |
|---:|---|---|
| 1 | CONFIG | 整板暂存配置提交并保存 |
| 2 | ZERO | 指定路当前姿态设零并保存 |
| 3 | MOVE | 相对输出轴角度，单位0.1° |
| 4 | STOP | 指定路停止 |
| 5 | STOP_ALL | 整板停止 |
| 6 | BRAKE | 1刹车、0释放 |
| 7 | DIRECTION | 0/1；MOVE会依据符号重新设置 |
| 8 | SAVE | 保存位置，不提交暂存配置 |
| 9 | CLEAR_FAULT | 显式清除指定路故障 |
| 10 | SELFTEST | 单路；0正向、1负向 |
| 11 | SELFTEST_ALL | 四路逐一自检；参数同上 |

### 4.2 寄存器和状态

通道基址`B=0x0100+0x40*ch`，ch为0～3。

| 地址/偏移 | 内容 |
|---|---|
| 0x0005～0x0007 | dirty、storage_error、pending_save |
| 0x0036～0x0037 | 最近序号、业务结果 |
| B+0～4 | 生效enabled、role、gear、inverted、rpm |
| B+8～12 | 暂存配置 |
| B+0x10～0x16 | state、fault、valid、brake、direction、duty、rpm |
| B+0x18～0x1D | 三个int32：角度0.1°、FG位置计数、目标角度0.1° |
| B+0x20～0x27 | 自检状态、原因、累计脉冲、去程通电ms、完成运行时刻、序号 |
| B+0x28～0x2F | 往返阶段、回程通电ms、去程/回程脉冲、有符号残差 |

状态0～7：UNREADY、IDLE、RUNNING、DECEL、SETTLING、DONE、FAULT、SELFTEST。UNREADY且valid=0、无故障，不等于禁止显式相对MOVE。

故障0～7：无、无反馈、运动超时、捕获错误、非预期FG、位置误差、存储、自检。业务结果0～6：成功、参数、忙、未就绪、存储、序号、地址。

Modbus异常：地址→02；参数/序号→03；忙→06；其他业务失败→04。Modbus异常04不是电机故障4，必须分层解释。

### 4.3 序号、失联与取消

自动化使用非零序号，整板统一分配，读取最近序号后初始化；回绕跳0，考虑16条RAM缓存、半范围顺序检查和重启持久序号。被拒绝的命令也可能消耗序号并缓存失败。

发送前记录UID、ROS目标ID、序号、操作码、参数和时间。相同序号同参数只在去重条件成立时返回既有结果，不是永久exactly-once保证。读请求可有限重试；MOVE超时先查询对账，不换新序号重发。不确定是否已执行、是否重启或缓存是否有效时标记UNKNOWN，停止新动作调度，人工处置。重连不续跑，不拿旧DONE当新请求成功。

STOP优先于待发送普通任务，但不能抢占半帧；单次交易超时必须有界。取消动作发送STOP并等停止确认；无回包时报告停止未确认，不声称机械已停。

## 5. 角度、速度与并发

ROS角度用弧度，固件参数`round(rad*180/pi*10)`；定义正负对称舍入并测试半单位边界。不再乘减速比。单次MOVE限±360°，超限拒绝，首版不隐式拆分。

FG为电机每圈6个上升沿，输出轴分辨率360/(gear*6)度/脉冲；单线不能测真实方向。绝对定位需valid=1、状态新鲜、关节空闲，并在持有关节锁时算差值。valid=0仅显式相对动作，不自动ZERO、不伪造绝对位置。软绝对限位对valid=0不具备可信依据。

RPM为减速前400～8000，固件自动减速；timeout是监控期限，不是承诺到位时间。RPM反馈是无符号幅值，若推算有符号速度必须注明方向来自命令。

更改RPM须整板静止并CONFIG写Flash。只在变化时提交，提交前核对全部暂存区，之后读回；不能每个控制周期提交。默认`keep_requested`保留请求速度减少写入；可选`restore_previous`但必须整板静止后恢复。历史人工测试每次恢复8000，不要求ROS照搬。

每关节动作锁＋每板通信/维护锁。首版整板串行。未来同板并行时先一次配置所需速度，再启动各路；不保证同步。多板启动非原子，一板失败时停止相关已启动动作并逐一核实。

固件无通信心跳停机，失联时已接受MOVE可能继续到目标或故障。节点退出STOP只属尽力措施，不能代替独立急停。

## 6. ROS接口约定（待实现）

### MoveJoint.action

```text
# Goal
uint8 RELATIVE=0
uint8 ABSOLUTE=1
string joint_name
uint8 mode
float64 angle_rad
uint16 motor_rpm
float64 timeout_sec
---
# Result
bool motion_completed
bool persisted
bool position_valid
uint16 error_code
string message
float64 final_angle_rad
---
# Feedback
uint16 command_sequence
uint8 state
uint8 fault
bool position_valid
bool brake
bool communication_ok
float64 current_angle_rad
float64 target_angle_rad
uint16 measured_motor_rpm
```

拒绝NaN/Inf、未知关节、非法RPM/模式/超时、越限、冲突动作。valid=false时角度是诊断计数换算，不是可信绝对姿态。参数量化为0时显式返回NO_OP，不等待新DONE。

本请求接受后，检查目标路DONE、fault0、brake1、PWM停止及停止确认；单路首版再等待dirty/storage_error/pending_save全0。未来并行模式分别报告运动完成和保存完成，不能提前报已保存。

| 接口 | 类型 | 作用 |
|---|---|---|
| /robot/move_joint | MoveJoint Action | 相对/绝对动作 |
| /robot/selftest | 自定义Action | 单路或整板，反馈往返结果 |
| /robot/stop | Service | 指定路或整板，区分接受与停稳 |
| /robot/clear_fault | Service | 显式确认，不自动续跑 |
| /robot/set_zero | Service | 人工参考姿态，全板静止，保存 |
| /robot/set_brake | Service | 独立刹车/显式释放 |
| /robot/apply_config | Service | 配置事务、保存、读回 |
| /robot/motor_states | 自定义Topic | UID、通道、状态、valid、原始计数、保存、时间、新鲜度 |
| /joint_states | sensor_msgs/JointState | 仅新鲜有效的位置估计，不给假零或假effort |
| /diagnostics | diagnostic_msgs/DiagnosticArray | 通信、故障、配置差异、存储 |

建议初始轮询5Hz，经总线负荷和停止延迟测试后调整；旧数据不能换时间戳伪装新测量。ROS时间用于消息，单调时钟用于超时。失联标记stale。

## 7. 建议配置格式

以下格式需新代码实现，不适用于未经审查的旧节点。

```yaml
driver:
  allow_motion: false
  allow_unreferenced_relative: false
  polling_hz: 5.0
  board_motion_policy: serial
  speed_policy: keep_requested
boards:
  main_board:
    uid: "C23B378115B092083830364A"
    transport: modbus_rtu
    port: "/dev/serial/by-path/platform-fc800000.usb-usb-0:1.2:1.0-port0"
    address: 1
    baudrate: 115200
joints:
  head_joint:
    board: main_board
    channel: 2
    role: 1
    gear_ratio: 516
    inverted: 0
    default_motor_rpm: 400
    limits_rad: null
  left_upper_arm_joint:
    board: main_board
    channel: 1
    role: 2
    gear_ratio: 516
    inverted: 0
    default_motor_rpm: 400
    limits_rad: null
  left_forearm_joint:
    board: main_board
    channel: 3
    role: 4
    gear_ratio: 290
    inverted: 0
    default_motor_rpm: 400
    limits_rad: null
  left_wrist_joint:
    board: main_board
    channel: 4
    role: 6
    gear_ratio: 82
    inverted: 0
    default_motor_rpm: 400
    limits_rad: null
```

YAML通道1～4仅在协议边界转0～3。启动只核对配置，不自动写default_motor_rpm。limits_rad=null表示未确认，默认仅只读/模拟；现场调试须显式启用commissioning模式及受限单次范围，不编造限位。

待确认：各轴机械方向/限位、负载速度、右臂UID/接线、多关节协调需求、跨关节故障停止策略、独立急停。缺失不阻塞模拟与只读开发，但阻塞对应实物验收。

## 8. 按阶段执行

### A：只读审查

以下命令现在可执行，不发送电机指令：

```bash
ssh xinhua@192.168.1.113
cat /etc/os-release
uname -m
python3 --version
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
python3 -c 'import rclpy, serial; print(rclpy.__file__); print(serial.__version__)'
command -v colcon
ls -l /dev/serial/by-path/
find /opt/ros/motor_control_ws/src/modbus-motor-ctl -name package.xml
git -C /opt/ros/motor_control_ws/src/modbus-motor-ctl status --short
```

Git命令仅适用实际Git仓库，否则记录非Git并制定备份方案。检查目标串口占用，不关闭其他业务串口进程。完整阅读旧业务代码、接口和依赖，列协议地址/单位/提交方式差异。

交付：`docs/compatibility_audit.md`、`docs/protocol_contract.md`及文件清单。

### B：独立协议库与模拟板卡

实现CRC、寄存器编解码、交易超时、序号和假传输，不依赖ROS即可单测。覆盖CRC错误、分片/粘包、负int32、异常帧、UID/版本不符、回绕、掉应答、失败缓存、重启、旧提交拒绝。

CAN若实现仅宣称已做的格式单测；未实现接口明确报不支持，不假返回成功。

### C：构建与只读节点

先生成第3节包和第7节配置，将配置样例放到`~/robot_motor_ros2_ws/config/robot.yaml`。以下命令是生成之后的验收要求，目前包尚不存在：

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_motor_ros2_ws
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
ros2 launch robot_motor_bringup bringup.launch.py transport:=fake allow_motion:=false
```

另开终端source环境后：

```bash
ros2 topic echo /robot/motor_states --once
```

模拟通过后，只读实板启动示例：

```bash
ros2 launch robot_motor_bringup bringup.launch.py transport:=modbus_rtu allow_motion:=false config_file:=$HOME/robot_motor_ros2_ws/config/robot.yaml
```

生成器必须实现这些launch参数。只读模式拦截所有写入，包括CONFIG/ZERO/CLEAR/STOP；需要维护停止须显式启用维护入口。启动/退出不偷偷发自检或运动。缺依赖列出准确缺项和安装方案，不擅自升级系统。

### D：动作及维护的模拟验收

覆盖取消/停止确认、fault4不自动清除、valid0相对许可/绝对拒绝、UNKNOWN、存储失败、配置锁、同路冲突、多板身份、停止优先和断线不续跑。

以下例子只用于fake后端，并在该模拟后端显式allow_motion=true；真实后端不得照抄执行：

```bash
ros2 action send_goal /robot/move_joint robot_motor_interfaces/action/MoveJoint "{joint_name: 'left_forearm_joint', mode: 0, angle_rad: 0.1745329252, motor_rpm: 400, timeout_sec: 30.0}" --feedback
```

代表相对+10°，不是30秒到位。假板卡不得打开真实串口。

### E：经授权的单关节实测

UID/协议/接线/机械方向/限位核对→只读状态→必要时显式清故障并观察→静止确认→需要绝对参考时人工设零→逐路小角度→DONE/停止/保存→核对物理方向及外部角度。

自检本身会运动，不每次自动执行。正负方向、取消、失联、断电恢复分别验收，破坏性条件测试需机械支持到位再授权。左手腕RPM异常未查明前不启用无人值守动作序列或宣称恒速通过。

### F：右臂与多板

添加新板真实UID和关节YAML，不复制左右臂逻辑。独立USB串口可分别站号1，但须各自校验UID；同一总线不得地址冲突。CH340 TTL现有接线不能凭软件变成多站RS485。CAN扩展另核验收发器与节点号。

首版每板串行，未来并行须增加跨板失败模拟和停止验证。要MoveIt/严格轨迹时另设计连续硬件接口、看门狗、限位和反馈，不在周期write中反复MOVE/CONFIG。

## 9. 错误处置与验收

| 问题 | 行为 |
|---|---|
| UID/协议不符 | 拒绝写入，提示核对目标板 |
| fault4 | 动作失败，不自动清；检查外力/FG，显式CLEAR后观察 |
| valid0 | 禁绝对；相对依明确策略；不自动ZERO |
| 无反馈/捕获错误 | 停止并报告，不自动重试 |
| 异常03/06/04 | 区分参数/序号、忙、未就绪/存储，查询业务结果 |
| 无应答 | 对账，UNKNOWN，不换号重复MOVE |
| STOP无确认 | 明确报告停止未确认，不能假成功 |
| 保存非0 | 不报告已保存，存储故障进入维护状态 |
| RPM明显偏离 | 诊断和按已确认策略处置，不宣称恒速 |

验收清单：

- Python3.8/Humble实际构建、导入、接口发现及测试通过。
- 配置错误、重复通道、未知关节、非法减速比/限位有测试。
- 默认只读；启动、退出、重连不产生运动。
- 无A55A、无序号0自动重试、无隐式ZERO/CLEAR。
- 一次ROS目标只对应一次新运动身份；结果未知不制造重复运动。
- 完成/保存/位置可信度/数据新鲜度分别报告。
- 不制造effort、机械精度、真实方向或转速证据。
- 测试记录含UID、协议、源码提交、ROS提交、原始回包和未验证项。
- 保留旧工作区，提供部署、日志与回滚说明。

## 10. 可直接复制给模型的任务提示词

```text
依据本文和提供的固件源码开发ROS 2电机控制软件。
先审查远程已有ROS工程，不直接覆盖，不修改MCU、不升级系统、不自动运行电机。

先完成A阶段报告，再依次完成B、C、D的离线和只读开发：
1. 输出实际已读文件、协议差异、待确认项。
2. 在~/robot_motor_ros2_ws生成独立包和配置，不冲突旧包。
3. 实现可单测协议库、假板卡、接口、状态机及只读节点。
4. 给完整新增文件或补丁、构建测试命令和实际证据。
5. 没有执行能力时明确未执行，不用伪代码宣称完成。

硬约束：Python3.8/Humble；UID绑定；协议版本2；单帧执行；
整板非零序号；MOVE超时不换号重发；每串口唯一所有者；
ROS弧度转换输出轴0.1度、不重复乘减速比；RPM为减速前；
valid0不伪造绝对位置；不自动清故障/设零/自检/重连续跑；
CONFIG整板静止且写Flash、不得高频；取消检查停止；
应答不等于完成，完成不等于保存；不承诺同步轨迹或固定到位时间。

当前M1左大臂516、M2头部516、M3左小臂290、M4左手腕82。
右臂通过配置扩展，缺UID/方向/限位保持未配置。
实物写入与运动等待本轮明确指令。
每阶段给实际验证、错误处理、剩余问题和回滚方法。
```

## 11. 本文证据范围

本轮读取本地README、robot.h、robot_protocol.c、robot.c命令/去重逻辑、robot_host.py接口与CAN实现、历史待确认记录；远程仅核对环境、基础ROS导入、包文件清单和package.xml。没有通读旧ROS业务实现，不声明其已兼容。

本文生成后仅检查UTF-8、链接、标题及代码块结构；没有执行文中colcon/ROS动作/安装命令。文件中未来命令是对代码生成结果的验收约定，不是当前存在的软件能力。
