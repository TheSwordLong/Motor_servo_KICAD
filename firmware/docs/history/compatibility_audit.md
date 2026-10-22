# 历史参考：compatibility_audit

> 保留当时方案、观测及验证结果，不是现行操作步骤。历史“当前”仅指记录当天；请使用 [文档索引](../../README.md)。原报文与测量值不改写。

# ROS 2 旧工程兼容审查（阶段 A）

审查日期：2026-09-29（远程主机本地时间）。执行方式：`ssh xinhua@192.168.1.113` **只读**命令 + 把旧源码打包取回本地逐行阅读。本轮未修改远程任何文件（仅在 `/data` 写了一个临时审查包 `dsh_old_ws.tar.gz`，sha256 前缀 `69e4ba61c0c0e37f`，可随时删除），未安装任何软件，未构建，未启动节点，未向板卡发送写命令。

依据任务书：[ROS 2 代码生成执行方案](ROS2_头部与双臂控制_代码生成执行方案.md) 第 8 节阶段 A。

## 1. 实际已读清单

远程旧工程（Git 仓库 `src/modbus-motor-ctl`，分支 `master`，HEAD `b7881bd` 2026-08-20，远端 `https://gitee.com/xmkj_-robot_-new/modbus-motor-ctl.git`，`git status --short` 为空＝工作区干净）：

| 文件 | 行数 | 阅读程度 |
|---|---:|---|
| `ros2_motor/motor_controller/motor_node.py` | 335 | 全文 |
| `ros2_motor/motor_controller/__init__.py` | 22 | 全文 |
| `ros2_motor/setup.py` / `launch/motor_controller.launch.py` / `package.xml` | 31 / 20 / 29 | 全文 |
| `ros2_motor/config/motor_params.yaml` | 130 | 全文 |
| `ros2_motor/msg/MotorStatus.msg`、`srv/MoveToPosition.srv`、`srv/SetLimit.srv` | 5 / 6 / 5 | 全文 |
| `motor_controller_interfaces/CMakeLists.txt` + 同名 msg/srv | 25 | 全文 |
| `Python_Ctrl/motor_controller.py` | 453 | 全文 |
| `Python_Ctrl/serial_comm.py` / `modbus_crc.py` / `config_loader.py` / `main.py` | 122 / 53 / 147 / 82 | 全文 |
| `config/motor_config.yaml` | 100 | 全文 |
| `Cpp_Ctrl/`（4 `.cpp` + 3 `.h` + CMakeLists） | — | 目录与文件清单，未逐行读 |
| `Sensors/sensors.py`、`motor_debug_gui.py` | 311 / 478 | 未逐行读，仅确认与电机控制链路无关或为独立工具 |

未读部分不影响本结论：电机控制链路的协议语义全部由上表前两组文件决定。

## 2. 远程环境实测

| 项 | 实测值 | 证据命令 |
|---|---|---|
| 架构/系统 | aarch64，Ubuntu 20.04.6，Python **3.8.10** | `uname -m`、`python3 --version` |
| ROS | `ROS_DISTRO=humble`，`rclpy` 导入成功（`/opt/ros/humble/rclpy/...`） | `source /opt/ros/humble/setup.bash` 后导入 |
| Humble 安装方式 | **colcon 源码安装**（存在 `/opt/ros/humble_src`、`ros2.repos`），`dpkg -l \| grep -c ros-humble` = **0** | 见实测 |
| 构建工具 | `colcon` = `/home/xinhua/.local/bin/colcon`（colcon-core 0.21.1、colcon-ros 0.5.0、colcon-test-result 0.3.8）、cmake 3.25.2、gcc 9.4.0、setuptools 75.3.4、empy 3.3.2 | `pip3 list`、`cmake --version` |
| 测试 | `pytest 8.3.5`（用户站点 `~/.local/lib/python3.8/site-packages`） | `python3 -m pytest --version` |
| ament 组件 | 有：`ament_cmake`、**`ament_cmake_python`**、**`ament_cmake_pytest`**、`ament_cmake_test`、`rosidl_default_generators`、`rosidl_generator_py` 可用 | `ls /opt/ros/humble`、rosidl 导入测试 |
| **缺失** | `ament_python`、`diagnostic_msgs`、`actionlib_msgs` | `ros2 pkg prefix` 逐项探测 |
| 有消息包 | `std_msgs`、`sensor_msgs`、`builtin_interfaces`、`action_msgs`、`unique_identifier_msgs`、`launch_ros` | 同上 |
| 旧工作区 | `/opt/ros/motor_control_ws` 与 `/data/ros2/motor_control_ws` 是**同一目录**（inode 363983，同一 ext4 分区绑定挂载），含 `build/install/log/src`，`install/` 内已有 `motor_controller`、`motor_controller_interfaces`、`MotorController`、`doors_control`、`kafka_command_gateway`、`servo_control_pkg`、`navi_servo_coordinator` 等包 | `stat -c %i`、`ls install/` |
| 串口 | 本板稳定路径 `platform-fc800000.usb-usb-0:1.2:1.0-port0` **当前＝ttyUSB0**（1a86:7523 HL-340，自编译 `/lib/modules/5.10.209/extra/ch341.ko`），空闲无占用；ttyUSB1＝CP2102＝`/dev/door_lock_uart`，被进程 `256299 /usr/bin/python3 -m gateway.kafka_server` 占用 | `ls -l /dev/serial/by-path/`、`fuser`、`lsusb` |
| 磁盘 | `/`（含 `/home`）**已用 97%，仅剩约 496 MB**；`/data` 剩 19 GB | `df -h` |
| 其他负载 | load average ≈10，另有 `robot-gateway`、`robot-h5`（pm2）及运行中的 ROS 控制栈（`/controller_manager`、`/joint_state_broadcaster`、`/diff_drive_controller`、多个 `robot_state_publisher`） | `uptime`、`pm2 ls`、`ros2 node list` |

## 3. 旧实现的协议契约（与本板固件的真实差异）

旧栈控制的是**另一种设备**，不是本工程的 GD32 机器人板。逐条差异：

| 维度 | 旧栈（`motor_config.yaml` + `Python_Ctrl`） | 本板固件（实测回包 + 协议文档） |
|---|---|---|
| 波特率 | **9600** 8N1，注释写“固定为 9600，不可修改” | **115200** 8N1 |
| 控制方式 | 功能码 `06` 写寄存器 `0x0001`，值＝控制字：`0001` 停止、`0002` 上、`0004` 下（整帧从 YAML 硬编码十六进制串取来） | 功能码 `0x10` 一次写 `0x0030～0x0034`＝`seq,op,ch,arg_hi,arg_lo` 的命令帧，操作码 1～11 |
| 位置读取 | `01 03 00 02 00 01`，返回 **单个 uint16**，范围 0～65535，视为“脉冲/位置计数” | 通道基址 `B=0x0100+0x40*ch`，`B+0x18` 起 **三个 int32**（输出轴角度 0.1°、FG 计数、目标角度），另有 state/fault/valid/brake/dir/duty/rpm |
| 写位置 | `06` 直接写 `0x0002`（`set_position()`） | 本板 `0x0035` 为禁写、旧提交帧 `A55A` 被拒绝；不存在“直接改写位置”的合法入口 |
| 多轴 | 单电机（`motor_name: "motor1"`） | 四通道 M1～M4，各有 enabled/role/gear/inverted/rpm 配置 |
| 减速比 | 无概念 | gear=516/516/290/82，角度按输出轴，客户端**不得再乘减速比** |
| 身份/版本 | 无 | UID `0x0008～0x000D`（本板 `C23B378115B092083830364A`）、命令协议版本 `0x000F=2` |
| 序号与去重 | 无 | 整板非零序号、`0x0036/0x0037` 最近结果、失败也缓存 |
| 故障/自检/刹车 | 无（只有主机侧“位置停滞 stuck_threshold”猜测式判据） | fault 0～7、SELFTEST/SELFTEST_ALL、BRAKE、CLEAR_FAULT、保存标志 `0x0005～0x0007` |
| 到位判据 | 主机轮询 `abs(pos-target)<=tolerance`、连续 5 次不变判“停滞”、超时后 `send_stop()` | 由固件状态机给出 DONE/DECEL/SETTLING/FAULT 及停止确认，主机不得自造判据 |
| 话题/接口 | `/motor/position`（`JointState.position`＝**原始计数**）、`/motor/status`、`/motor/stop`、`/motor/move_to_position`、`/motor/set_limit` | 任务书要求 `/robot/move_joint`（Action）、`/robot/motor_states`、`/joint_states` 仅发布**新鲜且有效**的弧度位置 |
| 并发模型 | `handle_move_to_position` 在 service 回调内 `while True` 阻塞轮询（默认单线程执行器会独占节点），GUI/节点退出时**无条件** `send_stop()` | 要求每串口唯一所有者 + 关节锁/整板锁 + 有界交易超时，退出停止只属尽力措施 |

**结论：旧栈不可复用为本板驱动。** 它的寄存器语义与波特率对本板完全错误。按 [协议契约](protocol_contract.md) 逐条核对旧栈会发出的帧，后果是确定的：

| 旧栈帧 | 本板实际行为 | 依据 |
|---|---|---|
| `01 06 00 01 00 02`（“向上”，CRC 59 CB） | `0x0001` 是**只读**的活动传输选择寄存器 → `R_ADDRESS` → **Modbus 异常 02**，电机不动 | `BSP/robot.c:500`、可写判定 `robot.c:564-568` |
| `01 06 00 01 00 01`（“停止”）/ `01 06 00 01 00 04`（“向下”） | 同上，异常 02 | 同上 |
| `01 03 00 02 00 01`（“读位置”） | 读回的是**板级 Modbus 站号**（本板实测为 1），旧栈会把它当成“位置=1”发布到 `/motor/position` 和 `JointState` | `BSP/robot.c:502`；实板回包 `01 03 08 00 00 00 01 00 01 61 A8 D1 F9` |
| `01 06 00 02 xx xx`（“写位置”） | `0x0002` 只读 → 异常 02 | `BSP/robot.c:502` |
| `01 06 00 35 A5 5A`（旧 A55A 提交） | `0x0035` 是墓碑寄存器：读恒 0、写被拒 → 异常 02，无副作用 | `BSP/robot.c:514`、`tests/robot_test.c:211-214` |

也就是说，把旧节点接到本板**不会**造成误运动（所有写都被拒），但它会以 9600 波特率收发（本板固定 115200，收不到任何应答），并在偶尔读到 `0x0002` 时把站号当成位置发布。旧节点也**当前无法启动**：`ros2_motor/motor_controller/__init__.py` 硬编码 `sys.path` 与文件路径 `/home/linaro/Machine_Ctrl/Python_Ctrl`，而本机 `/home/linaro` **不存在**（实测 `ls: cannot access '/home/linaro'`）。

### 3.1 审查中新发现、会影响任何上位机的固件事实

以下由 [协议契约](protocol_contract.md) 的逐行核对得出，均已在本轮新代码中处理：

- **`0x0017` 是空洞**：通道状态在 `B+0x10..0x16`、位置在 `B+0x18..0x1D`，中间那个偏移未映射；跨空洞的多寄存器读会**整帧失败**（`robot.c:561`），必须拆成两次读。本轮实现里就踩过这个坑并已修正。
- **CONFIG 不改当前站号/节点/传输**：`protocol/address/node` 只在 `robot_init` 赋值（`robot.c:69-70`），CONFIG 只更新配置结构（`robot.c:400`），只有 `pwm_hz` 立即生效（`robot.c:406`）。改站号后必须重启才生效，否则会“改完就失联”。
- **异常码比业务码粗**：`0x03` 同时覆盖 `R_VALUE` 与 `R_SEQUENCE`，`0x04` 同时覆盖 `R_UNREADY` 与 `R_STORAGE`（`robot_protocol.c:11-12`）。唯一判据是 `0x0036` 是否等于本次序号：序号被拒时它不变（`robot.c:480-485`），业务拒绝时它等于本次序号（`robot.c:488-494`）。
- **默认构建没有硬件急停**：`BUTTON_ESTOP_ENABLE = 0`（`BSP/board_config.h:111`），`stop_input` 恒为 0，`robot.c:367-368` 的急停互锁不生效。软件 STOP 因此是唯一停止路径，且**不是安全功能**。
- **`F_STORAGE=6` 在固件里没有置位路径**：存储故障只通过 `0x0006` 与 `R_STORAGE` 表达，上位机不要等 `B+0x11==6`。
- **量化陷阱**：`gear=82` 时 `arg=1`（0.1°）→ `steps=0`，板子仍会走完 RUNNING→SETTLING→**DONE**，但输出轴一点没动。NO_OP 判据必须是 `steps==0` 而不是 `arg==0`。
- **`dirty=1` 期间 MOVE 不落盘序号**（`robot.c:453-456` 对比 `360-362`），存在重启后同序号被再次受理的窗口，所以重启后一律按 UNKNOWN 处理。
- **RTU 门限是 750/1750 µs**（不是 3.5 字符），帧内超过 750 µs 的空隙整帧静默丢弃；发送忙、接收溢出、地址不符、CRC 错、超长同样**完全无应答**（`robot_rtu.c:10-15`、`robot_port.c:160-173`）→ 必须严格单条未完成事务，且超时逻辑要容忍“收不到任何东西”。

可复用内容（仅语义，不 import、不复制文件名）：`Python_Ctrl/modbus_crc.py` 的 CRC16 实现为标准 Modbus CRC-16（多项式反射 `0xA001`、初值 `0xFFFF`、低字节先发），与本板一致；本地参考实现见 `tools/robot_host.py`。旧 `serial_comm.py` 的逐字节超时读法说明 RS-485 半双工需要发送前清缓冲，新实现保留该做法。归属：以上思路来自远程 `modbus-motor-ctl`（gitee，MIT 声明于其 `package.xml`），新代码为独立实现。

## 4. 对新代码的工程决定（含与任务书的偏差）

1. **构建类型偏差**：任务书第 3 节要求 `robot_motor_driver`、`robot_motor_bringup` 用 `ament_python`。本机 Humble 为源码安装且 `ament_python` 缺失（`ros2 pkg prefix ament_python` → MISSING），且无 apt ROS 包可装。三个包一律改用 **`ament_cmake` + `ament_cmake_python`**（`ament_python_install_package` / `ament_python_executable`），测试用已存在的 `ament_cmake_pytest`。这是能力等价替换，不是升级系统。
2. **不覆盖旧工作区**：新工作区独立。因 `/` 仅剩 496 MB，实际目录建在 `/data/ros2/robot_motor_ros2_ws`，并在 `/home/xinhua/robot_motor_ros2_ws` 建立指向它的**符号链接**，使任务书中的 `~/robot_motor_ros2_ws` 命令原样可用。旧 `/opt/ros/motor_control_ws` 保持不变。
3. **包名不冲突**：`robot_motor_interfaces`、`robot_motor_driver`、`robot_motor_bringup`，与旧 `motor_controller` / `motor_controller_interfaces` 不同名，可在同一环境共存。
4. **`diagnostic_msgs` 缺失**：`/diagnostics`（`DiagnosticArray`）在远程当前**无法构建**。处理方式：代码把诊断发布做成可选（探测到 `diagnostic_msgs` 才发布），缺失时节点照常运行并在日志中报告缺项；安装方案见第 5 节。不得伪称已实现诊断话题。
5. **不得污染在跑的控制栈**：远程 domain 0 上已有 `/controller_manager`、`/joint_state_broadcaster`、多个 `robot_state_publisher`。因此 launch 默认设独立 `ROS_DOMAIN_ID`，且 `joint_states` 发布默认关闭（需显式开启）。这是安全默认，不改变任务书接口定义。
6. **串口纪律**：只允许打开本板稳定路径，不用裸 `ttyUSBn`；绝不打开或杀 `ttyUSB1`/`/dev/door_lock_uart`（门锁网关在用）。默认 `allow_motion: false`，只读模式拦截包括 STOP/CLEAR/ZERO/CONFIG 在内的全部写操作。

## 5. 待确认与缺项清单

- ~~需在远程构建前确认：`ament_python` 缺失是否接受用 `ament_cmake_python` 替代~~ → **已实证可行**：三个包在远程 `colcon build --symlink-install` 全部成功，`colcon test --parallel-workers 1` 352 项测试全部通过（含 `ament_cmake_pytest` 驱动的 8 个测试目标）。无需从源码补装 `ament_python`，也没有升级系统。
- `diagnostic_msgs` 若需要真实现：只能从 `ros2/common_interfaces` 源码在该机自行构建一个覆盖层，需用户批准安装动作。当前实现按运行时探测降级到 `/robot/diagnostics_json`，节点启动日志会点名该缺项。
- 各关节机械正方向与限位、右臂板 UID 与接线、负载转速：仍按任务书保持未配置状态（`limits_rad: null`）。
- 左手腕 RPM 异常（历史读到约 5737/5847 而设定 1500）根因未查明，未做无人值守动作验收。
- 本轮**未执行**：任何写命令或运动命令；未连实板运行 ROS 节点（实板仅被只读探测）；旧工作区未改动；除审查包外远程无新增文件。
