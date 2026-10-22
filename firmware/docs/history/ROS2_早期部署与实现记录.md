# ROS2早期部署与实现记录

> 旧README原文；板状态、磁盘占用和未实现项均指当时。当前操作见 [使用手册](../ROS2_使用与测试手册.md)。

# ROS 2 头部与双臂控制软件（robot_motor_*）

## ROS2 度数控制接口（2026-10-09）

日常操作新增 MoveJointDeg、move_joint_deg 与 motor_states_deg，可直接填写 angle_deg。旧弧度接口保留，标准 /joint_states 仍为rad。完整操作见 [度数接口使用说明](../ROS2_使用与测试手册.md#degrees-interface)。下方旧弧度示例作为兼容说明，历史日志不改写。


本目录是 [ROS 2 代码生成执行方案](ROS2_头部与双臂控制_代码生成执行方案.md) 的实现源码。源码在固件仓库内做版本管理，构建与测试在远程主机 `xinhua@192.168.1.113`（Ubuntu 20.04.6 / aarch64 / 源码安装的 ROS 2 Humble / Python 3.8.10）上进行。

**这份软件默认不动电机。** `allow_motion` 缺省为 `false`，此时驱动处于只读模式，拦截一切写操作——包括 STOP。

## 1. 包结构

```text
ros2/
  robot_motor_interfaces/    ament_cmake：msg / srv / action 定义
  robot_motor_driver/        ament_cmake + ament_cmake_python：驱动与节点
    robot_motor_driver/
      protocol.py            成帧、CRC、寄存器、枚举、单位换算（无 ROS、无串口依赖）
      transport_modbus.py    Modbus RTU 主站：单一所有者、有界交易、同帧重试
      transport_can.py       CAN 编解码 + 明确的拒绝路径（未做实物验证）
      board_session.py       UID 绑定、序号分配、失联对账、请求日志、只读闸门
      scheduler.py           唯一持有串口的 worker 线程：关节锁、STOP 优先、UNKNOWN 停机
      config.py              YAML 加载与校验
      fake_board.py          ADU 级模拟板卡 + pyserial 形状桩，可注入故障
      motor_node.py          ROS 2 适配层
    test/                    离线单测（不需要硬件，也不需要 ROS 运行时）
  robot_motor_bringup/       launch 与 config/robot.yaml
```

## 2. 与任务书的偏差（均因实测环境而必须调整）

| 项 | 任务书 | 本实现 | 原因 |
|---|---|---|---|
| 构建类型 | `robot_motor_driver`/`bringup` 用 `ament_python` | 三个包一律 `ament_cmake`（驱动用 `ament_cmake_python` 安装 Python 包，用 `ament_cmake_pytest` 跑测试） | 该机 Humble 是 colcon 源码安装，`ros2 pkg prefix ament_python` → MISSING，且 `dpkg -l \| grep -c ros-humble` = 0，无 apt 包可补 |
| 工作区位置 | `~/robot_motor_ros2_ws` | 实体在 `/data/ros2/robot_motor_ros2_ws`，`~/robot_motor_ros2_ws` 是指向它的符号链接 | `/`（含 `/home`）已用 97%，仅剩约 500 MB；`/data` 剩 19 GB。任务书里的命令原样可用 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 运行时探测：有 `diagnostic_msgs` 就发标准话题，没有则退化为 `/robot/diagnostics_json`（`std_msgs/String`）并在日志里点名缺项 | 该机 `diagnostic_msgs` 未安装；不擅自装系统包 |
| DDS 域 | 未指定 | launch 默认导出 `ROS_DOMAIN_ID=71` | 该机 domain 0 上已有在跑的 `/controller_manager`、`/joint_state_broadcaster`、多个 `robot_state_publisher`；加入同域会把位置喂给别人的控制器 |
| `/joint_states` | 发布新鲜有效位置 | 接口保留，但 `publish_joint_states` 缺省 `false` | 同上；且只在 `position_valid=true` 且数据新鲜时才发布，`effort` 永远留空 |
| 未知命令状态出口 | "人工处置" | 额外提供 `/robot/acknowledge_unknown`（`std_srvs/Trigger`） | 需要一个明确的人工确认入口；确认后**不自动续跑任何动作** |

审查旧工程的完整结论见 [compatibility_audit.md](compatibility_audit.md)；协议逐条出处见 [protocol_contract.md](protocol_contract.md)。旧工作区 `/opt/ros/motor_control_ws`（= `/data/ros2/motor_control_ws`，同一 inode）**未被修改**。

## 3. 构建与测试（远程实际命令）

```bash
ssh xinhua@192.168.1.113
source /opt/ros/humble/setup.bash
export PATH=$HOME/.local/bin:$PATH          # colcon 装在用户目录
cd ~/robot_motor_ros2_ws
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

只跑离线协议层（不需要 ROS 环境，任何机器都行）：

```bash
cd ros2/robot_motor_driver
python3 -m pytest test/test_protocol.py test/test_transport.py \
    test/test_board_session.py test/test_fake_board.py test/test_scheduler.py -q
```

## 4. 运行

模拟板卡（不碰任何串口，可安全跑动作流程）：

```bash
ros2 launch robot_motor_bringup bringup.launch.py transport:=fake allow_motion:=false
```

只读连实板（任务书 8C 的验收命令）：

```bash
ros2 launch robot_motor_bringup bringup.launch.py \
  transport:=modbus_rtu allow_motion:=false \
  config_file:=$HOME/robot_motor_ros2_ws/config/robot.yaml
```

维护模式（允许 STOP / 清故障 / 设零 / 刹车 / 配置，仍然拒绝运动）：

```bash
ros2 launch robot_motor_bringup bringup.launch.py transport:=modbus_rtu maintenance:=true
```

运动（必须先确认机械空间、断电方式与限位；`allow_unreferenced_relative` 控制是否允许 `valid=0` 下的相对运动）：

```bash
ros2 launch robot_motor_bringup bringup.launch.py transport:=modbus_rtu \
  allow_motion:=true allow_unreferenced_relative:=true
```

`fake` 后端的动作示例（任务书 8D，**不得**照抄到真实后端）：

```bash
source install/setup.bash
# motor_rpm 必须填 0（＝用板上已保存的速度）或与板上一致的值；填 400 而板上是 8000 会被拒（111）。
# limits_rad 为 null 时还必须 commissioning:=true，否则会被拒（112）。
ros2 action send_goal /robot/move_joint robot_motor_interfaces/action/MoveJoint \
  "{joint_name: 'left_forearm_joint', mode: 0, angle_rad: 0.1745329252, motor_rpm: 0, timeout_sec: 30.0}" \
  --feedback
ros2 topic echo /robot/motor_states --once
```

上面这条代表相对 +10°，`timeout_sec` 是监控期限，不是"30 秒到位"的承诺。注意 `timeout_sec` 还必须**大于板上自己的超时**（`robot.c:463` 的公式含 10 s 下限），否则驱动会先判超时转 UNKNOWN，而板子其实还在正常走。

## 4b. 多板（左右两块 GD32 板）

**一块板一个节点进程**，用 `board_name` 参数从同一份多板 YAML 里选板，用 namespace 区分接口名：

```bash
# 板 A（左臂+头部）：只读起步
ros2 launch robot_motor_bringup bringup.launch.py transport:=modbus_rtu \
  config_file:=/path/two_boards.yaml board_name:=main_board board_namespace:=board_a \
  ros_domain_id:=72

# 板 B（右臂）：独立进程、独立策略，可以只读而板 A 在动
ros2 launch robot_motor_bringup bringup.launch.py transport:=modbus_rtu \
  config_file:=/path/two_boards.yaml board_name:=right_board board_namespace:=board_b \
  ros_domain_id:=72
```

或用 `dual_board.launch.py` 一次拉起两块板（`board_b_start` 默认 `false`，两侧 `allow_motion` 默认 `false`）：

```bash
ros2 launch robot_motor_bringup dual_board.launch.py \
  config_file:=/path/two_boards.yaml board_b_start:=true ros_domain_id:=72
```

**为什么是"一板一进程"而不是"一个节点驱动两块板"**：进程隔离是结构性的，不靠代码自律——每块板有自己的串口所有权线程、序号分配器与去重缓存、自己的 UNKNOWN 停机；一块板卡死、掉线或进入 UNKNOWN，不会妨碍另一块被轮询和被停机。而且每块板可以有**自己的策略**（板 B 保持只读、板 A 允许运动），节点级参数做不到这一点。一个进程里的崩溃也不会带走另一条臂。

命名规则：节点用**相对名**声明接口（`robot/stop`），所以 namespace 决定最终名字。

| 场景 | 服务名 | 话题名 |
|---|---|---|
| 单板、无 namespace（历史行为，完全兼容） | `/robot/stop` | `/robot/motor_states` |
| `board_namespace:=board_b` | `/board_b/robot/stop` | `/board_b/robot/motor_states` |

`/diagnostics` **始终共享**（多发布者本来就是它的用法），每块板发自己的 `DiagnosticStatus`，`hardware_id` 是 UID、`name` 是 `robot_motor_driver/<board_name>`。`motor_states_topic`/`joint_states_topic` 若填**绝对名**就等于放弃 namespace 隔离，两块板会撞在一起——多板时不要填绝对名。

配置约束（`config.py` 强制）：两块板的 `uid` 必须不同（UID 是把每次写入绑定到正确硬件的唯一手段）；两块板的 `port` 必须不同（同一串口被两个 transport 打开会导致帧交错，双方都会把对方的回包当成自己的）；`address` **允许**相同（本机就是两块板都为 1，各走独立串口）。

必须知道的语义边界：
- **没有跨板原子停机**。`robot_stop_all()` 只作用于收到该帧的那一块板（`robot.c:144-148`）。"全部停下"＝分别调 `/board_a/robot/stop` 和 `/board_b/robot/stop`，并**分别**确认两个 `stop_confirmed`。任何一块没确认，就不得声称"已停"。
- **没有跨板同步**。两个独立 worker、两条独立串口，不保证同步轨迹，也不保证同时到位。
- **没有聚合成功**。一块板失败、另一块成功＝部分失败，必须如实分开报告。
- **关节不会跨板路由**。发给板 B 节点、却写着板 A 关节名的目标会被拒，消息里指明该关节属于哪块板。这一点很关键：早先的实现是从全局关节表解析名字的，那样会把板 A 的关节名解析成通道号、再驱动到板 B 的同号通道上——**动的是另一块板上的另一个关节，而结果看起来是成功的**。

## 5. 接口

| 名称 | 类型 | 说明 |
|---|---|---|
| `/robot/move_joint` | `MoveJoint` Action | 相对/绝对离散角度；绝对需 `position_valid=true`；`motor_rpm` 是对板上速度的**断言**不是指令 |
| `/robot/selftest` | `RunSelfTest` Action | 单路或整板；**会让电机短暂转动**，只由显式请求触发。整板是**逐路串行**，四路全部结束才返回，`channels[]` 逐路给出结果 |
| `/robot/stop` | `Stop` Service | `joint_name` 空＝整板；分别报告 `accepted` 与 `stop_confirmed`；UNKNOWN 停机期间**仍然可用** |
| `/robot/clear_fault` | `ClearFault` Service | 显式确认；不恢复 `position_valid`，不自动续跑 |
| `/robot/set_zero` | `SetZero` Service | 把当前机械姿态设为零点并写 Flash，只由人工触发 |
| `/robot/set_brake` | `SetBrake` Service | 独立抱闸/释放 |
| `/robot/apply_config` | `ApplyConfig` Service | 事务：暂存→回读校验→一次 CONFIG→再回读生效值。`channels` 为空＝**保持板上现值**（不是 YAML 值）；无变化则不发 CONFIG，`changed=false` |
| `/robot/acknowledge_unknown` | `std_srvs/Trigger` | 人工确认 UNKNOWN 状态；不自动恢复任何动作 |
| `/robot/board_identity` | `std_srvs/Trigger` | 回报 UID、命令协议版本与身份错误 |
| `/robot/motor_states` | `MotorStates` | UID、四通道原始寄存器、保存标志、新鲜度、只读/UNKNOWN 标志、`commissioning`/`soft_limits_enforced` |
| `/joint_states` | `sensor_msgs/JointState` | 默认关闭；只发新鲜且 `valid` 的弧度位置，`effort` 恒空 |
| `/diagnostics` 或 `/robot/diagnostics_json` | `DiagnosticArray` / `String` | 通信、故障、配置与 YAML 不一致、存储状态 |

`MotorState` 同时给出原始值（`angle_deci_deg`、`fg_count`、`duty_permille`）与换算值（`angle_rad`）。`duty_permille` 是**低电平千分比**（固件上限 950），不是百分比；`measured_rpm` 是**无符号幅值**，单线 FG 测不出真实方向，符号只能来自命令。`position_valid=false` 时 `angle_rad` 被强制置 0，消费者必须读 `position_valid` 而不是把角度当姿态。

## 6. 结果码

`error_code` / `result_code` 是同一个 uint16 空间：**0～6 是固件业务结果**，**≥100 是驱动自身判定**，两者不会混淆。

| 码 | 含义 |
|---:|---|
| 0 | 成功 |
| 1 | 参数非法（VALUE） |
| 2 | 忙（BUSY）：整板或本通道未静止、运动中、自检中、急停输入有效 |
| 3 | 未就绪（UNREADY）：未使能、gear=0、有故障、整板停止互锁 |
| 4 | 存储（STORAGE）：Flash 写失败，MOVE 与 SELFTEST 被阻断 |
| 5 | 序号（SEQUENCE）：重放参数不符或超出半区间窗口 |
| 6 | 地址（ADDRESS） |
| 100 | 只读模式拒绝（含 STOP） |
| 101 | 身份未确认（UID 或命令协议版本不符） |
| 102 | 结果 UNKNOWN，已停止调度 |
| 103 | 无应答且对账失败 |
| 104 | 绝对动作需要有效零点 / 未启用无零点相对运动 |
| 105 | 超出配置限位或单次 ±360° 限制 |
| 106 | NO_OP：目标在该减速比下量化为 0 步，未发送任何帧 |
| 107 | 同一路已有动作在进行 |
| 108 | 暂存配置回读不一致，CONFIG 未发送 |
| 109 | UNKNOWN 之后未确认，驱动处于停机（**STOP/STOP_ALL 例外，仍可发送**） |
| 110 | 未连接 |
| 111 | 目标 `motor_rpm` 与板上该路已保存的 rpm 不一致，未发送任何帧 |
| 112 | 机械限位未确认（`limits_rad: null`）或位置无效，无法核对终点；需 `commissioning:=true` |
| 113 | 读不到新鲜位置，拒绝用可能过期的数据计算位移 |
| 114 | 请求本身不合法（内部），未发送任何帧 |
| 120 | 目标被拒绝 |
| 121 | 目标字段非法（NaN/Inf、未知关节、模式、rpm、超时） |

注意 Modbus 异常码比业务码粗：`0x03` 同时覆盖 VALUE 与 SEQUENCE，`0x04` 同时覆盖 UNREADY 与 STORAGE。驱动的判据是 `0x0036` 是否等于本次序号——不变＝序号被拒（未执行），相等＝已执行但被拒，此时 `0x0037` 就是准确结果。

### `motor_rpm` 不是速度指令

固件的速度只来自板上每路已保存的 rpm（`BSP/robot.c:334`），MOVE 帧里只有角度，**没有任何字段能改速度**。所以 `MoveJoint.goal.motor_rpm` 的语义是"我断言板上就是这个名字速度"：

- `0` ＝ 不断言，用板上的值；
- 非 0 且与板上值**不一致** → 拒绝，`error_code=111`，一帧都不发（以前是只写日志然后按板上速度跑，调用方会误以为速度生效了）；
- 真要改速度：整板静止后走 `/robot/apply_config`（写 Flash），再把 YAML 的 `expected_motor_rpm` 改成同一个值。

YAML 里的字段因此叫 `expected_motor_rpm`（旧名 `default_motor_rpm` 仍可解析，作为兼容别名）。启动时只**比对**并写进诊断，绝不自动写板。

### 限位未确认时的行为

`limits_rad: null` 表示机械限位**没有确认**，驱动不会假装有：

- 默认**拒绝**该关节的任何运动，`error_code=112`，消息里说明怎么办；
- `commissioning:=true` 才放行，且每条命令被 `max_commissioning_step_deg`（默认 10°）封顶；此时日志、`/robot/motor_states`（`commissioning=true`、`soft_limits_enforced=false`）和诊断里都会写明"没有软限位保护"；
- 限位已配置时，**相对运动也会核对预计终点**（当前位置＋位移）是否落在 `limits_rad` 内；
- `position_valid=false` 时终点无法核对，驱动不会声称核对过——要么先设零点，要么显式进 commissioning。

### 现场会撞到的两件事

- **`busy(2)` 往往只是"整板还没静止 500 ms"**。MOVE / ZERO / CONFIG / SAVE / CLEAR_FAULT / DIRECTION / SELFTEST 都要求**整板**（不是本通道）连续 500 ms 没有任何 FG 边沿（`BSP/robot.c:167,382,387,420,424,430`）。上一段运动刚结束、或有人手转了别的关节，都会让下一条命令吃到 busy。驱动**不自动重试、不自动等待**：它把 busy 原样报出来，由调用方决定何时重发。
- **`allow_motion` 与 `allow_unreferenced_relative` 有两个来源**：YAML 的 `driver:` 段提供默认值，launch 参数（或程序化 override）覆盖它。两者都缺省为 `false`，所以不存在"因为 YAML 写了 true 就意外能动"的情况；反过来，只在 YAML 里写 true 而 launch 显式传 false，则以 false 为准。

## 7. 安全规则（代码里强制，不只是文档）

- 启动、退出、重连都不发送自检或运动；退出时的 STOP 只在 `allow_motion=true` 时尝试，且未确认就在日志里明说。
- 绝不发送 `A55A` 旧提交帧，绝不写 `0x0035`，绝不用序号 0。
- MOVE 超时**不换新序号重发**；只按原序号原字节重发，或转为 UNKNOWN 停机。
- **UNKNOWN 停机只拦新动作，不拦停机**：`STOP` / `STOP_ALL` 仍可发送（`board_session.STOP_OPCODES`），其余写命令一律拒绝；`BRAKE(1)` 虽然固件里等效于停，也**不在**放行名单里。停机成功**不会**自动清 UNKNOWN，必须人工 `/robot/acknowledge_unknown`。帧发出去 ≠ 已停：没有应答或没有确认静止，一律报 `unknown`、`stop_confirmed=false`。停机期间的**读**（poll）也保持放行，否则连"停没停"都无法确认。
- 不自动清故障、不自动设零、不自动重连续跑。
- CONFIG 要求整板静止且写 Flash，只在配置确实变化时提交，提交前核对整个暂存区，提交后回读。`channels` 为空时以**板上生效值**为基线，不会用 YAML 默认值覆盖角色/减速比/rpm。校验规则逐条对齐 `robot_config_valid`（`BSP/robot.c:11-23`），包括**禁用通道的非零 role 也参与唯一性检查**、rpm/gear 范围对禁用通道同样生效。
- 每条串口只有一个所有者线程；ROS 回调不直接碰串口。**绝对运动的"读当前位置→算位移→发 MOVE"整体在 worker 线程内完成**，因此两个目标不可能基于同一个起始位置各自算一遍位移。
- 位置读不到就**拒绝**运动（`113`），绝不退回用旧快照算位移；`refresh_now()` 失败时返回 `None` 而不是上一份快照。
- 角度换算是 `round(rad*1800/pi)`（正负对称、远离零），**不再乘减速比**；单次 MOVE 限 ±360°，超限拒绝而不是拆分。
- `valid=0` 时不伪造绝对位置；软件急停不能替代独立硬件急停（默认构建 `BUTTON_ESTOP_ENABLE=0`，板上急停输入恒为 0）。
- 整板自检是**逐路串行**的（`BSP/robot.c:294-296`），驱动等到"没有任何通道处于 queued/running/settling 且没有电机在 S_SELFTEST"才算结束。被丢弃的后续通道读回是 `UNTESTED`（`BSP/robot.c:102-106`），**不是**终态也**不是**通过；结果里 `executed`/`batch_completed`/`all_passed` 分开报告。"有 FG 响应"只证明抓到了脉冲，不证明接线、方向或准确回原位。

## 8. 部署注意事项（本轮实际踩到的坑）

- **入口脚本的可执行位**：`colcon build --symlink-install` 把 `lib/robot_motor_driver/robot_motor_node` 直接符号链接到**源文件**，而 `launch_ros` 用 `os.access(X_OK)` 过滤 libexec 目录。从 Windows 打包同步过去的文件没有可执行位，于是 `ros2 launch` 报 `executable 'robot_motor_node' not found`，而文件明明在那儿。`robot_motor_driver/CMakeLists.txt` 因此每次构建都 `chmod +x` 源文件（用 `add_custom_target(... ALL)`，不是只在 configure 期跑一次的 `execute_process`——重新同步源码后 CMake 不一定重配置，那样会悄悄失效）。手工解压源码后若不重新构建，需要自己 `chmod +x src/robot_motor_driver/scripts/robot_motor_node`。
- **`set -u` 与 colcon 环境脚本冲突**：`/opt/ros/humble/setup.bash` 引用未定义的 `COLCON_TRACE`，脚本里带 `set -u` 会在 source 时直接失败。source 前后要 `set +u` / `set -u`。
- **不要用裸 `ttyUSBn`**：同一物理口在本次会话内先后是 ttyUSB2 → ttyUSB1 → ttyUSB0。一律用 `/dev/serial/by-path/platform-fc800000.usb-usb-0:1.2:1.0-port0`，并在连接后核对 UID。
- **`ROS_DOMAIN_ID`**：该机 domain 0 上有在跑的实机控制栈，节点测试也强制用 71，避免测试流量进入实机图（曾观察到 node 测试在 domain 0 下被拖崩）。
- **轮询延迟：必须按需精确读取**。pyserial 的 `read(n)` 只有在**读满 n 字节**时才提前返回，否则等满 `timeout`。用 `read(64)` 读一个 9～37 字节的应答，每帧固定耗时 50.5 ms（实测），一轮 18 次交易 911 ms，5 Hz 轮询必然报 `data stale`。`inter_byte_timeout` 实测**无效**；改成先读 2～3 字节定出应答长度、再只读剩余字节后，实测降到 **5.6～8.1 ms/帧**，一轮约 126 ms（约 7.9 Hz），实板 `age_sec` 从 1.19 s 降到 0.30 s、`stale` 变为 false。
- **`/` 分区 97%～98% 满**：工作区、构建产物、证据文件都放 `/data`；`/tmp` 与 `/home` 都在快满的根分区上，`~/.ros/log` 已累积 24 MB / 6000+ 目录，长跑前值得清理。

## 9. 日志与回滚

- 构建日志：`~/robot_motor_ros2_ws/build.log`、`log/latest_build/`；测试日志：`test.log`、`log/latest_test/`。
- 运行时日志：`ros2 launch` 的前台输出（节点所有拒绝、对账、UNKNOWN 都写在这里）；`~/.ros/log/` 下有 rclpy 日志。
- 回滚：本软件不改动系统、不改动旧工作区、不改动固件。
  ```bash
  rm -f ~/robot_motor_ros2_ws                       # 只删符号链接
  rm -rf /data/ros2/robot_motor_ros2_ws             # 删实体工作区
  rm -rf /data/ros2/dsh_iface_probe                 # 删本轮的 action 生成探测包
  ```
  旧栈照原样可用：`source /opt/ros/humble/setup.bash && source /opt/ros/motor_control_ws/install/setup.bash`。

## 10. 本轮未验证 / 未实现

- **CAN 未做实物验证**，且该机没装 python-can；`CanTransport.open()` 默认抛 `TransportNotAvailable` 而不是假装成功。编解码有离线单测。
- **实板运动未执行**：本轮只对实板做过只读探测（UID `C23B378115B092083830364A`、`0x000F=2`、四路 `fault=0`）。阶段 E 需要用户授权与机械准备。
- **右臂未投入运行**：板 B 的 UID（`C23B37C715B092083830364A`）与 by-path 口已只读核对，映射写在 `config/robot_right_arm_UNCONFIRMED_GEARS.yaml`，但**三路减速比未经实物确认**（板上现存 516/516/0/516，与该文件按左臂推定的 516/290/82 不一致），`inverted` 与机械限位也未测。该文件只能 `allow_motion:=false` 只读用。用户已口头确认右臂的通道→关节映射与减速比"由人核对过"，但**具体数值尚未提供**，因此该文件里的 gear 仍是推定值，不得据此运动。
- **多板：已实现"一板一进程"，未实现"单进程驱动多板"**。`_select_board()` 现在支持多板 YAML，用 `board_name` 选板（板数 >1 且未指定 `board_name` 时明确拒绝，不猜），接口改为相对名以支持 namespace 隔离，并有 `dual_board.launch.py`。**没有**跨板原子停机、跨板同步或聚合成功（见 4b）。两块 fake 板的隔离性有单测；**两块实板同时运行从未测过**。
- **随多板一起修掉的一个隐患**：`_resolve_joint()` 原先从全局关节表解析名字，多板配置下会把板 A 的关节名解析成通道号再驱动到板 B 的同号通道——动的是另一块板上的另一个关节，而结果看起来是成功的。现在只在本板关节表内解析，跨板关节名会被拒并指明归属。
- 各关节机械限位未确认，YAML 里是 `limits_rad: null`。**这不再是"没有软限位保护"，而是默认拒绝运动**（`error_code=112`）；只有 `commissioning:=true` 才放行，且单条命令被 `max_commissioning_step_deg`（默认 10°）封顶，同时在日志、`/robot/motor_states` 和诊断里标明无软限位保护。
- 左手腕历史 RPM 异常（设定 1500 读到约 5737/5847）根因未查明，未做无人值守动作序列。
- 未实现同步轨迹、连续目标更新、逆运动学，也不承诺固定到位时间。
- 本轮五项修复（UNKNOWN 下可停机、整板自检逐路结果、`motor_rpm` 断言、`apply_config` 空列表语义、绝对运动新鲜度与软限位）**只在模拟板上验证**，未连真实电机、未烧录固件；需要在实板上复验的步骤见 `docs/Robot_自检验证记录.md`。
- **板 A 的 `storage_error=true` 未解决**，而 `robot.c:449,168` 让 MOVE 与 SELFTEST 在该位为真时一律返回 `R_STORAGE(4)`，所以板 A 现在物理上无法运动或自检；只读手段无法确定成因（`robot_store.c` 的 5 个失败出口都不通过 Modbus 暴露原因），须先做一次授权写入才能区分。
