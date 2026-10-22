# 历史参考：ROS2_左臂验证与右臂绑定_测试方案

> 保留当时方案、观测及验证结果，不是现行操作步骤。历史“当前”仅指记录当天；请使用 [文档索引](../../README.md)。原报文与测量值不改写。

> 2026-10-08 故障4规则更新：按已生效减速比计算累计非预期活动量，≤10°不报警、>10°报警；516/290/82分别在第87/49/14个脉冲触发，未配置有效减速比不触发。本文原固定10/11脉冲说明属于旧版，当前规则、运行逻辑和验证见[固件功能与运行逻辑](../Robot_固件功能与运行逻辑.md)。

> 当前日常操作请使用 [ROS2度数接口使用说明](../ROS2_使用与测试手册.md#degrees-interface)：angle_deg直接输入度数，motor_states_deg直接读度数。本文原弧度命令保留为兼容或历史参考，不改写已有实机日志。


# 左臂验证与右臂绑定测试方案

编制日期：2026-10-08。适用对象：`ros2/robot_motor_driver` + `robot_motor_bringup`（本仓库 `ros2/`，副本在 `../GD32F303VET6_MotorControl_ROS2/`）。
依据：[ROS 2 代码生成执行方案](ROS2_头部与双臂控制_代码生成执行方案.md) 第 8 节阶段 E/F、[协议契约](protocol_contract.md)、[兼容审查](compatibility_audit.md)。

本文是**待执行方案**，不是执行记录。除阶段 0 的只读探测外，本文生成过程中没有发送任何写命令、没有运动、没有清故障、没有设零、没有烧录。

## 0. 阶段 0 只读基线（已执行，2026-10-08 09:14）

| 项 | 板 A（左臂+头） | 板 B（右臂板，新接） |
|---|---|---|
| by-path | `platform-fc800000.usb-usb-0:1.2:1.0-port0`（ttyUSB0） | `platform-fc800000.usb-usb-0:1.4:1.0-port0`（ttyUSB2） |
| 芯片 | 1a86:7523 HL-340（ch341.ko 自编译） | 1a86:7523 HL-340 |
| UID | `C23B378115B092083830364A` | `C23B37C715B092083830364A` |
| schema / cmdver / addr / node / pwm | 1 / **2** / 1 / 1 / 25000 | 1 / **2** / 1 / 1 / 25000 |
| dirty / storage_error / pending_save | 0 / 0 / 0 | 0 / 0 / 0 |
| last_sequence / last_result | 178 / 0 | 28 / 0 |
| M1 | en=1 role=2(left_upper) gear=516 inv=0 rpm=8000；state0 fault0 valid0；0.0°/fg0 | **en=0 role=0** gear=516；state0 fault0 valid0；42.7°/fg368 |
| M2 | en=1 role=1(head) gear=516；**state6 fault4(unexpected_fg)** valid0；107.9°/fg928 | **en=0 role=0** gear=516；state0 fault0 valid0；20.3°/fg175 |
| M3 | en=1 role=4(left_forearm) gear=290；state1(IDLE) fault0 **valid1**；0.0°/fg0 | **en=0 role=0 gear=0**；state0 fault0 valid0 |
| M4 | en=1 role=6(left_palm) gear=82；**state6 fault4(unexpected_fg)** valid0；33.6°/fg46 | en=1 **role=2(left_upper)** gear=516；state0 fault0 valid0；20.3°/fg175 |

ttyUSB1（CP2102，`/dev/door_lock_uart`，被 `gateway.kafka_server` 占用）**与本任务无关，全程不碰**。

### 阶段 0 得出的四条硬约束

1. 板 A 的 M2/M4 处于 `fault 4`，而 `last_sequence` 自 2026-09-29 起仍是 178 → 故障不是命令造成的，是累计非预期 FG 边沿（`BSP/robot.c:31-37`，预算 10，第 11 个触发）。清故障前这两路必被 `R_UNREADY` 拒动，且**任一路有 fault 会让整板 SELFTEST 被拒**（`robot.c:170-171`）。
2. 板 B 未按右臂配置：3 路 disabled、M3 `gear=0`（必拒动）、M4 role 仍是 2(left_upper) 属上次本地测试残留。右臂角色应为 3(right_upper)/5(right_forearm)/7(right_palm)。
3. 两个 CH340 无 USB 序列号，`/dev/serial/by-id/` 只有一条 `usb-1a86_USB_Serial-if00-port0`（指向 ttyUSB2）→ **by-id 无法区分两板**，必须 by-path + 每次连接核对 UID。换 USB 口会改 by-path。
4. 两板 addr 都是 1，但各在独立串口（独立总线），**不冲突**；不得把两个 CH340 TTL 并到同一条 RS-485 上当多站（任务书 8.F）。
5. 当前驱动 `_select_board()` 在 `len(boards) != 1` 时抛 ConfigError → **绑定右臂前必须先做多板支持**（阶段 2）。

## 1. 授权与安全前提（每次实物步骤都要满足）

- 默认构建 `BUTTON_ESTOP_ENABLE = 0`（`BSP/board_config.h:111`），板上**没有硬件急停输入**，`stop_input` 恒 0。软件 STOP 不是安全功能 → 现场必须有**可立即断电**的手段，且操作者知道怎么用。
- 每次运动前确认：机械活动空间内无人无遮挡、负载已按实际状态固定、单次角度 ≤ 5°（首轮）、RPM 用 400（最低档）、只动一路。
- 驱动侧一律 `allow_motion:=false` 起步，逐项授权后才改 `true`；维护动作走 `maintenance:=true`（允许 STOP/清故障/设零/刹车/配置，仍拒绝运动）。
- 不自动清故障、不自动设零、不自动自检、不自动重试；`UNKNOWN` 后必须人工 `/robot/acknowledge_unknown` 才恢复调度。
- 左臂**机械正方向仍未解决**（见 [OPEN_QUESTIONS](../ai_context/OPEN_QUESTIONS.md)：M1/head 用户报告 +10° 与 −10° 都向后，PE15 与电机端方向线电压未同时测量）。因此本轮左臂试转的目的之一是**采集方向事实**，不得预设方向正确。
- 左手腕历史 RPM 异常（设定 1500 读到约 5737/5847）根因未明 → 不启用无人值守动作序列，不宣称恒速。

## 2. 阶段 1：左臂（板 A）验证

顺序原则：先无故障且已有零点的一路，再处理有故障的两路；每步都要读回判据，任何一步不满足就停在该步。

### 1.1 只读基线复采（无需授权）

```bash
ros2 launch robot_motor_bringup bringup.launch.py transport:=modbus_rtu allow_motion:=false
ros2 topic echo /robot/motor_states --once
ros2 service call /robot/board_identity std_srvs/srv/Trigger "{}"
```

判据：`uid == C23B378115B092083830364A`、`command_protocol == 2`、`identity_ok=true`、`communication_ok=true`、`stale=false`、节点日志 `TX ` 计数为 0。**UID 不符立即停**，不写任何东西。

### 1.2 清 M2/M4 的 fault 4（需授权，写操作）

前置：`maintenance:=true`（仍 `allow_motion:=false`）。逐路执行，不要批量：

```bash
ros2 service call /robot/clear_fault robot_motor_interfaces/srv/ClearFault "{joint_name: 'left_wrist_joint'}"
ros2 service call /robot/clear_fault robot_motor_interfaces/srv/ClearFault "{joint_name: 'head_joint'}"
```

判据与分支：
- `accepted=true`、`fault_after=0` → 通过；**静置 60 s 后重新只读轮询**，若 fault 复发为 4 → 判定为 FG 线干扰/接线问题，**停止本阶段**，转硬件排查（此时不是"手转造成"）。
- `fault_after=0` 但 `position_valid=false` → 正常：`CLEAR_FAULT` 不恢复零点（`robot.c:427`），需要 1.5 步显式设零才有绝对参考。
- 返回 `busy(2)` → 整板未静止 500 ms（`robot.c:424`），等待后重试一次；仍 busy 则记录并停止。

### 1.3 逐路小角度相对试转（需授权，会运动）

前置：1.2 通过、`allow_motion:=true`、`allow_unreferenced_relative:=true`（valid=0 下只做相对运动）、单次 ±5°。

> **2026-10-08 修正（驱动行为变更）**：本节的启动参数与目标字段与旧版不同，原因是修复了"软限位未确认却照常运动"和"`motor_rpm` 只写日志不生效"两个缺陷：
>
> 1. 四路都是 `limits_rad: null`（机械限位未确认），驱动现在**默认拒绝**这类关节的任何运动，返回 `error_code=112`。必须显式加 **`commissioning:=true`**；此时每条命令被 `max_commissioning_step_deg`（默认 10°，≥ 本节 5°）封顶，且日志、`/robot/motor_states`（`commissioning=true`、`soft_limits_enforced=false`）和诊断都会写明**没有软限位保护**。
> 2. `motor_rpm` 不是速度指令。固件只从板上已保存的每路 rpm 取速度（`BSP/robot.c:334`），MOVE 帧里没有速度字段。板上四路实测 rpm=**8000**，所以目标里写 `motor_rpm: 400` 会被**拒绝**（`error_code=111`，一帧不发）。本节一律写 **`motor_rpm: 0`**（＝不断言，用板上的值）。真要跑 400 rpm，必须先整板静止后用 `/robot/apply_config` 写 Flash，再把 YAML 的 `expected_motor_rpm` 改成 400 —— 这会改变所有四路的速度，属于独立授权项。
> 3. 因此"`measured_rpm` 明显高于 400"这条判据改为：**`measured_rpm` 应接近 8000**（板上现值）；若明显偏离 8000（例如左手腕历史问题里的 ~5737/5847），记录并停止该路，转测速排查。

启动命令（注意 `commissioning:=true`）：

```bash
ros2 launch robot_motor_bringup bringup.launch.py transport:=modbus_rtu \
  allow_motion:=true allow_unreferenced_relative:=true commissioning:=true
```

顺序：**M3(left_forearm, gear290, valid=1, 无故障) → M1(left_upper, gear516) → M4(left_palm, gear82) → M2(head, gear516)**。head 放最后，因为它就是方向未解决的那一路，且靠近人体。

每路规程（一次只动一路，动完读回再决定下一步）：

```bash
ros2 action send_goal /robot/move_joint robot_motor_interfaces/action/MoveJoint \
  "{joint_name: 'left_forearm_joint', mode: 0, angle_rad: 0.0872664626, motor_rpm: 0, timeout_sec: 30.0}" --feedback
ros2 topic echo /robot/motor_states --once
```

判据（全部满足才算该路通过）：
- 结果 `motion_completed=true`、`error_code=0`、`persisted=true`；结果消息里带 `no soft-limit check was possible`（如实说明限位未确认，不是错误）；
- 读回 `state=5(DONE)`、`fault=0`、`brake=true`、`duty_permille=0`、`measured_rpm=0`；
- 保存标志 `dirty=0 / storage_error=0 / pending_save=0`；
- `final_angle_rad` 与目标的差值能被 FG 量化解释（gear290 每步 3600/(290×6)=2.07 个 0.1°，gear516 每步 1.16，gear82 每步 7.32）；
- **操作者口头确认实际机械方向**，逐路记录"正角度→哪个方向"。这是本轮要采集的关键事实，不能由计数推断。
- 运动中 `measured_rpm` 明显偏离 8000 → 记录并停止该路，转测速排查（左手腕历史问题）。

然后反向 −5° 重复一次，确认往返都可控、无 fault 4 复发。

### 1.4 停止与取消验证（需授权）

- 运动进行中调 `/robot/stop`：判据 `accepted=true` 且 **`stop_confirmed=true`**（驱动只在板确认 state 不在 2/3/4/7 且 duty=0 时才置 true）；`unknown=true` 时按"停止未确认"处理，不得当作已停。
- 通过 `ros2 action send_goal` 的取消（Ctrl-C）触发 cancel：判据结果消息含 `cancelled`，且 `stop_confirmed` 如实反映。
- 超时路径：给一个 `timeout_sec` 小于实际所需的目标（例如 360° + 5 s），判据 `error_code=103`、`unknown=true`、`/robot/motor_states.unknown_command_state=true`，且**`last_sequence` 只增加 1**（证明没有换新序号重发）；随后 `/robot/acknowledge_unknown` 恢复调度。

### 1.5 设零（可选，需授权，写 Flash）

只在机械参考姿态已确定、且该路静止时执行：

```bash
ros2 service call /robot/set_zero robot_motor_interfaces/srv/SetZero "{joint_name: 'left_forearm_joint'}"
```

判据：`accepted=true`、`position_valid=true`、`persisted=true`、随后只读复核 `state=1(IDLE)`、`fg_count=0`、`angle_deci_deg=0`。设零后再跑一次绝对运动（`mode: 1`）验证绝对路径：判据 `position_valid=true` 且 `final_angle_rad` 落在量化容差内。

> **2026-10-08 修正**：绝对运动现在由 worker 线程"读当前位置→算位移→核对限位→发 MOVE"一气呵成，不再用发布快照算位移（避免两个目标基于同一起点各算一遍）。因为 `limits_rad` 仍是 null，绝对运动同样需要 `commissioning:=true`，且单条命令受 `max_commissioning_step_deg` 封顶——所以绝对目标与当前位置之差也要 ≤ 10°（默认值），否则返回 `error_code=105`。设零后当前位置为 0，绝对目标取 ≤10° 即可。
> 若已经记录了机械限位（`limits_rad` 非 null）且 `position_valid=true`，则**相对运动也会核对预计终点**，越界返回 `error_code=105`，一帧不发。

### 1.6 阶段 1 出口判据

四路各自：无 fault、往返 ±5° 可控、方向已由人确认并记录、保存标志全 0；停止/取消/超时三条路径都有实测回包；`unknown_command_state` 最终为 false。**任何一路未达标就只记录该路结论，不推进到阶段 3。**

## 3. 阶段 2：驱动多板支持（纯软件，无硬件风险，可与阶段 1 并行）

当前实现只驱动一块板，需要做的改动（都不涉及实板）：

1. `motor_node._select_board()` 改为构建 `{board_name: (transport, session, worker)}`，每板一条串口、**各自一个 worker 线程**（保持"每条串口唯一所有者"）；`/robot/motor_states` 按板发布，用 `header.frame_id=board_name` + `uid` 区分（消息定义不变，避免破坏既有接口）。
2. 关节→板的路由已有（`JointConfig.board`），动作/服务按 `joint_name` 找到对应板的 session；`/robot/stop` 空 `joint_name` 时的语义要明确：**逐板各发一次 STOP_ALL**，并分别报告 `stop_confirmed`，不宣称原子。
3. 身份绑定按板独立：任一板 UID 不符只让**该板**进入拒绝写状态，另一板不受影响；同理 `UNKNOWN` 停机是**按板**的，不得一板失联就冻住另一板。
4. 配置事务按板独立：`ApplyConfig` 需要带板标识（新增 `board` 字段或用 `ChannelConfig[]` 里的板名），避免把两板的 staged 混在一起。
5. fake 后端要能同时挂两块板（不同 UID），新增单测：多板身份混淆拒绝、单板失联不影响另一板、两板同时运动互不干扰、跨板启动非原子时一板失败要停另一板并逐一核实、`by-id` 不可用时必须靠 UID 区分。
6. 验收：现有 **356 项测试保持全绿**，新增多板测试全绿；`transport:=fake` 下跑双板动作流程；实板双板只做**只读**联测。

## 4. 阶段 3：右臂板（板 B）配置与绑定（需授权，写 Flash）

### 3.1 先补齐映射信息（阻塞项，必须由用户提供）

固件与本文都**不能凭左臂推断右臂**。需要确认：

| 待确认 | 现状 | 需要谁给 |
|---|---|---|
| 右臂各关节接在板 B 的哪个物理通道（M1..M4） | 未知；当前 M4 被上次测试配成 role=2(left_upper) | 用户按实际接线 |
| 每路的减速比 gear | M1/M2/M4=516、M3=**0**（非法，必拒动） | 用户按实物电机 |
| 每路 role（3=right_upper、5=right_forearm、7=right_palm） | 全 0(unassigned)，除 M4=2 | 用户确认关节定义 |
| 每路 inverted 与机械正方向 | 未知 | 用户实测确认 |
| 机械限位 | 未知（左臂也仍是 `limits_rad: null`） | 用户实测确认 |
| 右臂板 UID 绑定 | 已探明 `C23B37C715B092083830364A` | 已完成 |

### 3.2 只读基线 + 端口/UID 双确认（无需授权）

用 by-path `...0:1.4...` 启动单板只读，判据 `uid == C23B37C715B092083830364A`、`cmdver=2`、`TX ` 计数 0。**如果 UID 是板 A 的那个，说明接线或 by-path 映射错了，立即停**（两个 CH340 无法靠 by-id 区分，这是唯一可靠防错点）。

### 3.3 配置事务（需授权，写 Flash）

`/robot/apply_config` 是事务：先 staged 全部四路 → 回读校验 → 一次 CONFIG → 再回读生效值。

- 先用 `commit:=false` 只暂存并回读，人工核对 staged 与预期一致；
- 再 `commit:=true` 提交。前置：整板静止 500 ms（`robot.c:387`），四路 gear ∈ {0,82,290,516}、rpm 400..8000、role 唯一、enabled 的路必须有 gear 和 role；
- 判据：`accepted=true`、`result_code=0`、`persisted=true`，回读的 `applied` 与请求一致；
- 已知副作用（不是故障）：gear/role/inverted 变化的通道 **valid 会被清 0**、其自检记录被重置（`robot.c:394,402`）；`pwm_hz` 立即生效，但 **protocol/address/node 改动需重启才生效**（`robot.c:69-70,400,406`），本次不改这三项。
- 若返回 `value(1)` → staged 有非法值，**不要重复提交**，回读 staged 找出差异。
- 若返回 `busy(2)` → 整板未静止，等待后重试一次。

### 3.4 配置后只读复核 + 逐路试转

同 1.3 规程，但**首轮每路 ±5°、RPM 400、动一路读一路**，并额外记录：板 B 的 fg 残留计数（M1 368、M2/M4 175）不是零位，试转前先只读记录，试转后核对增量是否与步数一致（gear516 下 5° → `(50×516×6+1800)/3600 = 43` 步）。

### 3.5 阶段 3 出口判据

四路配置回读与请求一致、保存标志全 0、逐路 ±5° 往返可控、方向由人确认并记录、无 fault 复发；M3 的 `gear=0` 已改为实际减速比（否则该路永远拒动）。

## 5. 阶段 4：双板联合（只读为主，运动需授权）

1. 双板同时只读运行：两个 `/robot/motor_states`（按 `uid`/`frame_id` 区分），判据两板 `communication_ok=true`、`stale=false`、`age_sec` 稳定在 1/polling_hz 量级（单板实测 0.30 s；双板因各持一条串口、互不抢总线，应基本不变）。
2. 故障隔离：拔掉板 B 的 USB（或用 fake 注入失联），判据板 A 继续正常轮询、板 B 进入 `unknown_command_state=true` 且**不影响板 A**，重连后板 B 不自动续跑任何动作。
3. 跨板停止：`/robot/stop`（空 joint_name）判据两板分别报告 `stop_confirmed`，文档与响应都不宣称原子。
4. **不做**：两板同步轨迹、协调运动、跨板插补。任务书 8.F 与第 5 节都明确首版每板串行、不保证同步。

## 6. 每阶段的记录要求

每个实物步骤都要留：日期时间、板 UID、by-path 与当时的 ttyUSBn、`last_sequence` 前后值、发送的原始帧与回包、判据逐项结果、操作者对机械方向的口头确认、未达标项。记录写入 [Robot_自检验证记录](Robot_自检验证记录.md)，未执行的步骤写 `not actually run`，不得写"已通过"。

## 7. 现在需要用户决定的事

1. 阶段 1.2（清 M2/M4 的 fault 4）是否授权？是否先静置观察 60 s 看是否自发复发（用于区分"手转造成"与"FG 线干扰"）？
2. 阶段 1.3 的试转是否授权？机械空间、断电手段、单次 ±5°/RPM 400 是否接受？head 是否本轮跳过（方向问题未解决）？
3. 阶段 2 的多板改造是否现在就做（纯软件、无硬件风险，可与阶段 1 并行）？
4. 阶段 3.1 的右臂映射表（通道↔关节↔gear↔role↔inverted↔限位）由谁在何时提供？在拿到之前，右臂板只做只读，不写配置。
5. `/robot/apply_config` 改板 B 配置会清掉变化通道的 valid 并重置其自检记录，是否接受？
