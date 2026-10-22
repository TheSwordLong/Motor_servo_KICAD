# 历史参考：ROS2_双板实机使用与故障处理总结

> 保留当时方案、观测及验证结果，不是现行操作步骤。历史“当前”仅指记录当天；请使用 [文档索引](../../../GD32F303VET6_MotorControl_ROS2/README.md)。原报文与测量值不改写。

# ROS2 双板实机使用与故障处理总结

> 当前日常操作请使用 [ROS2度数接口使用说明](../ROS2_使用与测试手册.md#degrees-interface)：angle_deg直接输入度数，motor_states_deg直接读度数。本文原弧度命令保留为兼容或历史参考，不改写已有实机日志。


更新日期：2026-10-08。适用于 xinhua 当前两块板及本次测试版本。本文记录已执行操作，并提供后续命令；文中的命令示例不代表已经再次执行。

## 1. 环境、入口与当前状态

- SSH：`ssh xinhua@192.168.1.113`。密码不写入文档。
- 远程工作区：`/data/ros2/robot_motor_ros2_ws`。
- 本地 ROS2：`E:/Date_save/embedded_code/GD32Projects/GD32F303/GD32F303VET6_MotorControl_ROS2`。
- 本地固件：同级 `GD32F303VET6_MotorControl`。
- 本轮实际使用配置：[multi10_confirmed.yaml](../../../GD32F303VET6_MotorControl_ROS2/review_logs/multi10_confirmed.yaml)。它是用户确认映射后的测试配置，七个参与关节均为400 RPM。原 `src/.../robot_dual_UNCONFIRMED_RIGHT_ARM.yaml` 未被改写，不能把旧配置当作当前板上值。
- ROS_DOMAIN_ID=173；A板命名空间 `/board_a/robot`，B板 `/board_b/robot`。
- 两板均为命令协议版本2，Modbus单帧执行，无需A55A提交帧。ROS2驱动自行管理非零序号。
- 本轮使用Modbus RTU串口，CAN未实测。

**最终快照不是初始测试结果：**

| 板/通道 | 关节 | 当前记录角度 | 故障 | 位置有效 | 刹车 |
|---|---|---:|---:|---|---|
| A/M1 | 左大臂 | 10.0° | 0 | 否 | 锁定 |
| A/M2 | 头部 | -0.1° | 0 | 是 | 锁定 |
| A/M3 | 左小臂 | 9.9° | 0 | 是 | 锁定 |
| A/M4 | 左手腕 | 13.1° | 4 | 否 | 锁定 |
| B/M1 | 右大臂 | 10.3° | 0 | 是 | 锁定 |
| B/M3 | 右小臂 | 10.3° | 0 | 是 | 锁定 |
| B/M4 | 右手腕 | 0.0° | 4 | 否 | 锁定 |

位置无效时，表内角度只是保留计数，不能作为可信机械绝对位置。所有通道最终PWM占空比及反馈转速为0。

A板：dirty/storage_error/pending_save=0/0/0，ROS2只读。
B板：1/1/1；运行维护模式，允许维护写入但不允许MOVE。因此其消息 `read_only=false` 不等于已允许运动。

左/右手腕故障4是在整理文档时再次读取发现的；此前左手腕运动完成时故障为0，右手腕清故障并解除/锁定BRAKE时命令均成功。没有足够证据判定此次非预期FG的物理来源，未自动清除。

证据：[A板最终状态](../../../GD32F303VET6_MotorControl_ROS2/review_logs/usage_final_board_a.yaml)、[B板最终状态](../../../GD32F303VET6_MotorControl_ROS2/review_logs/usage_final_board_b.yaml)。

## 2. 板卡与关节绑定

| 板 | UID | 稳定串口路径 |
|---|---|---|
| A | C23B378115B092083830364A | /dev/serial/by-path/platform-fc800000.usb-usb-0:1.2:1.0-port0 |
| B | C23B37C715B092083830364A | /dev/serial/by-path/platform-fc800000.usb-usb-0:1.4:1.0-port0 |

USB插拔后ttyUSB编号会变。启动后必须核对UID，不能只认ttyUSB编号。

| 关节名称 | 板/接口 | 协议通道 | role | 减速比 |
|---|---|---:|---:|---:|
| head_joint | A/M2 | 1 | 1 | 516 |
| left_upper_arm_joint | A/M1 | 0 | 2 | 516 |
| left_forearm_joint | A/M3 | 2 | 4 | 290 |
| left_wrist_joint | A/M4 | 3 | 6 | 82 |
| right_upper_arm_joint | B/M1 | 0 | 3 | 516 |
| right_forearm_joint | B/M3 | 2 | 5 | 290 |
| right_wrist_joint | B/M4 | 3 | 7 | 82 |

B/M2未分配、禁用。YAML中channel为1～4，ApplyConfig服务使用0～3。当前上述七路 inverted=0、板上转速400 RPM。正角度表示逻辑正向，不保证左右两侧机械方向相同。

## 3. 每次开终端先做的操作

```bash
ssh xinhua@192.168.1.113
cd /data/ros2/robot_motor_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export PATH="$HOME/.local/bin:$PATH"
export ROS_DOMAIN_ID=173
```

若出现自定义消息/action类型invalid，先确认执行了两条source：
```bash
ros2 interface show robot_motor_interfaces/action/MoveJoint
ros2 interface show robot_motor_interfaces/msg/MotorStates
```

每块板只能由一个进程占用串口。现有节点运行时不要再启动同板节点，也不要同时使用socat或裸Modbus脚本。更换模式需先在原启动终端Ctrl+C退出，确认进程退出后再启动。写模式节点退出可能执行STOP_ALL，不能依赖退出节点保持BRAKE释放。

## 4. 启动模式

新启动示例，先确认该板没有已有驱动：
```bash
ros2 launch robot_motor_bringup bringup.launch.py \
  transport:=modbus_rtu \
  config_file:=$PWD/review_logs/multi10_confirmed.yaml \
  board_name:=main_board board_namespace:=board_a \
  allow_motion:=false ros_domain_id:=173
```

B板替换为 `board_name:=right_board board_namespace:=board_b`。

- 只读：`allow_motion:=false maintenance:=false`，所有写入包括STOP都被拒绝。
- 维护：`allow_motion:=false maintenance:=true`，允许清故障、配置、设零、刹车等维护操作，MOVE被拒绝。
- 运动：`allow_motion:=true`，同时通过UID、配置、故障、位置等检查后才可MOVE。
- `allowed_motion_joints:=right_wrist_joint` 将受限操作限定到指定关节；多关节用逗号分隔。
- 当前无机械软限位配置，测试使用 `commissioning:=true max_commissioning_step_deg:=10.0`。它只限制单次步长，不提供机械限位保护。
- 默认 `allow_unreferenced_relative:=false`。只有明确允许无有效零点相对运动时才设true；仍禁止无效零点下的绝对回零，不会自动设零。

## 5. 身份与状态读取

另开已source的终端：
```bash
ros2 service call /board_a/robot/board_identity std_srvs/srv/Trigger "{}"
ros2 service call /board_b/robot/board_identity std_srvs/srv/Trigger "{}"
ros2 topic echo /board_a/robot/motor_states --once
ros2 topic echo /board_b/robot/motor_states --once
```

检查identity_ok、communication_ok为true，stale和unknown_command_state为false。运动前检查目标通道enabled、role、gear_ratio、inverted、configured_rpm与配置一致，无故障且停止。检查dirty/storage_error/pending_save；命令接受不等于运动完成，也不等于保存成功。

## 6. 配置与保存

转速单位是**减速前电机RPM**。MoveJoint的motor_rpm是对板上配置的校验，不是调速命令；0表示使用板上值，非零必须与板上值一致。

例如B/M4已确认是右手腕82，设置400 RPM：
```bash
ros2 service call /board_b/robot/apply_config robot_motor_interfaces/srv/ApplyConfig \
"{channels: [{channel: 3, enabled: true, role: 7, gear_ratio: 82, inverted: 0, rpm: 400}], commit: true}"
```

部分列表只改变列出的通道，其余沿用板上配置。整板需停止；检查accepted、result_code=0、persisted=true和applied回读。YAML相应值也要匹配，否则运动会被配置一致性检查拒绝。重复提交完全相同配置可能changed=false，不代表错误，也不能当作强制SAVE。

本轮两板配置和六个手臂设零均已实际保存：[配置与设零记录](../../../GD32F303VET6_MotorControl_ROS2/review_logs/multi10_prepare_result.jsonl)。

## 7. 设零、相对运动、绝对回零

设零只在操作者明确指定当前姿态为机械参考时使用，不能作为自动消除故障的方法：
```bash
ros2 service call /board_b/robot/set_zero robot_motor_interfaces/srv/SetZero \
"{joint_name: right_wrist_joint}"
```
检查accepted、position_valid、persisted。此命令改变零点并保存，不会让电机返回之前的位置。

相对+10°示例（0.1745329252 rad），目标通道必须已具备运动条件：
```bash
ros2 action send_goal /board_b/robot/move_joint robot_motor_interfaces/action/MoveJoint \
"{joint_name: right_wrist_joint, mode: 0, angle_rad: 0.1745329252, motor_rpm: 400, timeout_sec: 30.0}" --feedback
```

负10°使用 `angle_rad: -0.1745329252`。每次执行相对命令都会在当前位置再增加该角度；不应将重复发送当作查询。

头部绝对回零：
```bash
ros2 action send_goal /board_a/robot/move_joint robot_motor_interfaces/action/MoveJoint \
"{joint_name: head_joint, mode: 1, angle_rad: 0.0, motor_rpm: 400, timeout_sec: 30.0}" --feedback
```

mode=0相对，mode=1绝对；角度为输出轴弧度。timeout_sec是监测超时，不是指定到位时长。完成后检查motion_completed、persisted、消息文本、通道fault、brake、duty和rpm。**本轮故障动作曾返回error_code=0但motion_completed=false，不能只看error_code。**

## 8. 停止、清故障、BRAKE

需要可写维护/运动模式：
```bash
# 停止B板右手腕
ros2 service call /board_b/robot/stop robot_motor_interfaces/srv/Stop "{joint_name: right_wrist_joint}"
# 停止整块B板；停止两板需分别调用A/B
ros2 service call /board_b/robot/stop robot_motor_interfaces/srv/Stop "{joint_name: ''}"
# 清除右手腕故障，不恢复位置有效性
ros2 service call /board_b/robot/clear_fault robot_motor_interfaces/srv/ClearFault "{joint_name: right_wrist_joint}"
# 解除刹车
ros2 service call /board_b/robot/set_brake robot_motor_interfaces/srv/SetBrake "{joint_name: right_wrist_joint, engage: false}"
# 锁上刹车
ros2 service call /board_b/robot/set_brake robot_motor_interfaces/srv/SetBrake "{joint_name: right_wrist_joint, engage: true}"
```

解除时需无忙状态、无目标故障并通过配置核对；有故障需先在静止条件下清除。BRAKE解除不是MOVE，PWM保持停止；减速箱可能仍不能轻松手动转动。锁上后应回读brake=true。

本轮右手腕清故障序号46、解除47成功；锁上48成功。本次最终已锁上。随后最终读取出现故障4，不能据解除时的fault=0判断当前仍无故障。

## 9. 故障处理流程

### 故障1：反馈异常
先停止并读状态，区分实际转动却无FG与完全未转动：
- 实际转动但FG=0：检查FG接线、共地、信号及对应捕获通道。
- 完全不转：检查电源、PWM、BRAKE、机械负载及是否能以所设速度起动。
- 命令被接受只表示固件接收，不能证明输出端电压或机械运动正确。
- 不自动反复清故障、提高转速或改零点。问题确认后，明确清故障并单次重试。

本轮左大臂第一次失败，清故障后相对+10°成功；右手腕两次运动均故障1、FG=0。不能把左大臂的恢复结果套用于右手腕。

### 故障4：非预期FG活动
新固件只对已提交有效减速比的通道按累计活动角度检查：≤10°不报警，>10°报警；不是净偏移。6 FG/电机转，516/290/82分别在第87/49/14个累计非预期脉冲触发。未配置有效减速比不使用固定10脉冲回退。

正常受控运动FG不纳入此活动累计。解除刹车后手动移动仍可能触发；因此BRAKE解除不等于开启无报警“拖动示教”。单路FG无法独立判断外力运动方向，不能凭它恢复可靠机械零点。

明确静止后可清故障，但失效零点不会自动恢复。需要绝对定位时由操作者重新建立参考；仅相对试转可明确启用allow_unreferenced_relative。

### 存储错误
dirty=1、storage_error=1、pending_save=1时，不把当前位置视为已成功保存。先确认整板停止，再尝试一次正常SAVE，回读三个标志。SAVE失败时停止进一步运动，检查供电与Flash保存路径。

当前ROS2节点**没有公开的/robot/save服务**。本轮通过停止该板ROS2串口占用后，使用现有ModbusRtuTransport发送SAVE操作码8、读取0x0036/37及0x0005～7，再重启节点。不能在ROS2占用串口时并行发送。

可在已停止该板驱动、source环境后使用下面的受控示例；只保存当前记录，不恢复无效零点：
```python
from robot_motor_driver.transport_modbus import ModbusRtuTransport
import time

port = "/dev/serial/by-path/platform-fc800000.usb-usb-0:1.4:1.0-port0"
expected_uid = "C23B37C715B092083830364A"
with ModbusRtuTransport(port) as t:
    uid = "".join("%04X" % v for v in t.read_registers(8, 6))
    assert uid == expected_uid
    # 操作者须确认整板已停止；仍需由固件quiet检查决定是否接受SAVE。
    seq = t.read_registers(0x36, 2)[0] % 65535 + 1
    t.write_command(seq, 8, 0, 0, retries=0)
    time.sleep(0.7)
    print("command:", t.read_registers(0x36, 2))
    print("dirty/storage/pending:", t.read_registers(5, 3))
```

库要求序号1～65535，传0会在发送前抛ValueError。不要盲目重发不确定是否执行的运动命令。本轮SAVE曾成功恢复，但运动失败后又复发；**Flash异常根因尚未查明**，不可写成已修复。

## 10. 本轮实际测试结论

| 操作 | 实际结果 |
|---|---|
| 七路配置400RPM、六臂设零 | 成功并保存 |
| 头部相对+10° | 10.0°，完成并保存 |
| 头部绝对0° | -0.1°，完成并保存 |
| 左大臂首次+10° | 故障1，未完成 |
| 左大臂清故障后+10° | 10.0°，完成并保存；绝对位置仍无效 |
| 左小臂+10° | 9.9°，完成并保存 |
| 左手腕+10° | 13.1°，完成并保存；之后出现故障4 |
| 右大臂+10° | 10.3°，完成并保存 |
| 右小臂+10° | 10.3°，完成并保存 |
| 右手腕首次及重试+10° | 均故障1，FG=0，未完成 |
| 右手腕解除/锁上BRAKE | 均命令成功；最终锁定，后来出现故障4 |

上述角度来自FG计数，不是外部角度仪实测。左手腕偏离请求3.1°，不能写成精确到10°。本轮未重新烧录、未做CAN、未做硬件电压测量，也未重新执行全套构建/单元测试。

日志目录：[review_logs](../../../GD32F303VET6_MotorControl_ROS2/review_logs)；关键结果：
- [头部回零](../../../GD32F303VET6_MotorControl_ROS2/review_logs/head_return_result.jsonl)
- [左大臂重试](../../../GD32F303VET6_MotorControl_ROS2/review_logs/left_retry_result.jsonl)
- [左小臂](../../../GD32F303VET6_MotorControl_ROS2/review_logs/remaining_left_forearm_joint.jsonl)、[左手腕](../../../GD32F303VET6_MotorControl_ROS2/review_logs/remaining_left_wrist_joint.jsonl)
- [右大臂](../../../GD32F303VET6_MotorControl_ROS2/review_logs/remaining_right_upper_arm_joint.jsonl)、[右小臂](../../../GD32F303VET6_MotorControl_ROS2/review_logs/remaining_right_forearm_joint.jsonl)
- [右手腕重试](../../../GD32F303VET6_MotorControl_ROS2/review_logs/right_retry_motion.jsonl)
- [右手腕解除BRAKE](../../../GD32F303VET6_MotorControl_ROS2/review_logs/right_brake_release_result.log)

## 11. 下一次测试顺序

1. 核对UID、稳定串口和上述最新状态；不要直接批量运行所有命令。
2. 优先排查右手腕无反馈、两手腕非预期FG，以及B板保存异常。
3. 确认机械状态，区分清故障、设零与相对试转，三者不能互相替代。
4. 单关节测试，等待完成且保存成功、刹车锁定后再测下一关节。
5. 记录命令序号、ROS2结果、角度、FG、fault、brake及三个存储标志。

历史文档：[控制修复与BRAKE说明](ROS2_控制修复与BRAKE操作说明.md)、[新固件头部测试记录](ROS2_新固件头部运动测试记录.md)。历史通过记录不代表当前故障已经消除。
