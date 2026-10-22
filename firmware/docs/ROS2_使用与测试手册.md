# ROS2 使用与测试手册

整理日期2026-10-09。维护源为固件工程ros2/；当前操作以本文为准，历史快照不代表现在板状态。新增接口只改变 ROS2 输入输出单位，固件继续使用输出轴0.1°计数；Modbus/CAN 报文、Flash格式不变。旧弧度接口兼容，标准 /joint_states 仍使用rad。历史实机日志保留原样。实际验证见 [验证记录](ROS2_度数接口验证记录.md)。

<a id="degrees-interface"></a>
重要参数、可修改入口和源码定位见[第17节](#ros2-parameter-guide)。

## 1. 加载环境

每个客户端终端重新加载构建后的工作区：
~~~bash
source /opt/ros/humble/setup.bash
source /data/ros2/robot_motor_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
ros2 interface show robot_motor_interfaces/action/MoveJointDeg
ros2 action list -t
~~~

板A命名空间为 /board_a/robot，板B为 /board_b/robot。关节绑定、UID、减速比和方向反相必须核对配置与板上读数。本次部署后节点为只读，allow_motion=false会拒绝运动与维护写入。以下运动示例需在试转授权后启用运动才能执行；启动不自动试转、设零或写入转速。

## 2. 相对运动和绝对回零

头部相对−3°示例：
~~~bash
ros2 action send_goal /board_a/robot/move_joint_deg \
  robot_motor_interfaces/action/MoveJointDeg \
  "{joint_name: head_joint, mode: 0, angle_deg: -3.0, motor_rpm: 4000, timeout_sec: 30.0}" \
  --feedback
~~~

绝对回到已建立的零点：
~~~bash
ros2 action send_goal /board_a/robot/move_joint_deg \
  robot_motor_interfaces/action/MoveJointDeg \
  "{joint_name: head_joint, mode: 1, angle_deg: 0.0, motor_rpm: 0, timeout_sec: 30.0}" \
  --feedback
~~~

板A左大臂、左小臂、左手腕分别用 left_upper_arm_joint、left_forearm_joint、left_wrist_joint；板B右侧分别用 right_upper_arm_joint、right_forearm_joint、right_wrist_joint。替换关节名和命名空间即可，未确认接线不应直接试转。

| 参数 | 作用 |
|---|---|
| joint_name | 本板配置中的关节名 |
| mode | 0相对当前位置；1绝对零点坐标 |
| angle_deg | 输出轴角度，正负选择逻辑方向，不是减速前电机角度 |
| motor_rpm | 0沿用板上转速；非零只校验已保存转速，不改变转速 |
| timeout_sec | 监控期限，不是承诺到位时间 |

度数直接乘10，沿用中点远离0的舍入：±0.05°变成±0.1°。小角度还可能受FG分辨率限制返回NO_OP。单次位移限±360°；绝对坐标允许跨圈，但本次位移仍受±360°及commissioning单步上限约束。绝对0°进入回零流程，相对0°为NO_OP。

## 3. 状态与限位

### 按关节标签读取（新增）

标签使用合法ROS话题名片段（例如head_joint，字母/数字/下划线，不含空格）；每个节点按本板 YAML 中的关节标签发布 robot/joints/<joint_name>/state，类型为 robot_motor_interfaces/msg/MotorStateDeg。接口随板命名空间变化，不依赖数组顺序；改变通道绑定后读取标签不变。仅为已配置到该板的关节创建话题，包括当前禁用/位置无效的通道，实际启用与有效性看消息字段。

~~~bash
# 头部：只读角度（°）
ros2 topic echo /board_a/robot/joints/head_joint/state --field angle_deg --once
# 查看有效性，false时显示的0不能当作机械零点
ros2 topic echo /board_a/robot/joints/head_joint/state --field position_valid --once
# 左右大臂完整状态
ros2 topic echo /board_a/robot/joints/left_upper_arm_joint/state --once
ros2 topic echo /board_b/robot/joints/right_upper_arm_joint/state --once
~~~

| 部位 | 常用标签（以所加载YAML为准） |
|---|---|
| 头部 | head_joint |
| 左大臂 / 左小臂 / 左手腕 | left_upper_arm_joint / left_forearm_joint / left_wrist_joint |
| 右大臂 / 右小臂 / 右手腕 | right_upper_arm_joint / right_forearm_joint / right_wrist_joint |

同样可读取 fault、brake、configured_rpm、target_deg。去掉 --once 持续观察。每条关节消息直接来自该周期的整板度数消息，不新增串口/CAN轮询或控制写入。MotorStateDeg本身没有时间戳、UID和通信新鲜度字段；用于控制判断时必须同时检查整板 motor_states_deg 的 identity_ok、communication_ok、stale 和 position_valid，不能仅凭关节角度认为通信正常。网络到达时间不保证同步。

新代码构建后需要重启对应驱动节点才会创建新增话题；加载环境本身不会更新正在运行的进程。

B板话题需要B节点实际启动；只启动board_a不会出现board_b话题。各终端必须加载install/setup.bash并使用相同ROS_DOMAIN_ID。找不到话题时先检查 ros2 node list、ros2 topic list，以及启动日志中的配置路径；不要在~目录使用指向工作区文件的$PWD相对路径。


~~~bash
ros2 topic echo /board_a/robot/motor_states_deg --once
ros2 topic echo /board_b/robot/motor_states_deg --once
~~~

结果 final_angle_deg；反馈 current_angle_deg、target_angle_deg；每路状态 angle_deg、target_deg。全部原始0.1°计数、有效标记、BRAKE、故障及保存字段保留。位置无效时转换后的姿态字段为0，不能解释为机械零点；原始计数保留。新旧话题由同一次采样及时间戳产生，网络到达时间不保证相同。

~~~yaml
joints:
  head_joint:
    board: main_board
    channel: 2
    role: 1
    gear_ratio: 516
    inverted: 0
    default_motor_rpm: 400
    limits_deg: [-30.0, 30.0]  # 示例，必须改为实际确认的机械限位
~~~

limits_deg:null表示未确认限位，默认拒绝运动，commissioning沿用已有单步保护。limits_rad仍有效；同一路同时提供两种键，即使都为null，也拒绝加载。范围必须是有限数值、最小≤最大且位于±360°。拒绝NaN、无穷、超限位移和不能表示的坐标。

## 4. 停止与错误处理

新旧动作共用worker、队列、关节锁、串口、序号管理和停止优先级，不能切换接口绕过UID、位置、故障、保存或速度校验。取消沿用Action CancelGoal。停止服务未变：
~~~bash
ros2 service call /board_a/robot/stop robot_motor_interfaces/srv/Stop \
  "{joint_name: head_joint}"
~~~

| 现象 | 处理 |
|---|---|
| action/message type invalid | 构建接口包并重新source环境；核对节点与客户端版本 |
| 找不到动作/话题 | 核对ROS_DOMAIN_ID、命名空间、节点和串口占用 |
| 只读拒绝 | 获准试转后启用allow_motion；维护服务另需maintenance |
| RPM mismatch | 读取板上转速，填0或相同值；改变速度走既有apply_config |
| 位置无效 | 不执行绝对回零，检查dirty、故障与实际机械基准 |
| 限位未确认/超限 | 确认机械范围后配置limits_deg，不同时填两种单位键 |
| UNKNOWN/取消后停止未确认 | 保持停机，检查串口和请求日志，按既有人工确认流程恢复 |
| persisted=false/storage_error | 检查Flash保存故障，不仅凭motion_completed继续动作 |

运动完成、位置有效、保存成功是三个独立结果，必须分别核对。不要通过重设零点掩盖未知机械位置；既有清故障、刹车释放、设零服务及操作互锁不变。

## 5. 弧度兼容

旧MoveJoint、MotorState、MotorStates定义、move_joint动作和motor_states话题不变。旧客户端仍可用angle_rad。日常建议使用度数动作；历史日志与旧弧度示例作为兼容记录保留。标准sensor_msgs/JointState继续使用弧度。


## 6. 板卡与关节绑定

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
| right_wrist_joint | B/M2 | 1 | 7 | 82 |

B/M4现在未分配，后续需在板端禁用并清除role。2026-10-10软件绑定已将右手腕改到M2；B板未连接，板端配置尚未迁移或验证。YAML中channel为1～4，ApplyConfig服务使用0～3。此前确认七路inverted=0；当前速度需重新读configured_rpm，不能沿用旧400RPM快照。正角度表示逻辑正向，不保证左右两侧机械方向相同。

## 7. 每次开终端先做的操作

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
ros2 interface show robot_motor_interfaces/action/MoveJointDeg
ros2 interface show robot_motor_interfaces/msg/MotorStatesDeg
```

每块板只能由一个进程占用串口。现有节点运行时不要再启动同板节点，也不要同时使用socat或裸Modbus脚本。更换模式需先在原启动终端Ctrl+C退出，确认进程退出后再启动。写模式节点退出可能执行STOP_ALL，不能依赖退出节点保持BRAKE释放。

## 8. 启动模式

新启动示例，先确认该板没有已有驱动：
```bash
ros2 launch robot_motor_bringup bringup.launch.py \
  transport:=modbus_rtu \
  config_file:=/data/ros2/robot_motor_ros2_ws/review_logs/head_back10_config.yaml \
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

## 9. 身份与状态读取

另开已source的终端：
```bash
ros2 service call /board_a/robot/board_identity std_srvs/srv/Trigger "{}"
ros2 service call /board_b/robot/board_identity std_srvs/srv/Trigger "{}"
ros2 topic echo /board_a/robot/motor_states_deg --once
ros2 topic echo /board_b/robot/motor_states_deg --once
```

检查identity_ok、communication_ok为true，stale和unknown_command_state为false。运动前检查目标通道enabled、role、gear_ratio、inverted、configured_rpm与配置一致，无故障且停止。检查dirty/storage_error/pending_save；命令接受不等于运动完成，也不等于保存成功。

## 10. 配置与保存

转速单位是**减速前电机RPM**。MoveJointDeg的motor_rpm是对板上配置的校验，不是调速命令；0表示使用板上值，非零必须与板上值一致。

例如B/M2已完成板端迁移且确认是右手腕82，设置400 RPM：
```bash
ros2 service call /board_b/robot/apply_config robot_motor_interfaces/srv/ApplyConfig \
"{channels: [{channel: 1, enabled: true, role: 7, gear_ratio: 82, inverted: 0, rpm: 400}], commit: true}"
```

部分列表只改变列出的通道，其余沿用板上配置。整板需停止；检查accepted、result_code=0、persisted=true和applied回读。YAML相应值也要匹配，否则运动会被配置一致性检查拒绝。重复提交完全相同配置可能changed=false，不代表错误，也不能当作强制SAVE。


## 11. 设零、相对运动、绝对回零

设零只在操作者明确指定当前姿态为机械参考时使用，不能作为自动消除故障的方法：
```bash
ros2 service call /board_b/robot/set_zero robot_motor_interfaces/srv/SetZero \
"{joint_name: right_wrist_joint}"
```
检查accepted、position_valid、persisted。此命令改变零点并保存，不会让电机返回之前的位置。

相对+10°示例，目标通道必须已具备运动条件：
```bash
ros2 action send_goal /board_b/robot/move_joint_deg robot_motor_interfaces/action/MoveJointDeg \
"{joint_name: right_wrist_joint, mode: 0, angle_deg: 10.0, motor_rpm: 400, timeout_sec: 30.0}" --feedback
```

负10°使用 `angle_deg: -10.0`。每次执行相对命令都会在当前位置再增加该角度；不应将重复发送当作查询。

头部绝对回零：
```bash
ros2 action send_goal /board_a/robot/move_joint_deg robot_motor_interfaces/action/MoveJointDeg \
"{joint_name: head_joint, mode: 1, angle_deg: 0.0, motor_rpm: 0, timeout_sec: 30.0}" --feedback
```

mode=0相对，mode=1绝对；角度为输出轴度数。timeout_sec是监测超时，不是指定到位时长。完成后检查motion_completed、persisted、消息文本、通道fault、brake、duty和rpm。**历史故障动作曾返回error_code=0但motion_completed=false，不能只看error_code。**

## 12. 停止、清故障、BRAKE

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


## 13. 故障处理流程

### 故障1：反馈异常
先停止并读状态，区分实际转动却无FG与完全未转动：
- 实际转动但FG=0：检查FG接线、共地、信号及对应捕获通道。
- 完全不转：检查电源、PWM、BRAKE、机械负载及是否能以所设速度起动。
- 命令被接受只表示固件接收，不能证明输出端电压或机械运动正确。
- 不自动反复清故障、提高转速或改零点。问题确认后，明确清故障并单次重试。


### 故障4：非预期FG活动
新固件只对已提交有效减速比的通道按累计活动角度检查：≤10°不报警，>10°报警；不是净偏移。6 FG/电机转，516/290/82分别在第87/49/14个累计非预期脉冲触发。未配置有效减速比不使用固定10脉冲回退。

正常受控运动FG不纳入此活动累计。解除刹车后手动移动仍可能触发；因此BRAKE解除不等于开启无报警“拖动示教”。单路FG无法独立判断外力运动方向，不能凭它恢复可靠机械零点。

明确静止后可清故障，但失效零点不会自动恢复。需要绝对定位时由操作者重新建立参考；仅相对试转可明确启用allow_unreferenced_relative。

### 存储错误
dirty=1、storage_error=1、pending_save=1时，不把当前位置视为已成功保存。先确认整板停止，再尝试一次正常SAVE，回读三个标志。SAVE失败时停止进一步运动，检查供电与Flash保存路径。

当前ROS2节点**没有公开的/robot/save服务**。历史测试通过停止该板ROS2串口占用后，使用现有ModbusRtuTransport发送SAVE操作码8、读取0x0036/37及0x0005～7，再重启节点。不能在ROS2占用串口时并行发送。

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


## 14. 自检、头部优先与左右臂测试

所有以下动作是供现场另行执行的步骤，本次整理不试转。先核对UID、接线、停止电平、负载和机械空间；头部优先，其次左右大臂/小臂，手腕可先不参与。

整板自检会依次试转全部四个物理输出，包括禁用通道：
~~~bash
ros2 action send_goal /board_a/robot/selftest robot_motor_interfaces/action/RunSelfTest   "{joint_name: '', scope: 1, direction: 0, timeout_sec: 60.0}" --feedback
~~~
scope=0指定joint_name进行单路自检；要求启用和有效减速比。查询channels、batch_completed、all_passed及persisted；未执行不是通过，残差只诊断。B板替换命名空间。

步骤：只读身份和状态→必要时配置并读回→故障原因解决后显式清除→自检并保存结果→获准设零→头部相对+5°并观察实际方向→绝对回零→左大臂、左小臂、右大臂、右小臂各+10°后按相同顺序绝对回零。
每路必须等motion_completed、fault=0、停稳及persisted后再下一步，不用shell循环盲发。绝对回零必须position_valid；若只是无参考相对移动，不伪造绝对零点。

## 15. 构建、模拟、双板启动与部署

三个包构建后客户端重新source。在174域测试，真实节点173域隔离：
~~~bash
cd /data/ros2/robot_motor_ros2_ws
set +u
source /opt/ros/humble/setup.bash
export PATH="$HOME/.local/bin:$PATH"
colcon build --packages-select robot_motor_interfaces robot_motor_driver robot_motor_bringup
source install/setup.bash
export ROS_DOMAIN_ID=174
colcon test --packages-select robot_motor_interfaces robot_motor_driver robot_motor_bringup
colcon test-result --verbose
~~~

默认robot.yaml只定义A板。双板要选择已核对且包含main_board、right_board的CONFIG；文件名含UNCONFIRMED的模板不能直接据此试转。两板每路role、减速比、inverted和预期转速要与实际匹配。
~~~bash
CONFIG="$PWD/review_logs/head_back10_config.yaml"
ros2 launch robot_motor_bringup dual_board.launch.py   config_file:="$CONFIG" transport:=modbus_rtu ros_domain_id:=173   board_a_start:=true board_b_start:=true   board_a_allow_motion:=false board_b_allow_motion:=false   board_a_maintenance:=false board_b_maintenance:=false
~~~

一板一进程、独立串口/worker/序号及UNKNOWN状态，B节点默认不启动；没有跨板同步轨迹、原子STOP_ALL或聚合成功。停止两板要分别确认stop_confirmed。
双fake用transport=fake和隔离测试域，不打开真实串口。节点运行时不要并用socat。

部署前备份src、docs、README及当前配置；停旧节点后同步维护源，构建模拟通过后恢复只读；不自动烧录、设零、自检或修改板上速度。
2026-10-09三包构建成功、490个独立pytest通过、47个源码文件三处一致，A板只读恢复；头部当时−13.3°。B板未因度数更新自动启动。见 [验证记录](ROS2_度数接口验证记录.md)；历史实机数据见 [历史目录](history/README.md)。


## 16. 发布状态、错误诊断与功能边界

新旧动作共用同一调度和串口队列，不能通过换单位接口绕过STOP、故障或Flash保护。每次用配置服务修改参数需核对applied和persisted，commit=false只暂存；channels=[]沿用板上配置，不用YAML覆盖。
当前运行配置中的expected_motor_rpm只是预期板上值，旧default_motor_rpm仍兼容；YAML不自动改板上速度。
没有公开ROS2 DIR服务或SAVE服务；方向通常由运动正负和inverted决定。需要手动发固件DIRECTION/SAVE时先释放该板串口所有权，不能与ROS2同时占用。

UNKNOWN状态保留STOP/STOP_ALL优先路径，帧发出不等于已停；人工核对机构停稳和实际请求结果后才调用：
~~~bash
ros2 service call /board_a/robot/acknowledge_unknown std_srvs/srv/Trigger "{}"
~~~
此确认不会自动续跑，也不修复硬件故障。

构建环境为远程Ubuntu20.04/aarch64、ROS2 Humble；diagnostic_msgs未安装时发布各板robot/diagnostics_json，包含angles_deg。标准/joint_states继续用rad且默认不发布。
本次文档整理未构建固件、未烧录、未自动试转或复现历史错误。实际是否能解除BRAKE及手动调整必须以服务结果、输出测量和机械负载判断，不能依据示例声称已解锁。

新增关节标签话题实际验证见 [验证记录](verification/joint_topics_20261009/验证记录.md)。

<a id="ros2-parameter-guide"></a>

## 17. 重要参数：修改入口、源码出处与生效方式

本节核对日期为2026-10-10。以下是源码规则与操作示例，不代表已执行配置或运动。所有路径中的工作区根目录均按实际安装位置替换。

### 17.1 三种修改入口与文件路径

| 修改方式 | 修改入口 | 生效方式 | 是否写板上Flash |
|---|---|---|---|
| 启动参数 | `ros2 launch ... 参数:=值` | 停止旧节点后重启，不能同时启动第二个串口节点 | 否 |
| 关节和板卡YAML | `config_file`实际指定的文件 | 编辑后重启节点读取；改源码模板不会改变已加载配置 | 否 |
| 板上配置 | `robot/apply_config`服务，`commit: true` | 事务提交后读取`applied`和保存标志；协议/地址/CAN节点号重启板后生效 | 是 |

板上配置成功后同步YAML中的预期值。不能通过修改`expected_motor_rpm`、减速比模板或动作中的`motor_rpm`替代板端提交。`commit: false`只暂存，不生效、不保存。

| 文件用途 | 固件工程维护源（相对工程根目录） | 本地独立ROS2工程 | 远程工作区（`/data/ros2/robot_motor_ros2_ws`下） |
|---|---|---|---|
| 启动参数定义 | `ros2/robot_motor_bringup/launch/bringup.launch.py` | `src/robot_motor_bringup/launch/bringup.launch.py` | `src/robot_motor_bringup/launch/bringup.launch.py` |
| 随包配置模板 | `ros2/robot_motor_bringup/config/robot.yaml`及双板模板 | `src/robot_motor_bringup/config/` | `src/robot_motor_bringup/config/` |
| 本次远程操作配置 | 本地模板不是该远程文件 | 不自动随模板修改 | `review_logs/head_back10_config.yaml` |
| 节点与调度逻辑 | `ros2/robot_motor_driver/robot_motor_driver/` | `src/robot_motor_driver/robot_motor_driver/` | `src/robot_motor_driver/robot_motor_driver/` |
| 消息、动作、服务 | `ros2/robot_motor_interfaces/` | `src/robot_motor_interfaces/` | `src/robot_motor_interfaces/` |
| PWM、运动和Flash规则 | `BSP/pwm.c`、`BSP/robot.c`、`BSP/robot.h` | 不含固件，查固件工程 | 不假定该工作区含固件源码 |

本地固件根目录：`E:/Date_save/embedded_code/GD32Projects/GD32F303/GD32F303VET6_MotorControl`。
本地独立工程根目录：`E:/Date_save/embedded_code/GD32Projects/GD32F303/GD32F303VET6_MotorControl_ROS2`。
远程当前操作配置的完整路径：`/data/ros2/robot_motor_ros2_ws/review_logs/head_back10_config.yaml`。

源码定位入口（点击文件后搜索下列函数/声明，不依赖固定行号）：

- **L，启动定义**：[bringup.launch.py](../ros2/robot_motor_bringup/launch/bringup.launch.py)，`generate_launch_description()`、`DeclareLaunchArgument`、传入`Node(parameters=...)`的字段。
- **C，YAML解析**：[config.py](../ros2/robot_motor_driver/robot_motor_driver/config.py)，`load_config()`、`_parse_driver()`、`_parse_boards()`、`_parse_joints()`、`_parse_degree_limits()`、`_parse_limits()`。
- **N，节点权限与动作检查**：[motor_node.py](../ros2/robot_motor_driver/robot_motor_driver/motor_node.py)，`RobotMotorNode.__init__()`、`_declare_parameters()`、`_select_board()`、`_on_move_goal()`、`_validate_move_goal()`、`_on_move()`、`_on_apply_config()`、`_states_message()`。
- **W，板端新鲜读数与互锁**：[scheduler.py](../ros2/robot_motor_driver/robot_motor_driver/scheduler.py)，`BoardWorker._task_move()`、`_check_soft_limits()`、`_config_mismatch()`、`_task_manual_adjust()`、`_task_config()`。
- **P，单位与范围**：[protocol.py](../ros2/robot_motor_driver/robot_motor_driver/protocol.py)，`RPM_MIN/MAX`、`MAX_MOVE_DECI_DEG`、`degrees_to_deci_deg()`、`validate_move_argument()`及各`validate_*()`。
- **S，身份、保存与UNKNOWN**：[board_session.py](../ros2/robot_motor_driver/robot_motor_driver/board_session.py)，`BoardSession.identify()`、`acknowledge_unknown()`、`read_save_flags()`及`DRIVER_*`错误码。
- **F，固件定义**：[robot.h](../BSP/robot.h)、[robot.c](../BSP/robot.c)、[pwm.c](../BSP/pwm.c)、[robot_port.c](../BSP/robot_port.c)，`robot_config_valid()`、`robot_command()`及其命令执行路径、`robot_update()`、`pwm_set_permille()`、`pwm_init()`和输出回调。

### 17.2 通信、身份和关节配置

以下字段在实际加载的YAML修改，由C解析，重启节点加载；不自动写板。

| 字段 | 作用、单位 | 默认/范围 | 定义与实际检查 |
|---|---|---|---|
| `boards.<板名>.uid` | 板身份，24位十六进制字符 | 必填，必须匹配实读UID | C `_parse_boards()`，P `validate_uid()`；S `identify()` |
| `transport` | `modbus_rtu`、`fake`、`can` | YAML必填；launch空值沿用板配置 | C `_parse_boards()`；N `_build_transport()`；CAN还需检查`transport_can.py`的依赖和硬件启用条件，不能仅改字段就声称可用 |
| `port` | RTU串口路径；CAN由具体适配器解释 | RTU的port必填非空；其他传输默认空；建议`/dev/serial/by-path/...` | C检查同配置内串口不重复；N `_build_transport()`；`transport_modbus.py`实际打开串口。稳定路径也必须核对UID |
| `address` | Modbus从站地址 | 默认1，1～247；独立串口两板可同为1 | C、P `validate_address()`；RTU传输使用此地址 |
| `baudrate` | 串口波特率，bit/s | 默认115200；解析要求正整数，必须匹配固件115200/8N1 | C `_parse_boards()`；`transport_modbus.py`。不是板端改波特率接口 |
| `node` | CAN节点号 | 默认1，1～127；CAN同配置不可重复 | C、P `validate_node()`；改板端需配置事务并重启板 |
| `pwm_hz` | 板PWM频率预期，Hz | 默认25000，20000～30000 | C、P `validate_pwm_hz()`；F配置提交和`pwm_set_frequency()`。YAML不会写频率；现有ApplyConfig读取并保留实际板级字段，不能通过该YAML字段直接修改板频率 |
| `joints.<标签>.board` | 所属YAML板名 | 必填，必须存在于`boards` | C `_parse_joints()`；N `_select_board()`选择本节点管理的板 |
| `channel` | YAML物理接口编号 | 必填1～4；M1～M4。服务、状态、报文为0～3 | C调用P `yaml_channel_to_protocol()`；同板同通道不可重复 |
| `role` | 固件关节角色 | 必填0～7，非零角色同板不可重复；0表示未分配 | C、P `Role`、`validate_roles_unique()`；W `_config_mismatch()`核对板上角色 |
| `gear_ratio` | 电机转数/输出轴转数 | 必填0、82、290、516；0未配置，不能正常运动 | C、P `validate_gear()`；W配置一致性及F有效减速比检查 |
| `inverted` | 逻辑方向反相 | 默认0，仅0/1 | C、W一致性；F方向=`(相对位移<0) XOR inverted`；不是物理方向测量 |
| `expected_motor_rpm` | 预期减速前电机转速，RPM | 默认400；400～8000；旧键`default_motor_rpm`兼容，新键存在时优先 | C `_parse_joints()`、N诊断；不是速度命令。动作非零RPM与板上值比较，不是与该YAML值比较 |
| `limits_deg` | 有效零点坐标中的机械范围，° | 未提供/null表示未知；两个有限数，最小≤最大，均在±360° | C `_parse_degree_limits()`；N检查绝对目标，W `_check_soft_limits()`检查本次预计终点 |
| `limits_rad` | 旧弧度限位 | null或两个有限数，±2π；与`limits_deg`同时存在即拒绝，包括两个null | C `_parse_limits()`；内部保存为rad；日常配置只用`limits_deg` |

角色表：1头部、2左大臂、3右大臂、4左小臂、5右小臂、6左手腕/手掌、7右手腕/手掌。`joint_name`是YAML标签，不是role数字；修改标签会改变动作参数及按标签发布的话题。板端`enabled`是ApplyConfig的字段，现有关节YAML没有可用的`enabled`开关，不要添加一个被忽略的键来尝试禁用电机。

### 17.3 启动权限、角度上限和参数覆盖

| 参数 | 默认、范围/含义 | 用户修改入口与生效 | 定义与实际判断 |
|---|---|---|---|
| `config_file` | 默认随包`config/robot.yaml` | L启动传绝对路径；重启读取 | L；N构造函数调用C `load_config()`；不要用`~`目录的`$PWD`拼工作区路径 |
| `board_name` | 默认空；多板必须指定已配置板名 | `main_board`或`right_board`，重启 | L、N `_select_board()` |
| `board_namespace` | 默认空；本项目A=`board_a`、B=`board_b` | L重启；动作/服务/话题随命名空间变化 | L的`Node(namespace=...)`、N `_absolute_name()` |
| `ros_domain_id` | L默认71；本次远程使用173；有效范围由ROS中间件决定 | L传173，同时每个客户端`export ROS_DOMAIN_ID=173` | L `SetEnvironmentVariable()`；不是YAML关节参数 |
| `allow_motion` | L默认false | L重启；true仍需通过其他检查 | N、W；false拒绝MOVE，不等于硬件自动制动 |
| `maintenance` | 默认false | L重启；允许配置/设零/刹车/人工调整等写入，不单独允许MOVE | N计算`read_only=not(allow_motion or maintenance)`；W及F手动互锁 |
| `allowed_motion_joints` | 默认`*`本板全部；空值不允许运动；逗号分隔本板标签 | L重启；建议明确白名单 | N解析，`_validate_move_goal()`；W允许通道检查。不是所有维护服务的通用白名单 |
| `allow_unreferenced_relative` | L默认false | 只有明确接受无基准相对位移才传true；重启 | N `_on_move()`传入W；W拒绝无效零点绝对运动，不自动设零 |
| `commissioning` | 默认false | L重启；true允许未知限位调试，并启用单次步长限制 | N `_on_move()`、W `_check_soft_limits()`；不是软限位开关，不会取消已配置且可验证的终点检查 |
| `max_commissioning_step_deg` | 默认10.0°；20/30是启动覆盖值 | L传值，重启；与commissioning一起使用 | N相对目标检查；W对绝对目标转换后的相对位移再检查。建议正有限数≤360且按0.1°设置；源码未对该启动值单独严格校验正值/有限性，勿填NaN、负数或无穷 |
| `polling_hz` | L默认5.0 Hz；0<值≤100 | L重启；RTU实际吞吐量限制可达频率，建议保持5 | N初始化、W轮询间隔；并非电机控制环频率 |
| `require_uid` | 默认true | L重启；实机保持true | N创建S时设置预期UID；关闭不是推荐的故障处理方法 |
| `publish_joint_states` | 默认false | L重启；标准`joint_states`使用rad | N `_publish_joint_states()`；自定义度数话题不受此开关影响 |
| `start_driver` / `log_level` | 默认true / info | L重启；false不启动驱动；日志级别按ROS支持值 | L `IfCondition()`、节点ROS参数 |

**覆盖关系**：通过现有launch启动时，L显式传入`allow_motion=false`、`allow_unreferenced_relative=false`、`polling_hz=5.0`等默认值，因此会覆盖YAML相应值。N `_declare_parameters()`以YAML值作节点默认值，但显式ROS参数优先。启动早期`configuration:`显示的是YAML摘要；确认实际模式看后面的`ready:`及参数查询。不要仅修改YAML中的`allow_motion`期待launch自动开启运动。

以下YAML策略字段由C解析：`board_motion_policy`默认且仅接受`serial`；`speed_policy`默认`keep_requested`，还接受`restore_previous`。目前后者只出现在解析/摘要中，没有调度器自动改速/恢复动作，不能据此声称启用自动转速恢复或跨板轨迹同步。

高级节点参数：`stale_after_sec`默认1.0秒（N `_states_message()`判过期），`response_timeout`默认1.5秒（P `DEFAULT_RESPONSE_TIMEOUT`，N赋给传输的响应超时）。两者不是动作超时，当前L未暴露同名启动参数；`... stale_after_sec:=...`不会自动成为该节点参数。需要使用ROS节点参数入口或另行修改launch才能定制；本次不修改代码。源码未对这两个值单独做正有限范围校验，保留默认值。`joint_states_topic`、`motor_states_topic`、`motor_states_deg_topic`和`publish_diagnostics`也由N声明，L未提供同名参数入口；默认依次为`joint_states`、`robot/motor_states`、`robot/motor_states_deg`、true。

### 17.4 三层角度限制及PWM、速度关系

| 限制 | 数值/入口 | 适用条件与实际检查 | 如何修改 |
|---|---|---|---|
| 调试单次步长 | `max_commissioning_step_deg`，默认10° | commissioning=true时，相对指令及绝对指令所需位移；N和W；超限105 | 修改启动命令重启，不改固件 |
| 关节机械角度范围 | YAML `limits_deg: [最小, 最大]` | 位置有效时检查相对运动终点和绝对目标；C、N、W。未知位置无法可靠检查终点 | 修改实际配置文件、确认零点后重启，不写板Flash |
| 单次固件MOVE位移 | ±360°（±3600个0.1°单位） | P `validate_move_argument()`、W `_task_move()`和F `robot_command()`命令执行路径；绝对坐标先转为本次位移，不自动拆分 | 修改固件与相应协议校验才可改变，本次不修改 |

单次上限20°不限制累计行程，连续两次+20°可累计到+40°。若已配置机械限位，位置有效时仍会检查累计后的终点。`limits_deg:null`不是无限安全范围；只有commissioning明确接受未知范围后才可试转。位置无效时绝对目标始终拒绝；无基准相对运动还需要allow_unreferenced_relative=true，并在无法验证终点时通过调试入口明确接受风险。设置新零点也会改变同一限位数值对应的机械姿态，必须重新核对。

动作接口：[MoveJointDeg.action](../ros2/robot_motor_interfaces/action/MoveJointDeg.action)。`joint_name`必须属于本板；`mode`仅0相对/1绝对；`angle_deg`为输出轴度数、必须有限并可转换成int32的0.1°；`motor_rpm`为0沿用或400～8000校验；`timeout_sec`为0.5～300秒、没有自动默认30秒。CLI示例显式给30，不是到位承诺。度数转换直接舍入到0.1°，±0.05°按远离0方向舍入；过小位移还可能没有足够FG步数，返回NO_OP。

RPM修改入口：[ApplyConfig.srv](../ros2/robot_motor_interfaces/srv/ApplyConfig.srv)与[ChannelConfig.msg](../ros2/robot_motor_interfaces/msg/ChannelConfig.msg)，N `_on_apply_config()`、W `_task_config()`、F `robot_config_valid()`。`channels`可给部分通道，其余取板上新鲜配置；不能省略该通道的role/gear/inverted并期待自动继承。`accepted/result_code`描述接受，`persisted`单独描述保存。

PWM入口为F `pwm_set_permille(channel,duty)`，通道0～3，底层把duty限制到0～1000。**低电平占空比**：0持续高电平停止，100是10%低电平，1000持续低电平；正常运动受`ROBOT_DUTY_MAX=950`限制。刹车锁定输出强制0。`robot_update()`由FG误差调PWM，目标RPM增大不保证PWM达到上限，接近目标会减速。`duty_permille`只是读回的软件值，不是引脚电压；目前没有ROS2/Modbus直接设PWM的公开命令，不能通过`motor_rpm:0`停机或通过1000填写RPM来设置PWM。固件PWM频率、占空比上限及控制参数要在固件修改、构建并烧录；改ROS2协议常量不能改变实际硬件。

### 17.5 启动、修改和核对示例

先在每个终端执行：

```bash
source /opt/ros/humble/setup.bash
source /data/ros2/robot_motor_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
```

**先检查串口所有者**。一个launch父进程加一个robot_motor_node子进程是正常的一组；同一板出现两组才是重复。A/B分别核对，不要用同一查询混入另一板。停止节点不会保证之前释放的刹车自动锁定，应先在通信正常且允许写入时锁刹车、确认停稳，再重启。

```bash
pgrep -af 'board_namespace:=board_b|__ns:=/board_b'
PORT=/dev/serial/by-path/platform-fc800000.usb-usb-0:1.4:1.0-port0
fuser -v "$(readlink -f "$PORT")"
# 按当前输出确认PID，停止本板旧launch及驱动；不要照抄历史PID
# kill -TERM <旧launch_PID> <旧driver_PID>
```

以下三套模板**只能选一套启动**，B板保留一组驱动；A板替换为main_board/board_a，白名单也改为本板标签。

```bash
# 只读：连STOP写入也拒绝；不自动改变硬件刹车
ros2 launch robot_motor_bringup bringup.launch.py \
  transport:=modbus_rtu \
  config_file:=/data/ros2/robot_motor_ros2_ws/review_logs/head_back10_config.yaml \
  board_name:=right_board board_namespace:=board_b \
  allow_motion:=false maintenance:=false ros_domain_id:=173

# 维护：配置/设零/刹车/人工调整，禁止MOVE
ros2 launch robot_motor_bringup bringup.launch.py \
  transport:=modbus_rtu \
  config_file:=/data/ros2/robot_motor_ros2_ws/review_logs/head_back10_config.yaml \
  board_name:=right_board board_namespace:=board_b \
  allow_motion:=false maintenance:=true ros_domain_id:=173

# 受限运动：20°改30°就是更改下面30.0并重启；单次限制并非机械限位
ros2 launch robot_motor_bringup bringup.launch.py \
  transport:=modbus_rtu \
  config_file:=/data/ros2/robot_motor_ros2_ws/review_logs/head_back10_config.yaml \
  board_name:=right_board board_namespace:=board_b \
  allow_motion:=true maintenance:=false \
  allow_unreferenced_relative:=false commissioning:=true \
  max_commissioning_step_deg:=30.0 \
  allowed_motion_joints:=right_upper_arm_joint ros_domain_id:=173
```

模板默认不允许无基准相对运动。只有明确允许时才改为`allow_unreferenced_relative:=true`，它不自动设零；右手腕迁移未完成不放入白名单。查询实际值和通信：

```bash
ros2 param get /board_b/robot_motor_driver max_commissioning_step_deg
ros2 param get /board_b/robot_motor_driver allow_motion
ros2 param get /board_b/robot_motor_driver allow_unreferenced_relative
ros2 param get /board_b/robot_motor_driver config_file
ros2 service call /board_b/robot/board_identity std_srvs/srv/Trigger "{}"
ros2 topic echo /board_b/robot/motor_states_deg --once
```

确认UID正确、communication_ok=true、stale=false、unknown_command_state=false、无运动/故障/存储错误。参数查询成功不等于板子已连上。

**机械限位配置示例**：编辑`config_file`实际指定文件中的原有right_upper_arm_joint条目，仅替换限位键；删除该条目已有的`limits_rad`，其余绑定不动。不要复制第二个同名关节。

```yaml
# 仅格式示例；-30～60不是现场确认范围，替换为实际机械限位后才能使用
limits_deg: [-30.0, 60.0]
```

建立并确认机械零点后，在右大臂测试时可使用`commissioning:=false`；所请求关节范围未知或位置无效仍拒绝。配置通过`load_config()`静态解析后重启，并查询关节position_valid与整板新鲜度。不通过反复重设零点绕过限位。

**右大臂RPM改400并保存**（示例不自动执行，整板必须静止且退出手动模式；维护或运动节点均可）：

```bash
ros2 service call /board_b/robot/apply_config \
  robot_motor_interfaces/srv/ApplyConfig \
  "{channels: [{channel: 0, enabled: true, role: 3, gear_ratio: 516, inverted: 0, rpm: 400}], commit: true}"
ros2 topic echo /board_b/robot/joints/right_upper_arm_joint/state --once
```

成功判据：accepted=true、result_code=0、persisted=true、applied中channel=0/rpm=400，回读configured_rpm=400。相同配置changed=false可能是正常免写Flash。更新实际YAML中的right_upper_arm_joint.expected_motor_rpm为400，重启使诊断预期一致；修改RPM不自动设零。以后动作填motor_rpm=400或0，不再填3000。

```bash
# 仅在已允许试转、通信正常、零点与限制符合条件后发送；本节不自动执行
ros2 action send_goal /board_b/robot/move_joint_deg \
  robot_motor_interfaces/action/MoveJointDeg \
  "{joint_name: right_upper_arm_joint, mode: 0, angle_deg: -10.0, motor_rpm: 400, timeout_sec: 30.0}" \
  --feedback
```

刹车、人工调整、设零及停止分别参见第11～13节与[人工调整手册](Robot_人工调整与故障4恢复.md)。人工调整会使位置无效，普通BRAKE释放不会暂停故障4。

### 17.6 错误判据与实机快照

| 现象 | 判断与处理 | 来源 |
|---|---|---|
| `The passed ... type is invalid` | 客户端未加载自定义接口，先source install，再用`ros2 interface show`；仍缺失再检查接口构建 | 接口定义文件、客户端环境 |
| `waiting for service` | 核对该板节点、ROS_DOMAIN_ID、命名空间、接口版本；不要重复启动第二组节点 | L命名空间与环境；N `_create_interfaces()` |
| `FC3 byte count ... does not match ...` | 回复与请求不匹配；先fuser查同串口重复访问，再查残留/延迟帧和通信，不能只盲目增大超时 | `transport_modbus.py`及P读响应校验 |
| 109 / `driver halted after an UNKNOWN outcome` | 未发送新的写命令；brake默认false不是有效电平。先恢复通信、核对请求结果和现场停稳，再显式调用acknowledge_unknown；不自动重试运动 | S `DRIVER_HALTED`、N `_on_acknowledge_unknown()` |
| 105 / `caps one command` | 调试单步超限，Nothing was sent；只在机械允许时调整启动上限或逐步执行，每步确认结果 | N `_on_move()`、W `_task_move()` |
| 105 / `outside ... limits` | 已配置机械范围的目标超限，不能靠增加单步上限解决 | W `_check_soft_limits()` |
| 111 / RPM mismatch | 动作非零motor_rpm与板上不同；读取configured_rpm，填一致值或0；需要改速走apply_config | S错误码、W `_task_move()` |
| 104 / 位置无效 | 不接受绝对坐标；确认机械参考后设零，或明确授权无基准相对调试，不能把angle_deg=0当实际零点 | W `_task_move()`、N状态转换 |
| 112 / 未确认限位或终点不可验证 | 配置并确认机械范围及零点，或明确选择调试；已有零点仍不等于有机械限位 | N、W限位检查 |
| 100 / 只读拒绝 | 使用一个维护节点或授权运动节点；只读连STOP写入也拒绝 | N `_maintenance_command()`、W、S |
| busy / 手动互锁 | 任一路manual或退出等待时整板禁止自动运动/配置提交；按手册退出并确认连续静止 | W及F `robot_command()` |
| persisted=false | 接受/完成/保存是独立结果；检查dirty/storage_error/pending_save，不能直接认定Flash成功 | S、W结果、F保存逻辑 |

恢复UNKNOWN的命令，仅在上述检查完成后使用：

```bash
ros2 service call /board_a/robot/acknowledge_unknown std_srvs/srv/Trigger "{}"
# B板将board_a改为board_b；确认只解除软件锁，不恢复运动、不释放刹车
```

**2026-10-10本会话最近实读记录，不是默认配置或当前持续在线保证**：A板UID C23B378115B092083830364A，M1左大臂516/4000RPM、M2头部516/4000RPM、M3左小臂290/4000RPM、M4左手腕82/400RPM；B板UID C23B37C715B092083830364A，M1右大臂已提交400RPM并设零成功，M3右小臂290/3000RPM，板上右手腕仍是M4/82/400RPM、M2禁用。软件及实际新接线右手腕为M2，板端迁移仍待完成，不能按软件标签直接试转。节点20°/30°上限只是启动参数，不随Flash保存；插拔或重启后必须重新核对。旧记录保留，不作为当前电平或位置证据。

## 右手腕M4迁移到M2（2026-10-10）

软件配置right_wrist_joint.channel=2，协议通道为1；右大臂M1、右小臂M3保持不变。减速比82、role=7、方向反相及各配置文件原速度保留；模板速度与已部署配置可能不同，不应用模板默认值覆盖板上速度。

迁移准备时只完成软件配置同步；2026-10-10后续实读仍显示板端M4为右手腕、M2禁用，因此迁移尚未完成。以下为待执行验收步骤，不代表已修改板端：确认UID C23B37C715B092083830364A和稳定串口路径；整板静止且无存储错误；读取四路现行配置，保存备份；以一次ApplyConfig事务将旧M4的右手腕配置复制到M2，同时将M4 enabled=false、role=0，其余通道保持原值。迁移时保留实读的gear_ratio/inverted/rpm，不直接使用示例速度。

提交后确认result_code=0、persisted=true及四路applied回读正确。旧M4位置和零点不复制到M2，不自动设零；未建立新零点前不要执行绝对回零。更新YAML端口/速度与实读一致后，重启B板只读节点，再读取：

~~~bash
ros2 topic echo /board_b/robot/joints/right_wrist_joint/state --once
~~~

应显示channel=1、role=7、gear_ratio=82，通信新鲜度仍查看整板motor_states_deg。标签读取命令和运动joint_name不变。历史M4测试日志保留原样，不作为新接口的验证结果。


## 2026-10-10人工调整与故障4恢复更新

新增操作码12、能力寄存器0010和每路+30模式查询。故障4每次重新累计>10°构成事件：5000ms内1～2次到期自动清除，第3次重启窗口；保持停止、制动及位置无效。人工调整暂停该路故障4、冻结有符号位置，并独占整板运动。完整报文、ROS2服务、退出及保存步骤以[人工调整与故障4恢复](Robot_人工调整与故障4恢复.md)为准，历史日志不改写。


## 18. 无需登录的Web管理界面（2026-10-12）

新增robot_motor_web，不修改固件或旧ROS接口。用户已将访问策略调整为无需登录；浏览器访问`http://192.168.1.113:8443/`查看状态，在唯一控制会话中操作。默认网页策略只读，节点权限和通信互锁仍有效，不自动试转、设零、解锁或写Flash。

安装、REST/WebSocket字段、配置与节点管理、断线制动和恢复步骤见[Web包使用说明](../ros2/robot_motor_web/README.md)。API参考为设备`/api/docs`，OpenAPI为`/api/openapi.json`，没有公网CDN依赖。网页不直接打开串口，外部节点占用时拒绝重复启动；配置草稿与板上Flash提交分开。

浏览器每秒心跳，失联5秒尝试停止并制动网页操作过的关节；只有ROS回读确认才显示已停止，通信异常报告未确认。无有效零点不显示可信0°，不自动设零。右手腕软件M2/旧板端M4冲突仍需单独处理，不自动迁移。

无需登录不是安全认证；仅部署在受控局域网，不做公网端口映射。HTTP默认使用8443，HTTPS可选但仍无需登录。真实运动及Flash事务需现场确认，部署验收只查询实机状态。

实际验证结果见[Web验证记录](verification/ROS2_Web验证记录.md)。

### 18.1 网页开启与控制入口

已运行时直接访问 **http://192.168.1.113:8443/**，无需登录，不要重复启动。服务未运行时，在xinhua执行：

```bash
source /opt/ros/humble/setup.bash
cd /data/ros2/robot_motor_ros2_ws
source install/setup.bash
source .web-venv/bin/activate
export ROS_DOMAIN_ID=173
ros2 run robot_motor_web robot_motor_web serve \
  --config /data/ros2/robot_motor_ros2_ws/review_logs/head_back10_config.yaml \
  --ip 192.168.1.113 --port 8443
```

1. 浏览器核对UID和通信，点击“取得控制权”。
2. 对应板点击“启动 / 参数”，先以只读模式保存草稿并应用启动，等待身份和状态正确。网页只管理自己启动的节点；存在同板外部节点/串口所有者时，先确认停稳后在原终端停止正确节点。
3. 需要写操作时，选择“维护”或“运动”，保存草稿后“应用并重启本板节点”。维护支持刹车、手调、设零、配置；运动还支持运动/自检，受白名单和互锁限制。整板停止、锁刹车、无手调及存储状态正常后才能切换。
4. 仅测试头部时，运动白名单填`head_joint`，单步上限先填`10`。未知限位仅在现场允许时开启`commissioning`；不默认开启无零点相对运动。
5. 在头部卡片选择“相对运动”、角度`5`、RPM`0`、超时`30`，点击一次“发送一次运动”。RPM0沿用板速，超时不是指定到位时间；核对最终完成、位置有效和保存结果。
6. 手动调姿态先支撑负载，再“进入手调”；完成后“退出手调”，等待停止及保存。模式使位置无效，不自动设零。零点只能在确认当前机械姿态后单独设置。

网页顶部红色“■ 双板停止”、各板“整板停止”、关节“停止”是软件停止入口；目前没有键盘急停快捷键。核对`stop_confirmed: true`，不要把接受请求当成确认停住。只读节点会拒绝STOP；关闭浏览器触发的失联处理和Ctrl+C关闭动作客户端不能代替主动停止或硬件急停。

完整的前台/后台启动、模式设置、相对/绝对运动、刹车和手调、自检、板配置保存、结束操作及ROS2停止命令见[网页逐步操作说明](../ros2/robot_motor_web/README.md)。API也可由其他客户端调用，但需唯一控制会话、每秒心跳及请求ID；不与CLI并发写入。

### 18.2 网页存储恢复

板卡“重试保存”复用固件SAVE，不设零、不运动、不清故障、不提交配置暂存。必须整板停止、锁刹车、退出手调；接受命令和确认保存分别报告。

若storage_error/dirty/pending_save导致普通重启被拒绝，先选择“启动 / 参数 → 保存恢复（仅STOP/SAVE）”，保存草稿并应用重启。该受限模式只允许停止和SAVE，仍检查其他停止/身份/通信互锁。点击“重试保存”后核对persisted、flags_valid及三个保存标志；失败或未知不自动重发。保存确认成功后再切换维护/运动，原电机故障和无效零点保持不变。

服务：`/board_a/robot/retry_save`、`/board_b/robot/retry_save`，类型`robot_motor_interfaces/srv/RetrySave`，请求`confirm: true`。API：`POST /api/v1/boards/{board}/save`。完整字段和步骤见[网页存储恢复说明](../ros2/robot_motor_web/README.md)。只读模式不能执行SAVE。

### 18.3 DIR电平换向按钮（2026-10-13）

用户最终确认需要切换实际DIR输出命令。关节卡片显示“DIR换向：低→高 / 高→低”；点击确认后调用SetDirection服务和固件操作码7。要求停止运动、退出手调、目标关节锁刹车且连续静止500ms，所有UID、通信、故障及UNKNOWN保护保留。只读及保存恢复模式拒绝换向。

服务`/board_a/robot/set_direction`、`/board_b/robot/set_direction`，类型`robot_motor_interfaces/srv/SetDirection`；请求joint_name、high。API为`POST .../joints/{joint}/direction`。返回direction_confirmed表示软件指令位回读一致，不表示电压实测正确。

该命令不运动、不释放刹车、不修改零点、配置或Flash。下一次MOVE会按角度符号与inverted重设DIR，不能用此按钮永久反转正负角度映射。原相对角度换向输入按钮已替换。完整说明见[DIR电平换向](../ros2/robot_motor_web/README.md)。

### 18.4 双板串口与连接检查

首页顶部提供“检查两板连接”按钮，无需取得控制权。A/B板分别自动刷新串口、驱动板通信和运行权限，检查不会打开串口、发送Modbus报文、重启节点或执行电机控制。串口路径每秒最多检查一次，点击按钮仍遵循该缓存，不是强制重新扫描USB。

- “串口存在”只说明当前生效配置的设备路径存在，不能证明驱动板或电机正常。稳定路径解析后的实际设备名同时显示；未应用的配置草稿不参与检测。
- “驱动板通信正常”要求ROS状态新鲜、通信成功且实际UID与预期UID匹配。网页接收状态超过2秒沿用现有规则标记过期；驱动节点自身的通信失效标志也会显示。
- “节点未启动”：串口可能已经连接，但没有对应的ROS驱动节点；在“启动 / 参数”中核对模式并启动，不能另开第二个串口控制节点。节点存在但尚无快照显示“等待ROS状态”。
- “串口未连接”：检查板子供电、USB连接和生效串口路径；拔插后设备名可能改变，优先使用稳定by-path路径。路径解析或权限检查异常显示“串口检查失败”，可展开错误。
- “UID不匹配”：检查两块板的串口绑定，不允许将另一块板当作目标板操作。失联时当前实际UID显示“当前未确认”，最后收到的UID单独标为历史，不作为当前身份。
- “模拟连接”不代表真实USB串口或电机连接。运行权限“只读／维护／运动／保存恢复”独立显示，不代表连接是否正常。

REST读取`GET /api/v1/boards`及`GET /api/v1/boards/{board}/state`，新增`state.connection`（单板响应为`connection`），WebSocket `/events`同样携带。主要字段如下：

| 字段 | 含义 |
|---|---|
| `transport`、`path`、`device` | 生效通信方式、配置串口、解析后的设备路径 |
| `serial_status` | `present`存在、`missing`缺失、`error`检查失败、`simulated`模拟、`not_applicable`非串口方式 |
| `serial_error`、`node_error` | 文件路径或ROS图检查错误，不等于电机故障 |
| `checked_at` | 后端串口检查时间，Unix秒 |
| `node_present` | 网页拥有的节点仍运行，或ROS图存在相应驱动节点 |
| `communication_status` | `connected`、`serial_missing`、`serial_error`、`node_not_started`、`node_check_error`、`waiting_state`、`state_stale`、`communication_error`、`uid_mismatch`、`uid_unconfirmed` |
| `expected_uid`、`current_uid`、`last_uid` | 预期、当前可信通信读到的UID、最近历史UID；当前未确认时为null |
| `last_state_stamp`、`state_age_sec` | 最近ROS状态时间戳及网页接收间隔；无记录时为null |
| `identity_ok` | 本次综合连接判定正常，不能作为电压或机械实测结论 |

检测实现为`robot_motor_web/connection.py:ConnectionMonitor`，通过`Gateway.state`接入原API及WebSocket；不修改ROS消息、固件协议或运动互锁。无法仅凭此功能确认每路电机已插入，电机响应仍需独立自检。
