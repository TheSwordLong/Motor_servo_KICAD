# 历史参考：ROS2_控制修复与BRAKE操作说明

> 保留当时方案、观测及验证结果，不是现行操作步骤。历史“当前”仅指记录当天；请使用 [文档索引](../../../GD32F303VET6_MotorControl_ROS2/README.md)。原报文与测量值不改写。

> 后续更新：新固件故障4模拟规则已同步，头部400RPM的+5°及回零已通过真实ROS2执行。旧固定第11脉冲说明已过时，见[新固件测试记录](ROS2_新固件头部运动测试记录.md)。

> 当前日常操作请使用 [ROS2度数接口使用说明](../ROS2_使用与测试手册.md#degrees-interface)：angle_deg直接输入度数，motor_states_deg直接读度数。本文原弧度命令保留为兼容或历史参考，不改写已有实机日志。


# ROS2 控制修复、功能与操作说明（2026-10-08）

## 本次范围

远程工作区：`/data/ros2/robot_motor_ros2_ws`。修复 ROS2 驱动及启动文件，不修改或烧录固件，不发送真实电机命令。测试使用 fake 模拟板及独立 ROS_DOMAIN_ID=173；软件通过不代表硬件验证通过。

## 修复内容

1. STOP 取消此前排队的对应关节运动；STOP_ALL 取消此前排队的整板运动及自检。任务在出队和发送前检查取消状态。已经开始传输的帧不能撤回，需等待停止确认。
2. MOVE、SELFTEST 和 BRAKE 释放检查板上 enabled、role、gear、inverted 与 YAML 是否一致，不一致返回驱动错误 108，不发送执行命令。RPM 单独处理：目标 motor_rpm 非零时必须与板上保存值一致；零表示使用板上当前值，不等于低速。
3. 新增启动参数 allowed_motion_joints（逗号分隔关节名）：`head_joint` 只允许头部；`*` 保持允许所有配置关节；空字符串拒绝全部运动。约束 MOVE、自检、BRAKE 释放及设零；STOP 和 BRAKE 接合不受白名单限制，但仍需要写入权限及正确 UID。受限白名单拒绝整板自检。
4. 绝对目标 0° 正常进入实时位置计算。单次位移与调试步长上限检查目标减去当前位置，绝对目标值不再被当作位移。
5. 双板 Action 测试改用 Action 图查询；双板启动显式传递 publish_joint_states=false。右臂模拟测试使用右臂角色配置。

白名单为启动时策略，更改后需要重启节点。只读模式仍拒绝所有写入（包括 STOP）；需要维护操作时启动 maintenance=true。它不是对其他独立程序的串口操作限制。

## 当前已实现功能

| 功能 | 实现与限制 |
|---|---|
| 双板运行 | 每板独立节点、通信工作线程、序号与 UNKNOWN 状态；命名空间隔离。不是双板同步轨迹控制 |
| 头部、左右臂运动 | MoveJoint 支持相对及绝对角度；angle_rad 为弧度；固件收到的是输出轴角度。右臂实际参数尚需确认 |
| 绝对回零 | mode=1、angle_rad=0；需要有效零点，且实时计算位移满足限制 |
| 速度 | motor_rpm 为减速前电机 RPM；非零值用于核对板上 RPM，不自动改写速度。改速需静止后 apply_config |
| BRAKE 接合/释放 | SetBrake.engage=true/false；检查 accepted、result_code 和读回 brake。false 只是释放电机制动，不是机械解锁或持续拖动模式 |
| 停止/取消 | 单路 STOP、整板 STOP_ALL、Action 取消；停止请求接受与 stop_confirmed 分开报告 |
| 自检 | 单关节及整板自检；反馈往返脉冲、残差与最终结果，整板结果与各通道通过分开报告 |
| 设零 | 当前机械姿态定义为零点并请求保存；不是回到原来的机械零点 |
| 清故障 | 显式服务，之后读回故障。不会自动恢复丢失的位置基准 |
| 配置 | 配置暂存、提交、读回校验；需要整板静止，改变角色/减速比/反相会影响零点与自检记录 |
| 状态及诊断 | UID、配置、位置、刹车、FG相关反馈、故障、保存状态；无 diagnostic_msgs 时使用 diagnostics_json |
| 通信异常 | 请求去重、超时核对、UNKNOWN 锁定；重连不自动续跑 |
| CAN | 有传输模块与编解码测试；仍保留依赖及硬件验证门槛，本次未验证真实 CAN |

## 释放 BRAKE 后能否手动调整角度

可以请求释放 BRAKE，但目前固件没有持续手动拖动模式。单路 FG 不包含方向，外力推动产生的脉冲不能被可靠换算为有符号当前位置。

已核对固件 BSP/robot.c：正常非运动状态下累计非预期 FG，0～10 个不报警，第 11 个触发 F_UNEXPECTED（故障4），停止输出并刹车；这个分支不因 brake=0 而豁免。因此手动调整可能重新触发刹车，且即使未超过阈值也不能据此证明位置仍可信。不要反复清故障、释放刹车来代替专门的拖动模式。

若只需要调整一次姿态：先停止并确认；支撑有重力负载的关节；释放后查看返回值；人工调整后接合刹车，检查 FG/故障/位置状态。只有明确要将新姿态定义为零点时，才清除可清故障、设零并检查保存。设零会替换原零点，不保留原机械坐标。需要持续拖动及准确跟踪手动角度，需另行设计模式和可判方向的反馈，当前接口不提供该能力。

## 头部操作命令（供现场执行，本次未执行真实命令）

先核对头部仍为 A 板 M2、减速比516，UID与配置匹配；不要用历史配置代替当前接线确认。

```bash
source /opt/ros/humble/setup.bash
source /data/ros2/robot_motor_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=71
cd /data/ros2/robot_motor_ros2_ws
# 仅维护：允许停止/刹车/设零，拒绝 MOVE 和自检
ros2 launch robot_motor_bringup bringup.launch.py \
  transport:=modbus_rtu \
  config_file:=/data/ros2/robot_motor_ros2_ws/src/robot_motor_bringup/config/robot_dual_UNCONFIRMED_RIGHT_ARM.yaml \
  board_name:=main_board board_namespace:=board_a \
  allow_motion:=false maintenance:=true allowed_motion_joints:=head_joint
```

另开终端，重复 source 与 ROS_DOMAIN_ID 设置：

```bash
ros2 service call /board_a/robot/board_identity std_srvs/srv/Trigger '{}'
ros2 topic echo /board_a/robot/motor_states --once
ros2 service call /board_a/robot/stop robot_motor_interfaces/srv/Stop "{joint_name: 'head_joint'}"
# 必须先得到 stop_confirmed=true，并确认机构已停稳
ros2 service call /board_a/robot/set_brake robot_motor_interfaces/srv/SetBrake "{joint_name: 'head_joint', engage: false}"
# 接合刹车
ros2 service call /board_a/robot/set_brake robot_motor_interfaces/srv/SetBrake "{joint_name: 'head_joint', engage: true}"
```

释放成功应 accepted=true、result_code=0、读回 brake=false；响应注明读回失败时不能认定释放完成。若返回错误108，核对 enabled/role/gear/inverted；若故障4重新出现，检查外力活动及FG，停止人工推动，不能当作单纯软件锁。

仅在确实要重新建立零点时：

```bash
ros2 service call /board_a/robot/clear_fault robot_motor_interfaces/srv/ClearFault "{joint_name: 'head_joint'}"
ros2 service call /board_a/robot/set_zero robot_motor_interfaces/srv/SetZero "{joint_name: 'head_joint'}"
ros2 topic echo /board_a/robot/motor_states --once
```

检查 fault=0、position_valid=true、persisted=true及保存标志；清故障不等于位置恢复。

需要运动时，停止维护节点，以 allow_motion=true、maintenance=false、allowed_motion_joints=head_joint 重启。机械限位未录入时需明确使用 commissioning=true、max_commissioning_step_deg=5.0。先核对板上速度；下面的400仅在板上已经保存400RPM时可执行，否则返回RPM不匹配而不会自动改速。

```bash
# 相对+5°；正向的实际机械方向仍需现场确认
ros2 action send_goal /board_a/robot/move_joint robot_motor_interfaces/action/MoveJoint \
  "{joint_name: 'head_joint', mode: 0, angle_rad: 0.0872664626, motor_rpm: 400, timeout_sec: 30.0}" --feedback
# 绝对回零；仅当与当前位置的差值不超过当前调试步长时可执行
ros2 action send_goal /board_a/robot/move_joint robot_motor_interfaces/action/MoveJoint \
  "{joint_name: 'head_joint', mode: 1, angle_rad: 0.0, motor_rpm: 400, timeout_sec: 30.0}" --feedback
```

## Verification

- 远程全量 pytest：440 passed，52.54s。日志 review_logs/full_regression.log。
- colcon build --symlink-install：3 packages finished [5.04s]。日志 review_logs/build.log。
- colcon test / test-result：449 tests, 0 errors, 0 failures, 0 skipped。日志 review_logs/colcon_test.log 和 review_logs/test_result.log；包含测试套件级统计，不应与 pytest 的440直接相加。
- 首轮新增配置拦截暴露两项右臂模拟角色数据错误，已修正测试夹具；此前两个双板测试失败也已修复。
- Flash：not actually run，本次未修改固件。
- 真实 Modbus/CAN、BRAKE电压、头部/手臂运动：not actually run，本次为软件修复与模拟回归。
- 下次验证：UID/配置/存储状态只读核对，确认白名单后执行头部单路小角度测试。
