# 历史参考：ROS2_新固件头部运动测试记录

> 保留当时方案、观测及验证结果，不是现行操作步骤。历史“当前”仅指记录当天；请使用 [文档索引](../../../GD32F303VET6_MotorControl_ROS2/README.md)。原报文与测量值不改写。

# 新固件兼容更新与头部ROS2实机测试（2026-10-08）

## 范围

远程xinhua@192.168.1.113，工作区/data/ros2/robot_motor_ros2_ws。用户确认已烧录新固件；读取协议版本2，但协议没有本次构建的唯一标识，不能仅以版本2证明二进制版本。

只对A板头部M2运动，左右臂不动。用户明确确认当前姿态设零，再以400RPM相对+5度及回零。通过真实ROS2服务和Action执行，ROS_DOMAIN_ID=173；不是模拟板，也不是直接串口命令代替ROS2。

## 软件更新及回归

真实Modbus/CAN帧格式、寄存器、故障号未变，不需改通信帧。protocol.py及fake_board.py移除固定10脉冲阈值，使用已应用减速比：516/290/82对应86/48/13个容忍脉冲，下一脉冲报故障4。未配置减速比不报警、计数饱和、暂存未提交不生效。新增三种边界和未配置/暂存测试。

- 全量pytest：444 passed in 59.36s，review_logs/fg10deg_pytest.log。
- colcon build：3 packages finished [4.90s]，review_logs/fg10deg_build.log。
- 头部配置RPM改为400后，更新旧的8000RPM配置断言，config/launch回归106 passed in 0.86s，review_logs/fg10deg_config_regression.log。
- 未重新运行整个colcon test；不得把上轮449统计当作本轮结果。

## 初始实板读取

A板UID C23B378115B092083830364A，by-path尾部0:1.2:1.0-port0，本次ttyUSB2。M1左大臂516、M2头部516、M3左小臂290、M4左手腕82，均启用且初始8000RPM。dirty=true、storage_error=false、pending_save=false；四路fault=0、刹车接合、位置无效。头部显示107.9度是无效历史计数，没有据此执行绝对运动。

B板UID C23B37C715B092083830364A，by-path尾部0:1.4:1.0-port0，本次ttyUSB0。M1/M2禁用gear516、M3禁用gear0、M4启用role2 gear516，仍不符合右臂配置。本次只读，没有修改或运动B板。证据review_logs/new_firmware_readonly.json。

## 实际动作与结果

| 操作 | 序号 | 结果 |
|---|---:|---|
| UID核对 | — | identity成功，协议2 |
| 仅M2速度改400RPM | 179 | CONFIG接受且保存，其他通道配置不变 |
| 头部当前姿态设零 | 180 | valid=true、persisted=true |
| 相对+5度 | 181 | SUCCEEDED、error_code=0、最终反馈5.0度，位置有效且保存完成 |
| 绝对回0度 | 182 | SUCCEEDED、error_code=0、最终反馈0.0度，位置有效且保存完成 |

随后独立topic读取头部DONE、fault=0、brake=true、PWM=0、rpm=0、signed FG=0。dirty/storage_error/pending_save均为false。回程最后一条Action反馈曾显示rpm=19，随后独立快照为0，保留此时序差异。左臂计数前后未变，未发送左右臂MOVE；没有外部测角，FG换算回零不能替代现场机械角度和方向确认。

头部400RPM与当前新零点已经保存，不恢复8000RPM。robot.yaml和双板YAML头部expected_motor_rpm同步为400。结束运动节点时执行正常停止清理；最终保留allow_motion=false的只读节点，域173、命名空间/board_a。最后读回头部IDLE、valid=true、fault=0、brake=true、角度0、rpm=0、保存标志全0。

## 可复制命令

每个终端先执行：

```bash
source /opt/ros/humble/setup.bash
source /data/ros2/robot_motor_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
cd /data/ros2/robot_motor_ros2_ws
```

当前只读节点允许状态查询。运动需要先停止只读节点再启动下述测试配置，同一串口不要启动两个节点：

```bash
ros2 launch robot_motor_bringup bringup.launch.py transport:=modbus_rtu \
 config_file:=/data/ros2/robot_motor_ros2_ws/src/robot_motor_bringup/config/robot_dual_UNCONFIRMED_RIGHT_ARM.yaml \
 board_name:=main_board board_namespace:=board_a allow_motion:=true \
 allowed_motion_joints:=head_joint commissioning:=true max_commissioning_step_deg:=5.0 ros_domain_id:=173
```

本轮执行的命令如下，设零和配置不应在每次运动前重复执行：

```bash
ros2 service call /board_a/robot/board_identity std_srvs/srv/Trigger '{}'
ros2 service call /board_a/robot/apply_config robot_motor_interfaces/srv/ApplyConfig "{channels: [{channel: 1, enabled: true, role: 1, gear_ratio: 516, inverted: 0, rpm: 400}], commit: true}"
ros2 service call /board_a/robot/set_zero robot_motor_interfaces/srv/SetZero "{joint_name: 'head_joint'}"
ros2 action send_goal /board_a/robot/move_joint robot_motor_interfaces/action/MoveJoint "{joint_name: 'head_joint', mode: 0, angle_rad: 0.0872664626, motor_rpm: 400, timeout_sec: 30.0}" --feedback
ros2 action send_goal /board_a/robot/move_joint robot_motor_interfaces/action/MoveJoint "{joint_name: 'head_joint', mode: 1, angle_rad: 0.0, motor_rpm: 400, timeout_sec: 30.0}" --feedback
ros2 topic echo /board_a/robot/motor_states --once
```

请求正常应答不代表运动完成；查看Action的motion_completed、error_code、persisted及位置有效性。发生错误不自动重发MOVE、不自动清故障或设零。RPM不匹配先读配置；位置无效先明确机械基准；通信UNKNOWN先核对停止状态。只读节点会拒绝运动和写入，按需重启维护/测试模式。

## 证据与未执行项目

原始记录位于review_logs：fg10deg_identity.log、fg10deg_head_config.log、fg10deg_head_zero.log、fg10deg_head_plus5.log、fg10deg_head_return0.log；动作前后及最终状态为fg10deg_before_move.yaml、fg10deg_after_move.yaml、fg10deg_final_readonly.yaml。

真实故障4阈值注入、自检、BRAKE释放手动拖动、左右臂运动、CAN实机：本轮not actually run。没有为了触发报警去推动关节。新版仍为累计活动量，不是净偏移；BRAKE释放不关闭保护，也不提供持续拖动模式。
