# 人工调整模式与故障4恢复

本说明对应2026-10-10源码；需要重新编译、烧录新固件，并重新构建ROS2接口。文档示例不是已执行的硬件操作。旧固件缺少能力寄存器时，ROS2报告不支持，不发送操作码12。

## 故障4事件与5秒窗口

仅使用CONFIG成功生效的有效减速比。累计非预期FG对应输出轴活动量超过10°才形成一次事件；不是单个脉冲，也不是净偏移。516、290、82分别在第87、49、14个脉冲形成事件；未配置有效减速比不触发故障4。正常受控运动FG不计入。

首次事件停止PWM、制动、使位置无效；启动5000ms窗口，次数=1并清活动计数。后续每重新累计超过10°计一次事件，并重新清活动计数。第2次不延长窗口；第3次重新开始5000ms窗口，并作为新窗口第1次。因此持续反复超限会延长故障锁存。每轮先消费FG，再判断窗口到期；5000ms边界上的第3次不会被遗漏。

窗口到期且只有1～2次时自动清故障4，并清计数；仍PWM=0、刹车锁定、位置无效。不续跑、不设零、不额外写Flash。不能把自动恢复当成硬件正常或干扰原因已经确认。其他故障没有本自动恢复规则。原有静止/STOP不清活动累计；新增每次超限事件、自动恢复、手动进入会重置相关累计。

## 人工调整操作与限制

进入前要求整板停止且连续500ms无FG，被选通道启用且配置有效减速比，无存储错误；该路已有故障4可以由进入命令清除，其他故障拒绝。可开启多路，但新开启仍要求全板静止500ms。任一路手动中或退出等待，整板拒绝MOVE、自检、配置提交、ZERO、SAVE、变向及普通释放刹车。

首次进入先写dirty记录，成功才释放刹车；PWM保持0、DIR不变，该路位置立即无效。手动FG继续采集，不能按旧DIR更新有符号位置。该路暂停故障4累计和自动恢复；其他通道保护、捕获错误和停机输入保留。模式无倒计时，通信断开不自动退出；重启模式关闭、PWM停止、刹车锁定。请现场支撑负载后再操作。

退出、该路STOP/BRAKE=1、STOP_ALL、停机输入或该路故障立即制动，模式变为退出等待；连续500ms无FG后关闭。退出余转也不产生可信位置。最后一路退出且整板静止才统一保存，无手动阶段Flash写入。保存失败保留存储错误锁，不能声称已保存。恢复可信零点必须由操作者确认当前机械姿态后另行ZERO。

建议顺序：只读身份/能力/配置/故障/保存状态→支撑机械负载→进入所选通道→回读mode=1、PWM=0、brake=0、valid=0→手调→退出→等待mode=0、brake=1及保存三标志全0→操作者确认是否重新设零。仅解除BRAKE不会获得这些保护语义。

## Modbus完整报文

地址1，115200/8N1，HEX发送，无额外换行；以下CRC由协议实现生成。零序号方便人工测试，每次是新请求；自动客户端应使用非零序号及原去重机制。完整FC16请求收到正常8字节应答只表示接受，不表示退出静止或保存已完成。不再发旧0035提交帧。

读取能力：`01 03 00 10 00 01 85 CF`。新固件值0003：bit0手动模式，bit1本次故障4恢复规则。旧固件可能返回异常02，表示此寄存器不支持；超时、CRC错误和异常04不能当作“不支持”。

| 通道 | 进入手调 | 退出手调 | 读取模式 |
|---|---|---|---|
| M1 | `01 10 00 30 00 05 0A 00 00 00 0C 00 00 00 00 00 01 3C 08` | `01 10 00 30 00 05 0A 00 00 00 0C 00 00 00 00 00 00 FD C8` | `01 03 01 30 00 01 85 F9` |
| M2 | `01 10 00 30 00 05 0A 00 00 00 0C 00 01 00 00 00 01 01 C8` | `01 10 00 30 00 05 0A 00 00 00 0C 00 01 00 00 00 00 C0 08` | `01 03 01 70 00 01 84 2D` |
| M3 | `01 10 00 30 00 05 0A 00 00 00 0C 00 02 00 00 00 01 45 C8` | `01 10 00 30 00 05 0A 00 00 00 0C 00 02 00 00 00 00 84 08` | `01 03 01 B0 00 01 84 11` |
| M4 | `01 10 00 30 00 05 0A 00 00 00 0C 00 03 00 00 00 01 78 08` | `01 10 00 30 00 05 0A 00 00 00 0C 00 03 00 00 00 00 B9 C8` | `01 03 01 F0 00 01 85 C5` |

操作码十进制12/HEX000C；通道0～3；int32参数1进入、0退出。每路基址+0x30：0关闭、1手动中、2退出等待。普通状态+0x10新增8手动，退出等待为4 SETTLING。模式查询正常回应`01 03 02 00 xx CRClo CRChi`。

命令结果查询：`01 03 00 36 00 02 24 05`；保存标志：`01 03 00 05 00 03 15 CA`。结果0接受、1参数、2忙、3未就绪、4存储、5序号、6地址；异常帧仍沿用原映射，需读0037区分原因。若忙：确认所有运动/自检已结束并无FG500ms；未就绪：检查启用、减速比、其他故障及停止锁；存储：退出并静止后修复存储，再显式SAVE重试；序号冲突：不可将相同非零序号用于不同内容。超时先读取模式与最近结果，不能盲目用新序号重复释放。

## CAN报文

标准帧，DLC8，请求ID=0x600+node，回应ID=0x580+node。命令格式`03 seqHi seqLo 0C channel argHi argLo 00`，参数int16。

例：节点1、序号1、M1进入：ID601，`03 00 01 0C 00 00 01 00`；序号2退出：`03 00 02 0C 00 00 00 00`。应答`83 seqHi result seqLo 00 00 00 00`。读取能力例：`01 01 00 10 01 00 00 00`；读取M1模式：`01 02 01 30 01 00 00 00`。其他通道模式地址0170/01B0/01F0。非零序号重放相同内容不会重复执行，与Modbus共用业务实现。

## ROS2服务与按标签查询

进入前核对节点绑定UID及关节配置。只读节点拒绝写入；维护模式maintenance:=true允许人工调整，无需allow_motion:=true。有排队或正在执行的运动时，拒绝手动进入；排队手动进入也阻止新运动。

```bash
source /opt/ros/humble/setup.bash
source /data/ros2/robot_motor_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
# 以下是操作示例，仅在现场允许后执行；按实际板命名空间及标签替换
ros2 service call /board_b/robot/set_manual_adjust robot_motor_interfaces/srv/SetManualAdjust \
  "{joint_name: right_forearm_joint, enable: true}"
ros2 topic echo /board_b/robot/joints/right_forearm_joint/manual_adjust --once
ros2 service call /board_b/robot/set_manual_adjust robot_motor_interfaces/srv/SetManualAdjust \
  "{joint_name: right_forearm_joint, enable: false}"
```

响应accepted/sequence/result_code只描述命令；active表示模式1，exiting表示模式2，position_valid和persisted回报实际读回。进入后persisted=false是dirty保留，不等同存储失败。退出仍有其他手动通道时persisted也为false；查询dirty/storage_error/pending_save确认最终状态。退出等待超时时accepted仍可为true、exiting=true，需等待FG静止或显式STOP，不应声称退出已完成。

新ManualAdjustState话题包含joint_name、协议channel（0起）、supported、mode、active、exiting、position_valid、时间戳、identity_ok、communication_ok、stale、age_sec及保存三标志；与原状态来自同一worker快照。stale或通信失败时不能把缓存mode当作当前硬件已解锁。原消息、度数/弧度动作、Flash记录格式均不变。

## 源码与验收

核心逻辑：[robot.c](../BSP/robot.c)、[常量和RAM状态](../BSP/robot.h)。ROS2维护源：[worker](../ros2/robot_motor_driver/robot_motor_driver/scheduler.py)、[服务与话题](../ros2/robot_motor_driver/robot_motor_driver/motor_node.py)、[模拟板](../ros2/robot_motor_driver/robot_motor_driver/fake_board.py)。实际验证记录见[本次验证](verification/Robot_人工调整与故障4恢复验证.md)。

烧录后现场验收需另行授权：核对能力0003和实际引脚停止电平；分别对三种减速比测试事件阈值、1/2次到期恢复及第3次重启；有支撑地进入手动并拖动超过10°，确认无故障4且位置无效；STOP/BRAKE1/退出确认制动及500ms等待；多路退出最后保存；断电重启保持制动且位置无效。捕获错误、其他故障及存储失败必须单独验证。本次不执行这些实机步骤。
