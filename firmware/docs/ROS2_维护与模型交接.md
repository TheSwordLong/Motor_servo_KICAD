# ROS2维护与模型交接

维护源为固件工程ros2/，同步到独立工程src/与远程/data/ros2/robot_motor_ros2_ws/src。当前程序已实现，不按历史“待实现”方案重新生成整个工程。
先阅读 [使用手册](ROS2_使用与测试手册.md)、[固件运行逻辑](Robot_固件功能与运行逻辑.md)、[通信手册](Robot_Modbus与CAN使用手册.md)，再实际读取相关代码。

| 模块（相对ros2/robot_motor_driver/robot_motor_driver） | 职责 |
|---|---|
| protocol.py | RTU/CAN帧、CRC、寄存器、单位、枚举 |
| board_session.py | UID、非零序号、去重、失联对账、只读/UNKNOWN保护 |
| scheduler.py | 唯一总线worker、队列、STOP优先、新鲜位置、限位与保存 |
| motor_node.py | 新旧动作、服务、状态与诊断 |
| config.py | YAML解析、1～4到0～3编号、限位单位冲突 |
| fake_board.py | 可注入故障的模拟板，不代表真实电气/负载 |

接口在robot_motor_interfaces，启动与配置在robot_motor_bringup；三包ament_cmake，驱动使用ament_cmake_python及ament_cmake_pytest。

## 可复制模型任务

~~~text
实际阅读当前相关源码和测试，不能只凭文档声称核对代码。
需求：【目标、参与板/关节、输入输出、通信、时序、异常、验收】
先给修改文件及方案，再提供完整新增文件或可应用补丁。
维护源为固件工程ros2/。保留MoveJointDeg、motor_states_deg和limits_deg，
旧弧度动作/消息定义不变，标准JointState仍rad。度数直接转0.1°，不乘减速比。
保留单串口worker、UID绑定、非零序号、STOP优先、UNKNOWN对账、
新鲜位置、有效零点、速度断言、软限位及Flash保护。
配置部分列表保留其他板上值，空列表也沿用板上值，不用YAML隐式覆盖。
故障4按已提交减速比累计活动>10°判断，不是净偏移；无配置不回退10脉冲。
自检残差只诊断，运动完成、位置有效和保存成功分别报告。
不自动清故障、设零、解锁BRAKE、自检、试转、烧录、重连续跑。
关键接线/方向/机械限位缺失先询问，不伪造实测。说明是否需要改固件及原因。
提供单元、模拟、构建和硬件验证步骤，记录实际执行结果；未执行要明确。
同步前备份，更新本地独立工程与远程，部署后恢复只读。
~~~

左右臂使用独立节点与namespace；没有同步轨迹、原子停机、聚合成功或完整拖动示教。修改后保留旧接口回归、度数舍入、限位冲突、取消/STOP、跨接口排队、故障及保存失败测试。
