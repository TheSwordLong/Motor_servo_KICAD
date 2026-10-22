# ROS2头部与双臂电机控制

三个robot_motor_*包支持UID绑定、度数/兼容弧度动作、状态、自检、配置保存、设零、清故障、停止和BRAKE。每板独立进程/worker，默认只读。
维护源为固件工程ros2/；独立工程src/与远程src/是同步目标，不要只修改副本。

- [网页与 API](robot_motor_web/README.md)：无需登录的中文控制台，默认只读，REST/WebSocket、节点管理与断线制动。
- [使用与测试手册](../docs/ROS2_使用与测试手册.md)：头部优先、左右臂、angle_deg、双板、BRAKE和错误处理。
- [维护与模型交接](../docs/ROS2_维护与模型交接.md)
- [验证记录](../docs/ROS2_度数接口验证记录.md) · [文档索引](../docs/README.md) · [历史记录](../docs/history/README.md)

~~~bash
source /opt/ros/humble/setup.bash
source /data/ros2/robot_motor_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
ros2 topic echo /board_a/robot/joints/head_joint/state --field angle_deg --once
ros2 topic echo /board_a/robot/motor_states_deg --once
~~~

运动参数angle_deg直接填度数，motor_rpm=0沿用板上速度，非零只校验；绝对mode=1需要可信零点。旧MoveJoint/angle_rad仍可用，标准JointState仍rad。
无同步轨迹或跨板原子停机，停止两板需分别确认。新增关节状态话题不写板上配置，不自动运动、不烧录。

关节状态按YAML标签发布robot/joints/<joint_name>/state，类型MotorStateDeg；头部、左右臂分别查询其板命名空间。原整板话题保留，通信新鲜度仍需看整板状态。

按标签话题构建及模拟测试：[验证记录](../docs/verification/joint_topics_20261009/验证记录.md)。

2026-10-10：右手腕软件绑定改为B/M2，右大臂M1、右小臂M3保持；B/M4待禁用。B板未连接，板端迁移和实机验证待后续完成。


新增[人工调整与故障4次数恢复](../docs/Robot_人工调整与故障4恢复.md)：手动调整服务及按关节查询话题，操作码12；需新固件和重新构建ROS2。仅解除BRAKE不等于人工调整模式。
