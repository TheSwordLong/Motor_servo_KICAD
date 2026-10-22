# FG 全局调试变量

连续角度测试按[Modbus/CAN 多次 MOVE 流程](Robot_免序号操作.md#repeated-angle)进行。M2 可同时观察 `g_robot.motor[1].position`、`target`、`state`、`fault`、`valid`、`unexpected_fg`；position/target 是有符号 FG 脉冲，不是度数。原始 `g_fg_debug[1].pulse_count` 在正反转时都累计增加，不能用它是否回零判断回位。正常 MOVE 与允许余转计数不会消耗非预期 FG 容许量，也不会自动清除已有累计；不要在每次动作前清故障或设零来掩盖累计。

下载本次固件并使用同一次构建的 ELF 调试，在 Watch 中添加 `g_fg_debug`，展开四个元素即可。也可直接添加以下 M2 表达式：

```text
g_fg_debug[1].pulse_count
g_fg_debug[1].pulse_delta
g_fg_debug[1].level
g_fg_debug[1].speed_rpm
g_fg_debug[1].errors
g_fg_debug[1].age_ms
g_fg_debug[1].sample_ms
```

| 数组元素 | 通道 | FG 引脚 |
|---|---|---|
| g_fg_debug[0] | M1 | PB10 |
| g_fg_debug[1] | M2 | PE11 |
| g_fg_debug[2] | M3 | PB4 |
| g_fg_debug[3] | M4 | PA0 |

| 字段 | 含义 |
|---|---|
| pulse_count | 启动后累计捕获的 FG 上升沿数，uint32 回绕；不区分正反转，不是位置角度 |
| pulse_delta | 相比上一次调试采样新增的脉冲数；首次为 0。采样间隔不固定，不能直接把它当转速 |
| level | 采样瞬间 GPIO 输入，0 低、1 高；不是示波器波形，可能漏看两次采样之间的跳变 |
| speed_rpm | 减速前电机转速；连续超过 250 ms 无 FG 后显示 0；刚收到第一个边沿尚不能计算周期 |
| errors | 原始捕获溢出累计次数，不是 robot 的业务故障码，也不代表所有电气干扰都能被检测 |
| period_us | 最近测得的 FG 周期，微秒；停转后保留历史值，以 speed_rpm/age_ms 辅助判断 |
| last_edge_ms | 最近 FG 边沿对应的系统运行毫秒 |
| age_ms | 采样时距离上次边沿的毫秒数；启动后尚无边沿时等于系统运行时间 |
| sample_ms | 此次采样的系统运行毫秒，不是电脑日历时间；主循环停住后不再刷新 |

定义在 [capture.c](../BSP/capture.c)，类型声明在 [capture.h](../BSP/capture.h)。变量为 volatile；[main.c](../Core/Src/main.c) 在每轮控制处理后调用 `capture_debug_update()`，函数内部间隔至少 10 ms 才采样。若主循环因 Flash 等工作延迟，采样间隔会增大。短暂关闭中断只用于读取一致的原始数据，恢复中断后发布观察副本；控制逻辑不读取调试副本。

原有 `g_fg[4]` 继续保留，由捕获中断更新。其 `speed_rpm` 是最后一次边沿计算值，停转后不会自行清零；新变量提供超时处理后的转速。调试器跨字段读取可能跨越一次刷新，不保证整个 Watch 窗口为同一时刻的快照。不要修改观察变量来模拟真实反馈。

观察静止误脉冲时先确认实体轴确实未动，再看 `pulse_count` 是否增长；增长说明捕获到了边沿，但不能仅凭此判断来自真实转动、上电跳变还是干扰。`level` 持续高/低不能单独证明电机是否连接。M2 故障码另看 `g_robot.motor[1].fault`；清业务故障不会清原始累计计数/捕获错误。

若调试器支持运行时观察，可让程序保持运行查看；若只在暂停时刷新，先确保动力已断开或电机已可靠停止再暂停。**电机运行时不要用断点暂停来停机**，暂停 CPU 可能阻止软件截止保护；暂停后的时序、捕获和恢复现象不能当正常运行实测结论。

## 当前故障4与Watch

g_robot.motor[0..3].unexpected_fg是非预期活动累计，uint32饱和，不回绕；不是有符号净偏移，也不同于原始pulse_count。
已CONFIG生效的gear=516/290/82分别容忍86/48/13个脉冲，第87/49/14个触发：累计输出轴活动≤10°不报警，>10°报警。未配置有效减速比不回退固定10脉冲保护。
正常受控运动、自检被测通道及允许余转不消耗该预算。静止、普通运动、CONFIG、STOP、SAVE不清累计；初始化/重启、成功CLEAR_FAULT或ZERO清累计；本版每次超限事件、自动恢复或人工调整进入也重置相关累计。
可观察recovery_events（0/1/2）、recovery_ms（窗口起点ms）、manual（0/1/2）。故障锁存后unexpected_fg用于下一个事件的活动预算，不能把原始pulse_count每次增长都当新事件。
未配置期间也累计活动，提交配置不清除旧累计；需要新观察周期时在静止后显式清故障。10°以内不因本规则改变有效位；超过阈值停止PWM、刹车、位置失效并安排保存。捕获错误和静止确认规则不变。

~~~text
g_robot.motor[1].unexpected_fg
g_robot.motor[1].fault
g_robot.motor[1].state
g_robot.motor[1].sign
~~~

主循环与自检1ms保护预算一致，见 [固件运行逻辑](Robot_固件功能与运行逻辑.md)。旧10/11脉冲测试见 [历史记录](history/Robot_FG历史验证.md)，不能当作本版验收。


## 2026-10-10人工调整与故障4恢复更新

新增操作码12、能力寄存器0010和每路+30模式查询。故障4每次重新累计>10°构成事件：5000ms内1～2次到期自动清除，第3次重启窗口；保持停止、制动及位置无效。人工调整暂停该路故障4、冻结有符号位置，并独占整板运动。完整报文、ROS2服务、退出及保存步骤以[人工调整与故障4恢复](Robot_人工调整与故障4恢复.md)为准，历史日志不改写。
