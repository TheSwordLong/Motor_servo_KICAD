# FG历史阈值与验证记录

> 固定10脉冲属于旧版，现行规则见 [FG观察](../Robot_FG调试观察.md)。

## 历史：固定10脉冲故障4规则（已被2026-10-08角度规则替代）

当前 [robot.h](../../BSP/robot.h) 定义 `ROBOT_UNEXPECTED_FG_LIMIT=10`。每路独立累计非预期 FG：0～10 个不报警，累计第 11 个触发 `F_UNEXPECTED`（故障 4），停止输出、施加刹车并使该路位置失效。一次收到多个脉冲跨过 10 也立即触发。

- 非预期范围：普通控制中不满足“运动/余转状态且计数方向非零”的 FG，以及自检时当前被测通道之外的 FG。正常运动、被测通道自检及其余转仍完整计数，不受此阈值限制。
- 每路计数保存在 RAM，跨主循环、静止间隔、正常运动及自检往返/通道切换累计；连续静止 500 ms 不会清零。重启、成功执行该路 CLEAR_FAULT 或 ZERO 时清零；失败或忙状态拒绝的命令不清零。CONFIG、SAVE、STOP 不清零。
- 主循环和每毫秒自检保护使用同一阈值。中断在每程开始记录剩余容许量与原始计数基线，不因主循环已消费反馈而重复计算。
- 即使不超过 10，每个脉冲仍更新最后活动时间，静止确认仍要求连续 500 ms 无 FG。捕获错误、停止超时、Flash 写入窗口反馈变化保护继续生效，不能将“不报故障 4”解释为所有命令都能立即执行。
- 容忍范围内的非预期脉冲保留在原始计数和诊断累计值中，不计入有符号位置，也不单独使位置失效。单线 FG 无法确定这些脉冲的真实转向；放宽报警阈值不代表实际轴未移动，也不解决上电短转。
- 自检时其他通道累计达到 11 后，中止整批；被测通道和异常通道位置失效。已有故障仍需按原流程清除，不会因新阈值自动解除。

M2 的 Live Watch 可增加（数组下标 0～3 对应 M1～M4）：

```text
g_robot.motor[1].unexpected_fg
g_robot.motor[1].fault
g_robot.motor[1].state
g_robot.motor[1].sign
```

`unexpected_fg` 是上述业务累计值，达到 uint32 上限后饱和，不回绕；它不同于 `g_fg_debug[1].pulse_count` 原始总计数，也不同于 `pulse_delta` 或 RPM。不新增 Modbus/CAN 寄存器，Flash 格式不变。

## 阈值修改验证（2026-09-25）

Build / host tests（实际执行）：

- `powershell -ExecutionPolicy Bypass -File tools/test_robot.ps1`：11 组通过，含 10/11 边界、分次累计、通道独立、清故障/设零成功和拒绝、饱和、原始计数回绕、正常运动/余转及自检往返。[日志](../../build/unexpected-fg-tests.log)。
- `python tools/test_robot_port_selftest.py`：通过；使用实际端口函数并模拟 IO，覆盖主循环已消费脉冲时的中断阈值、返程剩余容许量、截止快照和捕获错误。[日志](../../build/unexpected-fg-port.log)。这不是实体电气或中断延迟测量。
- `cmake --build --preset debug`：通过；首次 Ninja 受沙箱限制，获准后重跑。[日志](../../build/unexpected-fg-cmake.log)。
- `powershell -ExecutionPolicy Bypass -File tools/build_eide.ps1`：53 个对象构建通过。[日志](../../build/unexpected-fg-eide.log)。两套 size 均为 text=23088、data=20、bss=10788；已有 RWX 警告仍在。

Flash / Monitor / hardware manual checks：**not actually run**。未烧录、连接串口或启动电机。

Next verification step：人工下载对应固件，安全停止条件下用受控 FG 信号逐个输入，确认第 10 个仍无故障、第 11 个报故障 4；停稳后清故障，确认业务累计归零而原始 FG 总计数保留。实体转动、上电扰动及自检中的其他通道测试须在机械活动空间允许时进行。硬件是否适合此容许量仍待实测。

## 历史：FG 观察变量验证（2026-09-25）

以下尺寸和哈希属于增加观察变量时的版本；当前阈值修改构建见上一节。

Build:

- CMake `cmake --build --preset debug`：通过，首次 Ninja 沙箱受限后获准重跑。[日志](../../build/fgdebug-cmake.log)。
- EIDE `powershell -ExecutionPolicy Bypass -File tools/build_eide.ps1`：通过，53 个对象。[日志](../../build/fgdebug-eide.log)。
- 两套 size：text=23104、data=20、bss=10756；EIDE FLASH=23124 B、RAM=10768 B。既有 RWX 和 USB 未使用变量警告仍存在。
- 两套 ELF 都包含 144 字节 BSS 全局符号 `g_fg_debug`，反汇编确认 main 调用刷新函数。[符号与 SHA-256 记录](../../build/fgdebug-symbol-check.log)。仅构建/链接检查，不是实体 FG 或采样时序测试；本轮未重跑业务回归。

Flash / Monitor / Manual checks: **not actually run**。未下载、复位、连接调试器/串口或启动电机。

Next verification step: 人工下载后使用对应 ELF，在安全停止条件下先观察 M2 静止时 count/level/age；之后以受控 FG 信号检查计数和转速更新。实际硬件结果待记录。
