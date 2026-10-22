# 历史参考：protocol_contract

> 保留当时方案、观测及验证结果，不是现行操作步骤。历史“当前”仅指记录当天；请使用 [文档索引](../../README.md)。原报文与测量值不改写。

> 2026-10-08 故障4规则更新：按已生效减速比计算累计非预期活动量，≤10°不报警、>10°报警；516/290/82分别在第87/49/14个脉冲触发，未配置有效减速比不触发。本文原固定10/11脉冲说明属于旧版，当前规则、运行逻辑和验证见[固件功能与运行逻辑](../Robot_固件功能与运行逻辑.md)。

# GD32F303 电机板 Modbus RTU / CAN 协议契约（阶段 A）

生成日期：2026-09-29。依据任务书：[ROS 2 代码生成执行方案](ROS2_头部与双臂控制_代码生成执行方案.md) 第 8 节阶段 A，为阶段 A 的第二份交付物（第一份为 [ROS 2 旧工程兼容审查](compatibility_audit.md)）。

**本文的性质与证据边界（必读）**

1. 本文全部结论来自**源码审查**，即逐行阅读下列文件后提取，每条结论后附 `（依据：文件:行号）`：`BSP/robot.h`、`BSP/robot.c`、`BSP/robot_protocol.c`、`BSP/robot_protocol.h`、`BSP/robot_port.c`、`BSP/robot_rtu.c`、`BSP/robot_rtu.h`、`BSP/robot_store.c`、`BSP/robot_store.h`、`BSP/usart0.c`、`BSP/usart0.h`、`BSP/pwm.c`、`BSP/motor_io.c`、`BSP/motor_pins.h`、`BSP/capture.h`、`BSP/capture.c`、`BSP/board_config.h`、`tools/robot_host.py`。客户端参考实现以 `tools/robot_host.py` 为准。
2. 本文数值**本轮未在实板上验证**。本轮对实板只做了一次**只读**回包核对（功能码 03，无任何写命令、无运动）：板 UID `C23B378115B092083830364A`、`0x000F=2`、`0x0005/0x0006/0x0007=0/0/0`、`0x0036/0x0037=144/0`、四路 `fault=0`，以及 M2/M4 的“角度寄存器 = FG 计数按第 7 节公式换算”的一致性（928 → 1079，−142 → −1039）。这些只读事实见第 14.2 节，**不能**用来证明板上二进制与本地源码一致（未读取固件哈希），也不能证明任何写路径、时序、超时或互锁行为。
3. 本次会话中 `git rev-parse HEAD` 因仓库所有权保护（`detected dubious ownership`）未能读取，因此本文不给出提交哈希；上表所列源码文件的本地修改时间为 2026-09-25 至 2026-09-29 之间（`BSP/robot.h`、`BSP/robot.c` 为 2026-09-29 09:34，`BSP/robot_port.c` 为 2026-09-29 13:45）。若后续源码变动，本文所有行号与数值必须重新核对。
4. 面向读者：要编写上位机 / ROS 2 客户端的工程师。文中 **MUST / MUST NOT** 表示客户端实现义务（汇总见第 13 节）；“固件行为”表示板端已有实现，客户端不能假定它会改变。
5. 约定：寄存器地址均为 **PDU 零基地址**（如 `0x0100`，不是 40001 编号）；16 位值高字节先发；32 位值高字在前；有符号数用二进制补码；角度单位 **0.1°（ddeg）**；`ch` 为 0～3（对应面板/文档中的 M1～M4）。

---

## 1. 传输与成帧

### 1.1 传输选择

| 项 | 值 | 依据 |
|---|---|---|
| Modbus RTU 物理口 | USART0，PA9=TX / PA10=RX（非重映射） | `BSP/usart0.h:9-13`、`BSP/robot_port.c:226` |
| 串口参数 | 115200、8 数据位、无校验、1 停止位 | `BSP/usart0.h:23`、`BSP/usart0.c:47-50` |
| CAN | CAN0，PD0=RX / PD1=TX，全重映射，500 kbit/s（60 MHz /(8×(1+12+2))），标准帧 | `BSP/can_bus.h:8`、`BSP/can_bus.c:75-81` |
| 传输二选一 | `protocol==0` → 只初始化并轮询 USART0；否则只初始化并轮询 CAN | `BSP/robot_port.c:226`、`BSP/robot_port.c:253` |

**固件行为**：传输选择在 `robot_port_init()` 完成一次，并在每轮 `robot_port_update()` 依据 `g_robot.protocol` 二选一（依据：`BSP/robot_port.c:226`、`BSP/robot_port.c:253`）。而 `r->protocol`、`r->address`、`r->node` **只在 `robot_init()` 中赋值一次**（依据：`BSP/robot.c:69-70`），CONFIG 提交只更新 `r->config`（依据：`BSP/robot.c:400`）。因此：

- 改写暂存区 `0x0020`（protocol）/`0x0021`（address）/`0x0022`（node）并 CONFIG 成功后，**当前会话的帧地址、CAN 节点号与传输类型都不变**，必须重启才生效（依据：`BSP/robot.c:69-70`、`BSP/robot.c:400`、`BSP/robot_port.c:226`）。
- 唯一在 CONFIG 中立即生效的板级参数是 `pwm_hz`：`r->ops.frequency(next.pwm_hz)` 在提交时调用（依据：`BSP/robot.c:406`）。
- 读 `0x0001/0x0002/0x0003` 返回的是**运行时生效值**（`r->protocol/r->address/r->node`），读 `0x0020/0x0021/0x0022` 返回的是**暂存值**；两者在 CONFIG 之后、重启之前必然不一致（依据：`BSP/robot.c:501-503`、`BSP/robot.c:511-512`）。

### 1.2 RTU 成帧判定（固件侧）

固件不使用“3.5 字符时间”，而是用微秒级绝对间隔判定（时间戳在接收中断中按字节打戳，时间基准 1 µs）：

| 条件 | 行为 | 依据 |
|---|---|---|
| 与上一字节间隔 ≥ **1750 µs** | 结束上一帧并交付解析（若该帧未被标脏） | `BSP/robot_rtu.c:10`、`BSP/robot_rtu.c:17-18` |
| 间隔 > **750 µs** 且 < 1750 µs | 当前帧标记为 `bad`，**整帧丢弃且不回复** | `BSP/robot_rtu.c:11` |
| 字节带错误标志（`USART_STAT0` 低 4 位任一置位：校验/帧/噪声/溢出类） | 当前帧标记为 `bad`，整帧丢弃且不回复 | `BSP/usart0.c:91`、`BSP/robot_rtu.c:13` |
| 帧长超过 **256 字节** | 超出即标记 `bad`，整帧丢弃 | `BSP/robot_rtu.h:5`、`BSP/robot_rtu.c:14-15` |
| 接收环形缓冲（**512 字节**）溢出 | 溢出字节被丢弃并计数；`poll_serial()` 检测到计数变化后丢弃整个接收队列并把当前帧标脏 | `BSP/usart0.c:16`、`BSP/usart0.c:79-87`、`BSP/robot_port.c:166-173` |

参考换算：115200 8N1 下 1 字符 ≈ 86.8 µs，因此 750 µs ≈ 8.6 字符时间、1750 µs ≈ 20 字符时间——比 Modbus 规范的 1.5/3.5 字符时间宽松得多，标准 3.5 字符静默足以分帧（依据：`BSP/usart0.h:23`、`BSP/robot_rtu.c:10-11`）。每轮主循环最多消费 512 个接收字节，然后才做静默判定；静默判定先取时间再判队列空，避免新到字节造成提前分帧（依据：`BSP/robot_port.c:175-180`）。

### 1.3 ADU 结构与“完全无应答”的情形

`robot_modbus()` 的入口门槛：`n<4`、`n>256`、`q[0] != r->address`、或整帧 CRC16 校验不为 0（即 CRC 附在尾部后按标准算法算得残值为 0）——**任一成立即返回 0，不发送任何字节**（依据：`BSP/robot_protocol.c:16`）。CRC16 为标准 Modbus CRC-16：初值 `0xFFFF`、反射多项式 `0xA001`、**低字节先发**（依据：`BSP/robot_protocol.c:3-8`、`BSP/robot_protocol.c:39`；客户端同实现见 `tools/robot_host.py:40-46`、`tools/robot_host.py:57`）。

除 CRC/地址/长度错误外，还有两类**链路层丢弃**同样表现为完全静默：

- 发送侧仍忙（上一应答未发完，或 TC 未置位）→ 本次请求直接丢弃（依据：`BSP/robot_port.c:160-164`、`BSP/usart0.c:112-114`、`BSP/usart0.c:122-123`）。
- 检测到接收溢出计数变化 → 本次请求直接丢弃（依据：`BSP/robot_port.c:162`、`BSP/robot_port.c:168-173`）。

**因此“无应答”在本协议中是多义的**：可能是帧被丢弃、可能是板子正忙、也可能是应答已发出但主机没收到（例如掉电/拔线）。客户端 MUST NOT 把静默解释为“未执行”（详见第 4.5、13 节）。广播地址 0 永不匹配（`r->address ≥ 1`），因此广播帧既不执行也不回复（依据：`BSP/robot_protocol.c:16`、`BSP/robot.c:13`）。

### 1.4 支持的功能码与长度校验

| 功能码 | 请求长度约束 | 数量约束 | 应答 | 依据 |
|---|---|---|---|---|
| `03` 读寄存器 | 必须恰好 8 字节 | `1 ≤ count ≤ 125` | `addr,03,bytecount,data…,CRC`，`bytecount = 2×count` | `BSP/robot_protocol.c:18-25`、`BSP/robot.c:560` |
| `06` 写单寄存器 | 必须恰好 8 字节 | — | 原样回显请求前 6 字节 + CRC | `BSP/robot_protocol.c:26-28` |
| `10` 写多寄存器 | `n ≥ 9` 且 `n == 9 + 2×count` | `1 ≤ count ≤ 123`，且 `q[6] == 2×count` | 回显 `addr,func,startHi,startLo,countHi,countLo` + CRC | `BSP/robot_protocol.c:29-36` |
| 其他 | — | — | 异常码 **01**（非法功能） | `BSP/robot_protocol.c:37` |

长度/字节数不匹配一律映射为 `R_VALUE` → 异常码 **03**，不会静默截断（依据：`BSP/robot_protocol.c:19`、`BSP/robot_protocol.c:27`、`BSP/robot_protocol.c:30`、`BSP/robot_protocol.c:33`、`BSP/robot_protocol.c:11-12`）。读寄存器的地址范围检查为 `count ≥ 1 && count ≤ 125 && addr+count ≤ 65536`（依据：`BSP/robot.c:560`）；写路径为 `count ≥ 1 && count ≤ 123 && addr+count ≤ 65536`（依据：`BSP/robot.c:578`）。

### 1.5 客户端交易参数（参考实现）

`tools/robot_host.py` 的做法可作为下限：串口 `timeout=0.05 s`、`write_timeout=0.5 s`（依据：`tools/robot_host.py:53`）；每次发送前 `sleep(0.004)` 并清空接收缓冲（依据：`tools/robot_host.py:59-61`）；单次交易截止 **1.5 s**，注释明确原因是“Flash 回滚可能延迟应答”（依据：`tools/robot_host.py:64`）；最多 **3 次**重试，且重试必须使用**完全相同的字节与相同的非零序号**（依据：`tools/robot_host.py:58-59`）；应答长度按功能码推导（异常 5 字节、`03` 为 `5+bytecount`、`06/10` 为 8 字节），并校验 CRC、站号回显、功能码回显、`06/10` 的字段回显（依据：`tools/robot_host.py:69`、`tools/robot_host.py:72-79`）。三次都无有效应答时抛超时，并明确提示“操作可能已完成，不要盲目新建运动请求”（依据：`tools/robot_host.py:81`）。

---

## 2. 寄存器映射

### 2.1 板级寄存器（0x0000～0x0037）

| 地址 | 名称 | 访问 | 内容 | 依据 |
|---:|---|---|---|---|
| `0x0000` | Flash schema | R | 常量 `1`（持久化镜像版本，**不是**命令协议版本） | `BSP/robot.c:500`、`BSP/robot.h:6` |
| `0x0001` | protocol（生效） | R | `0`=Modbus、`1`=CAN；重启后才随 CONFIG 改变 | `BSP/robot.c:501`、`BSP/robot.c:69`、`BSP/robot.c:13` |
| `0x0002` | address（生效） | R | 当前 Modbus 站号，`1..247` | `BSP/robot.c:502`、`BSP/robot.c:69` |
| `0x0003` | node（生效） | R | 当前 CAN 节点号，`1..127` | `BSP/robot.c:503`、`BSP/robot.c:70` |
| `0x0004` | pwm_hz（生效） | R | 当前 PWM 频率，`20000..30000`；CONFIG 提交即刻生效 | `BSP/robot.c:504`、`BSP/robot.c:406` |
| `0x0005` | dirty | R | 1 = Flash 中的位置记录不可信（详见第 8 节） | `BSP/robot.c:505` |
| `0x0006` | storage_error | R | 1 = 最近一次落盘失败 | `BSP/robot.c:506` |
| `0x0007` | pending_save | R | 1 = 有位置变化等待落盘 | `BSP/robot.c:507` |
| `0x0008`～`0x000D` | UID | R | 3 个 32 位字共 6 寄存器，**高字在前** | `BSP/robot.c:508`、`BSP/robot_port.c:221-222` |
| `0x000E` | have_seq | R | 1 = 板端已记录过序号（0 表示序号窗口检查被跳过） | `BSP/robot.c:509`、`BSP/robot.c:484-485` |
| `0x000F` | 命令协议版本 | R | 常量 `2`（单帧命令协议） | `BSP/robot.c:510`、`BSP/robot.h:7-8` |
| `0x0010`～`0x001F` | — | **未映射** | 读/写均返回 `R_ADDRESS` → 异常码 02；**保留用途在源码中未找到** | `BSP/robot.c:497-556`、`BSP/robot.c:564-568` |
| `0x0020`～`0x0023` | 暂存板级配置 | **RW** | 依次 protocol、address、node、pwm_hz；需 CONFIG 提交 | `BSP/robot.c:511-512`、`BSP/robot.c:566`、`BSP/robot.c:581-584` |
| `0x0024`～`0x002F` | — | **未映射** | 异常码 02 | `BSP/robot.c:511`、`BSP/robot.c:555` |
| `0x0030`～`0x0034` | 命令快照 / 原子命令入口 | R；**W 仅限整块 5 寄存器** | 读＝最近一次收到的完整命令帧内容；写＝`seq,op,ch,arg_hi,arg_lo` 并**立即执行** | `BSP/robot.c:513`、`BSP/robot.c:573-577` |
| `0x0035` | 旧提交寄存器（墓碑） | R（恒 0）；**W 被拒** | 写入返回 `R_ADDRESS` → 异常码 02；旧 `A55A` 提交流程已废除 | `BSP/robot.c:514`、`BSP/robot.c:564-568`、`BSP/robot.c:579` |
| `0x0036` | last_seq | R | 板端**实际使用**的最近序号（`seq=0` 时为板端分配值） | `BSP/robot.c:515`、`BSP/robot.c:494` |
| `0x0037` | last_result | R | 该序号的业务结果码（第 5.1 节） | `BSP/robot.c:516`、`BSP/robot.c:494` |
| `0x0038`～`0x00FF` | — | **未映射** | 异常码 02 | `BSP/robot.c:555` |

**关键细节**

- 命令快照 `0x0030～0x0034` 在**验证与执行之前**就被 `memcpy` 覆盖（依据：`BSP/robot.c:573-574`）。因此读到一条命令快照**不能**证明该命令被接受；真伪只能看 `0x0036/0x0037`（依据：`BSP/robot.c:494`）。
- 只有 `addr==0x0030 && count==5` 这一种写法会触发执行（依据：`BSP/robot.c:573`）。对 `0x0030` 的 `06` 单寄存器写、或 `count≠5` 的 `10` 写，都会落到通用路径并因 `writable()` 为假而返回异常码 02（依据：`BSP/robot.c:578-579`、`BSP/robot.c:564-568`）。跨边界写（如从 `0x002F` 起写 6 个）同样被拒（依据：`BSP/robot.c:573`、`BSP/robot.c:579`）。
- 多寄存器读**遇到第一个未映射地址就整帧失败**，不返回部分数据（依据：`BSP/robot.c:561`）。因此 `0x000D..0x0010` 这类跨空洞读取会得到异常码 02，必须拆成两次读。
- 兼容审查中提出的两个具体问题的答案：旧栈的 `01 03 00 02 00 01`（CRC `25 CA`）读回的是**板级 address（站号）**，与位置无关（依据：`BSP/robot.c:502`）；旧栈的 `01 06 00 01 00 02`（CRC `59 CB`）写 `0x0001` 会因 `writable()` 为假返回 `R_ADDRESS` → **Modbus 异常码 02**（依据：`BSP/robot.c:564-568`、`BSP/robot.c:579`、`BSP/robot_protocol.c:11-12`）。这回答了 [compatibility_audit.md](compatibility_audit.md) 第 3 节末尾“具体异常码待协议契约核对”。

### 2.2 通道块（`B = 0x0100 + 0x40×ch`，ch = 0..3）

通道号由 `i=(addr-0x100)/0x40` 推出，偏移 `off=(addr-0x100)%0x40`（依据：`BSP/robot.c:517-518`）。四路基址：M1=`0x0100`、M2=`0x0140`、M3=`0x0180`、M4=`0x01C0`。

| 偏移 | 名称 | 访问 | 内容 | 依据 |
|---:|---|---|---|---|
| `B+0x00`～`B+0x04` | 生效配置 | R | enabled、role、gear、inverted、rpm（**减速前**巡航转速） | `BSP/robot.c:520-522`、`BSP/robot.h:53` |
| `B+0x05`～`B+0x07` | — | **未映射** | 异常码 02 | `BSP/robot.c:554` |
| `B+0x08`～`B+0x0C` | 暂存配置 | **RW** | 同上五项；需 CONFIG 提交 | `BSP/robot.c:520`、`BSP/robot.c:523`、`BSP/robot.c:567`、`BSP/robot.c:586-589` |
| `B+0x0D`～`B+0x0F` | — | **未映射** | 异常码 02 | `BSP/robot.c:554` |
| `B+0x10` | state | R | 运动状态 0..7（第 6.1 节） | `BSP/robot.c:524` |
| `B+0x11` | fault | R | 故障码 0..7（第 6.2 节） | `BSP/robot.c:525` |
| `B+0x12` | valid | R | 1 = 拥有可信零点参考 | `BSP/robot.c:526`、`BSP/robot.h:58` |
| `B+0x13` | brake | R | 1 = 刹车施加 | `BSP/robot.c:527` |
| `B+0x14` | direction | R | DIR 电平（0/1），MOVE 会覆盖 | `BSP/robot.c:528`、`BSP/robot.c:458` |
| `B+0x15` | duty | R | **低电平千分比** 0..1000 | `BSP/robot.c:529`、`BSP/robot.h:75-76`、`BSP/pwm.c:12-16` |
| `B+0x16` | rpm | R | 实测电机转速（减速前、**无符号幅值**）；>250 ms 无 FG 边沿则为 0 | `BSP/robot.c:530`、`BSP/capture.c:262`、`BSP/capture.h:46` |
| `B+0x17` | — | **未映射** | 异常码 02 | `BSP/robot.c:554` |
| `B+0x18`～`B+0x19` | 当前角度 | R | int32，0.1°，由 FG 计数实时换算 | `BSP/robot.c:531-533`、`BSP/robot.c:81-89` |
| `B+0x1A`～`B+0x1B` | FG 位置计数 | R | int32，有符号边沿累计，**不模一圈** | `BSP/robot.c:531-532`、`BSP/robot.h:57` |
| `B+0x1C`～`B+0x1D` | 目标角度 | R | int32，0.1°，由目标计数换算 | `BSP/robot.c:531-533` |
| `B+0x1E`～`B+0x1F` | — | **未映射** | 异常码 02 | `BSP/robot.c:554` |
| `B+0x20` | 自检 state | R | 0..7（第 6.3 节） | `BSP/robot.c:537` |
| `B+0x21` | 自检 reason | R | 0..10（第 6.4 节） | `BSP/robot.c:538` |
| `B+0x22`～`B+0x23` | 累计脉冲 | R | uint32 高/低字 | `BSP/robot.c:539-540` |
| `B+0x24` | powered_ms | R | 去程通电毫秒（uint16，饱和于 65535） | `BSP/robot.c:541`、`BSP/robot.c:116` |
| `B+0x25`～`B+0x26` | completed_ms | R | 完成时刻（板端运行毫秒，uint32 高/低字） | `BSP/robot.c:542-543` |
| `B+0x27` | 自检序号 | R | 启动该次自检的请求序号 | `BSP/robot.c:544`、`BSP/robot.c:179` |
| `B+0x28` | phase | R | 往返阶段 0..5（第 6.5 节） | `BSP/robot.c:545` |
| `B+0x29` | return_ms | R | 回程通电毫秒（uint16） | `BSP/robot.c:546`、`BSP/robot.c:118` |
| `B+0x2A`～`B+0x2B` | 去程脉冲 | R | uint32 高/低字 | `BSP/robot.c:547-548` |
| `B+0x2C`～`B+0x2D` | 回程脉冲 | R | uint32 高/低字 | `BSP/robot.c:549-550` |
| `B+0x2E`～`B+0x2F` | residual | R | int32，有符号残差（**仅诊断，不判失败**） | `BSP/robot.c:551-552`、`BSP/robot.c:289` |
| `B+0x30`～`B+0x3F` | — | **未映射** | 异常码 02 | `BSP/robot.c:554` |

自检结果块（`B+0x20`～`B+0x2F`）是 **RAM-only**，`robot_image_t` 与 Flash schema 有意不包含它（依据：`BSP/robot.h:83-84`）；`robot_init()` 对整个 `robot_t` 做 `memset`，故每次上电自检记录清零（依据：`BSP/robot.c:65`）。CONFIG 提交时，凡 enabled/gear/role/inverted 变化的通道，其自检记录会被清零（依据：`BSP/robot.c:395-396`、`BSP/robot.c:402`）。

### 2.3 可写地址的完整集合

`writable()` 只允许两类：`0x0020..0x0023`，以及通道块内 `off ∈ [8,12]`（依据：`BSP/robot.c:564-568`）。写请求中**任一**地址不可写，整帧返回 `R_ADDRESS` → 异常码 02，且不做部分写入（依据：`BSP/robot.c:579`）。加上第 2.1 节的原子命令入口（`0x0030` 起 5 个寄存器，依据：`BSP/robot.c:573`），这就是全部合法写入。

---

## 3. 命令帧与操作码

### 3.1 单帧命令

唯一执行入口：功能码 `10`，起址 `0x0030`，数量 `5`，数据 `seq, op, ch, arg_hi, arg_lo`；`arg` 为**有符号 int32**（`arg = (v[3]<<16) | v[4]`），收到即尝试执行，无二次提交（依据：`BSP/robot.c:573-577`、`BSP/robot_protocol.c:29-36`）。正常应答是请求前 6 字节的回显：`01 10 00 30 00 05` + CRC（依据：`BSP/robot_protocol.c:35`、`BSP/robot.c:576` 返回 `R_OK`）。**应答被接受 ≠ 运动完成 ≠ 已保存**（依据：`tools/robot_host.py:283`）。

暂存配置写入是**普通写**，不消耗序号、不触发执行：`0x0020` 起 4 个（板级）或 `B+0x08` 起 5 个（通道），随后必须发一条 CONFIG 命令才提交（依据：`BSP/robot.c:580-591`、`BSP/robot.c:386-407`、`tools/robot_host.py:214-219`）。

### 3.2 操作码表

`op` 取值 1..11（依据：`BSP/robot.h:30-31`；客户端同表见 `tools/robot_host.py:15-17`）。

| op | 名称 | ch | arg | 前置条件（不满足即拒绝） | 成功语义 | 依据 |
|---:|---|---|---|---|---|---|
| 1 | CONFIG | 忽略 | 忽略 | 整板静止（`quiet()`）；暂存配置整体合法 | 先落盘再提交；`dirty=0`、`pending_save=0`、`storage_error=0`；变更通道的 `valid` 清零、自检记录清零；立即应用新 `pwm_hz` | `BSP/robot.c:386-407` |
| 2 | ZERO | 0..3 | 忽略 | 整板静止；该路 `gear≠0` | 该路 `position=0`、`target=0`、`valid=1`、`fault=0`、`state=IDLE`，并落盘（`dirty=0`）；清该路意外脉冲预算与停止确认故障位 | `BSP/robot.c:429-438` |
| 3 | MOVE | 0..3 | **−3600..3600**（0.1°，相对） | 该路不忙；无 fault；`enabled=1` 且 `gear≠0`；该路 ≥500 ms 无边沿；无 `storage_error`；目标计数不溢出 int32；若 `dirty=0` 还需整板静止以落盘 `dirty=1` | 相对当前计数运动；`direction` 由符号与 `inverted` 决定；`state=RUNNING`、`brake=0`；设置监控超时 | `BSP/robot.c:440-464` |
| 4 | STOP | 0..3 | 忽略 | 无（自检中会取消自检） | `duty=0`、`brake=1`、`completing=0`；忙则进入 SETTLING，否则 FAULT/IDLE/UNREADY | `BSP/robot.c:411`、`BSP/robot.c:90-98`、`BSP/robot.c:369-372` |
| 5 | STOP_ALL | 忽略 | 忽略 | 无 | 取消自检（记为 CANCELLED）+ 四路同时 STOP | `BSP/robot.c:366`、`BSP/robot.c:144-148` |
| 6 | BRAKE | 0..3 | 0 或 1 | arg 必须 0/1；释放（arg=0）要求该路不忙、无 fault | arg=1 等价 STOP；arg=0 仅释放刹车（`brake=0`） | `BSP/robot.c:412-417` |
| 7 | DIRECTION | 0..3 | 0 或 1 | arg 必须 0/1；该路不忙且 ≥500 ms 无边沿；无停止确认故障位 | 直接写 DIR 电平并输出 | `BSP/robot.c:418-421`、`BSP/robot.c:377-379` |
| 8 | SAVE | 忽略 | 忽略 | 整板静止 | 以 `dirty=0` 落盘当前位置/valid/序号；`pending_save=0`；**不提交暂存配置** | `BSP/robot.c:381-385`、`BSP/robot.c:55-61` |
| 9 | CLEAR_FAULT | 0..3 | 忽略 | 整板静止 | 清该路 `fault`、意外脉冲预算、停止确认故障位；`state = valid?IDLE:UNREADY` | `BSP/robot.c:423-427` |
| 10 | SELFTEST | 0..3 | 0=正向、1=负向 | arg 必须 0/1；`ch<4`；整板静止；无 `storage_error`；**四路均无 fault**；该路 `enabled && gear≠0` | 单路响应探测（会真实通电转动） | `BSP/robot.c:163-200` |
| 11 | SELFTEST_ALL | 忽略 | 0 或 1 | 同上，但对象为所有 `enabled && gear≠0` 的通道 | 逐路串行探测，中间不落盘 | `BSP/robot.c:172`、`BSP/robot.c:177`、`BSP/robot.c:293-296` |

`ch ≥ 4` 时，除 STOP_ALL/CONFIG/SAVE/SELFTEST_ALL 之外的通道级操作返回 `R_VALUE`（依据：`BSP/robot.c:408`）；`op > 11` 同样返回 `R_VALUE`（依据：`BSP/robot.c:440`）。

两个**编译期/运行期互锁**：

- `stop_input` 有效时，MOVE / SELFTEST / SELFTEST_ALL / DIRECTION / BRAKE(释放) 一律返回 `R_BUSY`（依据：`BSP/robot.c:367-368`）。该输入来自 PA15 按钮，**默认构建关闭**：`BUTTON_ESTOP_ENABLE = 0`（依据：`BSP/board_config.h:111`、`BSP/robot_port.c:214-217`、`BSP/robot_port.c:240-259`），此时 `stop_input` 恒为 0，该互锁不生效。
- `probe.stop_fault_mask` 非零（曾发生停止确认超时）时，MOVE / DIRECTION / BRAKE(释放) 返回 `R_UNREADY`，即使已恢复静止；只有 CLEAR_FAULT（对应通道）或 ZERO 才清除该位（依据：`BSP/robot.c:375-379`、`BSP/robot.c:425`、`BSP/robot.c:436`）。

### 3.3 自检（SELFTEST / SELFTEST_ALL）时序

1. 启动即把 `dirty=1` 与本次序号落盘，**在任何通道通电之前**；落盘失败则所有被选通道记为 `FAILED/ST_STORAGE_ERROR` 并返回 `R_STORAGE`，此时没有任何通道动过（依据：`BSP/robot.c:183-191`）。
2. 未选中的通道保持 `duty=0`、`brake=1`（包括此前被释放刹车的通道）（依据：`BSP/robot.c:196-197`）。
3. 逐路串行：一次只有一路处于 `S_SELFTEST`，其余排队（`ST_QUEUED`）或跳过（`ST_SKIPPED`，仅单路模式下未选中者）（依据：`BSP/robot.c:180-181`、`BSP/robot.c:198`、`BSP/robot.c:293-296`）。批次内通道之间**不做中间 Flash 写**（依据：`BSP/robot.c:295`）。
4. 去程：`duty` 从 0 起每 **10 ms** 增加 **10** 千分比，上限 **100**（即低电平 10%），不使用正常速度环（依据：`BSP/robot.c:251-256`、`BSP/robot.h:21-23`）。
5. 去程结束条件：累计边沿 ≥ **3**（`leg_target`）→ `PASSED` 进入回程；或通电达 **2000 ms** → `FAILED`，原因为 `ST_NO_PULSES`（0 脉冲）/`ST_TOO_FEW_PULSES`（有但不足）（依据：`BSP/robot.c:157`、`BSP/robot.c:247-250`、`BSP/robot.h:24-25`）。
6. 回程：`leg_target` 改为去程实测脉冲数，符号与 DIR 取反；结束条件同上，超时原因为 `ST_RETURN_TIMEOUT`（依据：`BSP/robot.c:278-288`、`BSP/robot.c:248-250`）。
7. 每程结束后进入 SETTLING：需 `stopped_ms` 之后 ≥ **500 ms**、该路最后边沿之后 ≥ **500 ms**、且**四路全部静止**才继续（依据：`BSP/robot.c:273-277`、`BSP/robot.h:12`）。若 `stopped_ms` 之后已满 **2000 ms** 仍不静止 → `FAILED/ST_STOP_TIMEOUT`，置 `fault=F_SELFTEST`、`valid=0`、`aborting=1`、置 `stop_fault_mask` 对应位，并把排队通道全部丢弃为 `ST_UNTESTED`+同一原因（依据：`BSP/robot.c:265-272`、`BSP/robot.h:26`）。
8. 残差 `residual = position − origin_position` 只记录、**不判失败**（依据：`BSP/robot.c:226-230`、`BSP/robot.c:289`）。
9. 批量模式（`scan_all`）下，`ST_NO_PULSES` / `ST_TOO_FEW_PULSES` 属“预期发现结果”，**不中止**后续通道；其他任何失败与取消都中止整批（依据：`BSP/robot.c:130-140`）。
10. 自检期间非被测通道出现边沿：累计到该路 `unexpected_fg`，超过预算即 `FAILED/ST_OTHER_ACTIVITY`，并对该路 `stop(F_UNEXPECTED)`、`valid=0`、把该路自检记录标为 `FAILED`（即使它未被选中）（依据：`BSP/robot.c:232-242`）。
11. 硬件层还有一份独立的安全兜底：`robot_selftest_safety_tick()` 在中断/主循环侧再判 2000 ms 通电上限、捕获错误计数变化、达到 `leg_target`、以及非被测通道超出“剩余意外预算”，命中即立刻 `duty=0`+`brake=1` 并锁存快照（依据：`BSP/robot_port.c:31-51`）。该兜底用的余量是 `10 − 已用 unexpected_fg`（下限 0），即**历史累积会压缩本次自检的容许量**（依据：`BSP/robot_port.c:107-109`、`BSP/robot.h:13-14`）。
12. 结束时 `pending_save=1` 并以 `dirty=0` 落盘；落盘失败且原因为 `NONE` 时把原因改为 `ST_STORAGE_ERROR`（响应状态保留）（依据：`BSP/robot.c:298-303`）。
13. 自检**会真实转动电机**，客户端 MUST NOT 把它当作纯查询（依据：`tools/robot_host.py:242`）。

### 3.4 示例帧（CRC 已用 `tools/robot_host.py:crc16` 本地核算；序号 1 仅为示例，实发必须替换并重算 CRC）

```text
读 UID                    01 03 00 08 00 06 44 0A
读命令协议版本            01 03 00 0F 00 01 B4 09
读 have_seq/最近序号      01 03 00 0E 00 02 A5 C8
读最近序号/业务结果       01 03 00 36 00 02 24 05
读保存标志                01 03 00 05 00 03 15 CA
读板级生效配置            01 03 00 00 00 04 44 09
读 M1 生效配置            01 03 01 00 00 05 84 35
读 M1 暂存配置            01 03 01 08 00 05 05 F7
读 M1 状态 7 项           01 03 01 10 00 07 04 31
读 M1 角度/计数/目标      01 03 01 18 00 06 44 33
读 M1 自检结果            01 03 01 20 00 08 44 3A
读 M1 往返细节            01 03 01 28 00 08 C5 F8
写暂存板级(0,1,1,25000)   01 10 00 20 00 04 08 00 00 00 01 00 01 61 A8 73 EB
写 M1 暂存(1,2,516,0,8000)01 10 01 08 00 05 0A 00 01 00 02 02 04 00 00 1F 40 1E 82
CONFIG  seq=1             01 10 00 30 00 05 0A 00 01 00 01 00 00 00 00 00 00 2C 98
ZERO    seq=1 ch=0        01 10 00 30 00 05 0A 00 01 00 02 00 00 00 00 00 00 1F 98
MOVE    seq=1 ch=0 +20.0° 01 10 00 30 00 05 0A 00 01 00 03 00 00 00 00 00 C8 0E CE
MOVE    seq=1 ch=0 −20.0° 01 10 00 30 00 05 0A 00 01 00 03 00 00 FF FF FF 38 4F 5E
STOP    seq=1 ch=0        01 10 00 30 00 05 0A 00 01 00 04 00 00 00 00 00 00 79 98
STOP_ALL seq=1            01 10 00 30 00 05 0A 00 01 00 05 00 00 00 00 00 00 69 58
BRAKE=1 seq=1 ch=0        01 10 00 30 00 05 0A 00 01 00 06 00 00 00 00 00 01 9B 98
BRAKE=0 seq=1 ch=0        01 10 00 30 00 05 0A 00 01 00 06 00 00 00 00 00 00 5A 58
DIRECTION=1 seq=1 ch=0    01 10 00 30 00 05 0A 00 01 00 07 00 00 00 00 00 01 8B 58
SAVE    seq=1             01 10 00 30 00 05 0A 00 01 00 08 00 00 00 00 00 00 B5 98
CLEAR_FAULT seq=1 ch=0    01 10 00 30 00 05 0A 00 01 00 09 00 00 00 00 00 00 A5 58
SELFTEST seq=1 ch=0 正向  01 10 00 30 00 05 0A 00 01 00 0A 00 00 00 00 00 00 96 58
SELFTEST seq=1 ch=0 负向  01 10 00 30 00 05 0A 00 01 00 0A 00 00 00 00 00 01 57 98
SELFTEST_ALL seq=1 正向   01 10 00 30 00 05 0A 00 01 00 0B 00 00 00 00 00 00 86 98
MOVE seq=145 ch=0 +20.0°  01 10 00 30 00 05 0A 00 91 00 03 00 00 00 00 00 C8 A2 C8
被拒示例：写 0x0001       01 06 00 01 00 02 59 CB   → 异常 02
被拒示例：写 0x0035=A55A  01 06 00 35 A5 5A 62 AF   → 异常 02
被拒示例：读 0x0010       01 03 00 10 00 01 85 CF   → 异常 02
```

正常应答示例：命令帧成功 → `01 10 00 30 00 05` + CRC；读成功 → `01 03 <2×count> <data…>` + CRC；失败 → `01 <func|0x80> <异常码>` + CRC（依据：`BSP/robot_protocol.c:24`、`BSP/robot_protocol.c:35`、`BSP/robot_protocol.c:37-38`）。

---

## 4. 序号、去重与重放

### 4.1 序号来源

- `seq = 0`（`ROBOT_AUTO_SEQUENCE`）表示**板端自动分配**：取 `last_seq+1`（若无记录则 1），回绕跳过 0，并**跳过当前去重缓存中已存在的序号**（最多 16 个），以免重放到旧的显式跳号（依据：`BSP/robot.h:9-10`、`BSP/robot.c:468-478`）。板端实际使用的序号事后可从 `0x0036` 读到（依据：`BSP/robot.c:494`、`BSP/robot.h:103-104`）。
- `seq = 0` **不是重放安全的**：它不携带身份，客户端无法区分“哪一次”被执行（依据：`BSP/robot.h:103`）。参考实现对任何命令都要求 `1 ≤ seq ≤ 65535`，否则直接报错（依据：`tools/robot_host.py:97-98`、`tools/robot_host.py:154-155`、`tools/robot_host.py:256-257`）。
- 客户端默认取号方式：读 `0x0036`，`seq = ((last+1) & 0xFFFF) or 1`（依据：`tools/robot_host.py:255`）。整板必须由**单一主站**统一分配（依据：任务书 §4.3）。

### 4.2 去重缓存

16 条环形缓存，每条记 `seq, op, ch, arg, result`（依据：`BSP/robot.h:91-92`、`BSP/robot.c:490-493`）。命中规则：

- `seq` 命中且 `op/ch/arg` **完全相同** → 直接返回**缓存的结果**，不再执行（依据：`BSP/robot.c:480-483`）。
- `seq` 命中但任一字段不同 → 返回 `R_SEQUENCE`（依据：`BSP/robot.c:481`）。
- 缓存是 **RAM-only**，重启即失效（依据：`BSP/robot.c:65`）。

**失败结果同样入缓存**：`execute()` 的返回值无条件写入缓存与 `0x0036/0x0037`（依据：`BSP/robot.c:489-494`）。因此一条被拒的 MOVE 也消耗序号；重发同一序号只会拿回同一个失败码（依据：`BSP/robot.c:482`）。

### 4.3 顺序窗口

- 若 `have_seq` 且 `(uint16)(seq − last_seq) ≥ 0x8000` → `R_SEQUENCE`（即只接受“向前”半窗口内的序号；等价于把回绕后半区视为旧号）（依据：`BSP/robot.c:484`）。
- 若 `have_seq` 且 `seq == last_seq` → `R_SEQUENCE`（依据：`BSP/robot.c:485`）。
- `have_seq == 0`（`0x000E` 读 0）时上述两条都跳过，任何非零序号都被接受（依据：`BSP/robot.c:484-485`、`BSP/robot.c:509`）。
- 这三条拒绝路径在 `last_seq` 更新**之前**返回，因此 `0x0036/0x0037` **不会**变化（依据：`BSP/robot.c:480-488`）。这与 `execute()` 内部的 `R_VALUE` 不同——后者会更新 `0x0036/0x0037`（依据：`BSP/robot.c:488-494`）。**这是客户端区分“序号被拒（未执行）”与“参数被拒（已消耗序号）”的唯一可靠方法。**

### 4.4 持久化与重启后的重放

- 每次落盘都会把 `request_sequence = last_seq`、`request_seen = have_seq` 写进镜像（依据：`BSP/robot.c:50-54`）；`robot_init()` 从上电镜像恢复（依据：`BSP/robot.c:68`）。因此序号在重启后仍受窗口约束（依据：`tests/robot_test.c:89`、`tests/robot_test.c:591`）。
- `dirty=0` 时的 MOVE 会**先落盘 `dirty=1` 与本次序号，再开始运动**，注释明确目的是“重启也不能重放已受理的运动”（依据：`BSP/robot.c:453-456`、`BSP/robot.c:486-487`）。
- **残留窗口**：若 MOVE 时 `dirty` 已为 1，则不会立即落盘（依据：`BSP/robot.c:453`）；此时序号只在下一次落盘（静止后 `pending_save` 冲刷、SAVE、CONFIG、ZERO）时才进 Flash（依据：`BSP/robot.c:360-362`、`BSP/robot.c:381-385`）。若在这个区间内重启，Flash 中仍是较旧的序号，同序号重发**可能被再次受理**。客户端 MUST 把“重启后状态”视为 UNKNOWN：先读 `0x000E`、`0x0036/0x0037`、四路 `state/fault/valid` 再决定（依据：`BSP/robot.c:68`、`BSP/robot.c:509`、`BSP/robot.c:515-516`）。

### 4.5 失联与对账

- 应答超时后 MUST 用**相同字节、相同序号**重试（最多有限次），MUST NOT 换新序号重发 MOVE（依据：`tools/robot_host.py:58-59`、`tools/robot_host.py:81`、`tools/robot_host.py:135`）。
- 重试仍无应答 → 标记 UNKNOWN，读 `0x0036/0x0037` 与通道状态对账；若 `0x0036` 已等于本次序号，说明板端已受理并按 `0x0037` 给出结果（依据：`BSP/robot.c:494`）。
- 重连 MUST NOT 续跑，MUST NOT 把上一次的 `DONE` 当作新请求成功（依据：任务书 §4.3）。
- STOP 优先于待发普通任务，但不能抢占半帧；单次交易超时必须有界（依据：`BSP/robot_port.c:160-164`、任务书 §4.3）。无回包时 MUST 报告“停止未确认”，MUST NOT 声称机械已停（依据：任务书 §4.3、§9）。

---

## 5. 业务结果码与 Modbus 异常码

### 5.1 业务结果码（`0x0037` / CAN `p[2]`）

枚举顺序即数值（依据：`BSP/robot.h:27`；客户端同表见 `tools/robot_host.py:236-243`、`tools/robot_host.py:133`）：

| 值 | 名称 | 典型触发 | 依据 |
|---:|---|---|---|
| 0 | `R_OK` | 受理 | `BSP/robot.h:27` |
| 1 | `R_VALUE` | 参数非法：`op>11`、`ch≥4`、MOVE 超出 ±3600、目标溢出 int32、BRAKE/DIRECTION/SELFTEST 的 arg 非 0/1、Modbus 长度/字节数不符 | `BSP/robot.c:408`、`BSP/robot.c:440-441`、`BSP/robot.c:452`、`BSP/robot.c:413`、`BSP/robot.c:419`、`BSP/robot.c:165-166`、`BSP/robot_protocol.c:19`、`BSP/robot_protocol.c:33` |
| 2 | `R_BUSY` | 通道忙、距最后边沿 <500 ms、整板不静止（CONFIG/SAVE/ZERO/CLEAR_FAULT）、自检进行中、释放刹车时该路忙/有故障、`stop_input` 互锁 | `BSP/robot.c:442`、`BSP/robot.c:447`、`BSP/robot.c:387`、`BSP/robot.c:382`、`BSP/robot.c:430`、`BSP/robot.c:424`、`BSP/robot.c:373`、`BSP/robot.c:415`、`BSP/robot.c:420`、`BSP/robot.c:367-368` |
| 3 | `R_UNREADY` | 有 fault、未 enabled/gear=0、停止确认故障位未清、ZERO 时 gear=0、自检无可选通道 | `BSP/robot.c:446`、`BSP/robot.c:378`、`BSP/robot.c:431`、`BSP/robot.c:171`、`BSP/robot.c:175` |
| 4 | `R_STORAGE` | 落盘失败（含 `save()` 因运动/占空比/FG 变化而拒绝） | `BSP/robot.c:58`、`BSP/robot.c:383`、`BSP/robot.c:399`、`BSP/robot.c:434`、`BSP/robot.c:449`、`BSP/robot.c:168`、`BSP/robot_port.c:141-143` |
| 5 | `R_SEQUENCE` | 序号重放冲突、超出顺序窗口、等于 `last_seq` | `BSP/robot.c:481`、`BSP/robot.c:484-485` |
| 6 | `R_ADDRESS` | 寄存器不存在、不可写、读数量非法 | `BSP/robot.c:554-555`、`BSP/robot.c:560`、`BSP/robot.c:578-579` |

### 5.2 Modbus 异常码映射（多对一）

`exception()` 的映射（依据：`BSP/robot_protocol.c:11-12`）：

| Modbus 异常码 | 覆盖的业务结果 | 含义 |
|---:|---|---|
| `01` 非法功能 | —（功能码不受支持） | 仅 03/06/10 之外的功能码（依据：`BSP/robot_protocol.c:37`） |
| `02` 非法数据地址 | `R_ADDRESS`(6) | 寄存器不存在或不可写（依据：`BSP/robot_protocol.c:12`） |
| `03` 非法数据值 | `R_VALUE`(1) **与** `R_SEQUENCE`(5) | 两种完全不同的原因共用一个异常码（依据：`BSP/robot_protocol.c:12`） |
| `04` 从站设备失败 | `R_UNREADY`(3) **与** `R_STORAGE`(4) | 未就绪与存储故障共用（依据：`BSP/robot_protocol.c:12`） |
| `06` 从站忙 | `R_BUSY`(2) | 忙（依据：`BSP/robot_protocol.c:12`） |

**MUST**：异常码只是“分类提示”，**精确原因必须回读 `0x0036/0x0037`**（依据：`BSP/robot_protocol.c:12`、`BSP/robot.c:494`、`tools/robot_host.py:75`）。尤其：

- 异常 `03` + `0x0036` **未变** ⇒ 序号被拒，命令**未执行**、未消耗序号（依据：`BSP/robot.c:480-485`）。
- 异常 `03` + `0x0036` **等于本次序号** ⇒ 参数被拒，序号**已消耗**且失败结果已入缓存（依据：`BSP/robot.c:488-494`）。
- 异常 `04` 可能是 `R_UNREADY`（例如存在未清故障）也可能是 `R_STORAGE`（Flash 写失败），处置方式完全不同：前者要查 `B+0x11` 与 `0x0006`（依据：`BSP/robot.c:525`、`BSP/robot.c:506`）。
- **Modbus 异常码 04 ≠ 电机故障码 4（`F_UNEXPECTED`）**，两者属不同层次，MUST NOT 混用（依据：任务书 §4.2、`BSP/robot.h:29`、`BSP/robot_protocol.c:12`）。

### 5.3 CAN 的结果表达

CAN 应答第 3 字节 `p[2]` **直接就是业务结果码 0..6**，没有 Modbus 异常码概念；应答总是产生（`robot_can()` 恒返回 1）（依据：`BSP/robot_protocol.c:54`、`BSP/robot_protocol.c:45`）。客户端据此抛错并列出 1..6 的含义（依据：`tools/robot_host.py:132-133`）。

---

## 6. 状态 / 故障 / 自检枚举

### 6.1 运动状态 `B+0x10`（0..7）

`enum { S_UNREADY, S_IDLE, S_RUNNING, S_DECEL, S_SETTLING, S_DONE, S_FAULT, S_SELFTEST }`（依据：`BSP/robot.h:28`；客户端同表见 `tools/robot_host.py:246-254`）。

| 值 | 名称 | 语义与进入条件 | 依据 |
|---:|---|---|---|
| 0 | UNREADY | 无可信零点（`valid=0`）且无故障；**不等于禁止相对 MOVE** | `BSP/robot.c:96`、`BSP/robot.c:76`、`BSP/robot.h:58` |
| 1 | IDLE | 空闲，`valid=1` 且无故障 | `BSP/robot.c:96`、`BSP/robot.c:76` |
| 2 | RUNNING | MOVE 已启动 | `BSP/robot.c:459` |
| 3 | DECEL | 剩余角距 < 300 ddeg，进入爬行减速 | `BSP/robot.c:335-337` |
| 4 | SETTLING | 已切 PWM=0/刹车，等待余转与静止确认 | `BSP/robot.c:95`、`BSP/robot.c:351` |
| 5 | DONE | 正常到位（`completing=1` 且静止确认完成） | `BSP/robot.c:356`、`BSP/robot.c:332` |
| 6 | FAULT | 存在非零故障 | `BSP/robot.c:96`、`BSP/robot.c:356` |
| 7 | SELFTEST | 自检试转中 | `BSP/robot.c:160`、`BSP/robot.c:286` |

“忙”的定义是 `RUNNING|DECEL|SETTLING|SELFTEST`（依据：`BSP/robot.c:29-30`）；整板忙还包含自检会话进行中（依据：`BSP/robot.c:38-43`）。注意 `DONE`(5) 与自检 `PASSED`(4) 是**不同寄存器的不同编码**，MUST NOT 混用（依据：`BSP/robot.c:524`、`BSP/robot.c:537`）。

### 6.2 故障码 `B+0x11`（0..7）

`enum { F_NONE, F_FEEDBACK, F_TIMEOUT, F_CAPTURE, F_UNEXPECTED, F_POSITION, F_STORAGE, F_SELFTEST }`（依据：`BSP/robot.h:29`；客户端同表见 `tools/robot_host.py:262-270`）。

| 值 | 名称 | 触发 | 是否清 `valid` | 依据 |
|---:|---|---|---|---|
| 0 | 无 | — | — | `BSP/robot.h:29` |
| 1 | FEEDBACK | RUNNING/DECEL 中 >1000 ms 无 FG 边沿 | 是 | `BSP/robot.c:330`、`BSP/robot.c:94` |
| 2 | TIMEOUT | 超过该次 MOVE 的监控超时 | 是 | `BSP/robot.c:331`、`BSP/robot.c:463` |
| 3 | CAPTURE | 捕获错误计数变化（编码器接口异常） | 是 | `BSP/robot.c:315`、`BSP/robot.c:236` |
| 4 | UNEXPECTED | 非指令运动期间意外边沿累计 > 10 | 是 | `BSP/robot.c:322-323`、`BSP/robot.c:31-37` |
| 5 | POSITION | 到位误差超容差，或计数溢出 int32 | 是 | `BSP/robot.c:354-355`、`BSP/robot.c:320` |
| 6 | STORAGE | 存储故障（枚举保留位；板级以 `0x0006` 为准） | — | `BSP/robot.h:29`、`BSP/robot.c:506` |
| 7 | SELFTEST | 自检失败 / 停止确认超时 | 是 | `BSP/robot.c:132`、`BSP/robot.c:269` |

故障 MUST 由 CLEAR_FAULT 显式清除，固件不会自动清（依据：`BSP/robot.c:423-427`、任务书 §9）。`stop()` 中带故障参数即清 `valid`（依据：`BSP/robot.c:94`）。

### 6.3 自检状态 `B+0x20`（0..7）

`enum { ST_UNTESTED, ST_QUEUED, ST_RUNNING, ST_SETTLING, ST_PASSED, ST_FAILED, ST_CANCELLED, ST_SKIPPED }`（依据：`BSP/robot.h:32-33`；客户端同表见 `tools/robot_host.py:18-19`、`tools/robot_host.py:273-281`）。

| 值 | 名称 | 语义 | 依据 |
|---:|---|---|---|
| 0 | UNTESTED | 未检测；或被选中但从未通电（批次中止时排队项被丢弃） | `BSP/robot.c:102-106` |
| 1 | QUEUED | 已排队等待通电 | `BSP/robot.c:180` |
| 2 | RUNNING | 正在试转（去程或回程） | `BSP/robot.c:155`、`BSP/robot.c:283` |
| 3 | SETTLING | 已断电，等待静止确认 | `BSP/robot.c:123` |
| 4 | PASSED | 通过（去程与回程都采到足够边沿且静止确认成功） | `BSP/robot.c:247`、`BSP/robot.c:290` |
| 5 | FAILED | 失败（原因见 `B+0x21`） | `BSP/robot.c:249`、`BSP/robot.c:268` |
| 6 | CANCELLED | 被 STOP / BRAKE(1) / STOP_ALL / 急停取消 | `BSP/robot.c:146`、`BSP/robot.c:371` |
| 7 | SKIPPED | 单路模式下未被选中的通道（兼容保留；当前批量不跳过禁用通道） | `BSP/robot.c:180-181` |

**PASSED(4) 不等于位置已保存**，MUST 另读 `0x0005/0x0006/0x0007`（依据：`BSP/robot.c:298-303`、`BSP/robot.c:505-507`）。

### 6.4 自检原因 `B+0x21`（0..10）

`enum { ST_REASON_NONE, ST_NO_PULSES, ST_TOO_FEW_PULSES, ST_CAPTURE_ERROR, ST_OTHER_ACTIVITY, ST_STOP_TIMEOUT, ST_CANCEL, ST_STORAGE_ERROR, ST_POSITION_OVERFLOW, ST_RETURN_TIMEOUT, ST_RETURN_ERROR }`（依据：`BSP/robot.h:34-36`；客户端同表见 `tools/robot_host.py:20-22`、`tools/robot_host.py:289-300`）。

| 值 | 名称 | 触发 | 依据 |
|---:|---|---|---|
| 0 | NONE | 无异常 | `BSP/robot.c:247` |
| 1 | NO_PULSES | 去程 2000 ms 内 0 边沿 | `BSP/robot.c:248-250` |
| 2 | TOO_FEW_PULSES | 去程 2000 ms 内有边沿但不足 3 | `BSP/robot.c:248-250` |
| 3 | CAPTURE_ERROR | 捕获错误计数变化 | `BSP/robot.c:231`、`BSP/robot.c:235` |
| 4 | OTHER_ACTIVITY | 非被测通道出现意外边沿 | `BSP/robot.c:235`、`BSP/robot.c:239` |
| 5 | STOP_TIMEOUT | 断电后 2000 ms 仍未静止 | `BSP/robot.c:265-268` |
| 6 | CANCEL | 被取消 | `BSP/robot.c:146`、`BSP/robot.c:371` |
| 7 | STORAGE_ERROR | 启动前落盘失败，或结束时落盘失败 | `BSP/robot.c:187`、`BSP/robot.c:302` |
| 8 | POSITION_OVERFLOW | 计数/残差溢出 int32 或 uint32 | `BSP/robot.c:218`、`BSP/robot.c:221`、`BSP/robot.c:224`、`BSP/robot.c:229` |
| 9 | RETURN_TIMEOUT | 回程 2000 ms 内未采够边沿 | `BSP/robot.c:248-250` |
| 10 | RETURN_ERROR | **源码中未找到产生路径**；`BSP/robot.h:36` 注释标注“10 保留给历史残差失败”，固件代码从未使用该枚举，只出现在客户端名称表里 | `BSP/robot.h:36`、`tools/robot_host.py:22`（另见阶段 B 产物 `ros2/robot_motor_driver/robot_motor_driver/protocol.py` 的 `SelftestReason.RETURN_ERROR`，该文件正在演进，本文不固定其行号） |

客户端 MUST 容忍 10 这个值出现在历史记录里，但 MUST NOT 依赖它会再次产生（依据：`BSP/robot.h:36`）。

### 6.5 往返阶段 `B+0x28`（0..5）

`enum { SP_NONE, SP_OUTBOUND, SP_OUTBOUND_SETTLING, SP_RETURN, SP_RETURN_SETTLING, SP_FINISHED }`（依据：`BSP/robot.h:37`；客户端同表见 `tools/robot_host.py:23`、`tools/robot_host.py:303-309`）。去程/回程的通电时间分别落在 `B+0x24`（powered_ms）与 `B+0x29`（return_ms），脉冲分别落在 `B+0x22/0x23`（累计）、`B+0x2A/0x2B`（去程）、`B+0x2C/0x2D`（回程）（依据：`BSP/robot.c:117-121`、`BSP/robot.c:539-552`）。**阶段 5（FINISHED）不等于通过**，MUST 同时核对 `B+0x20 == 4`（依据：`BSP/robot.c:290-291`）。

---

## 7. 角度与脉冲换算

### 7.1 基本量

- FG 为单线反馈，**电机每转 6 个上升沿**（依据：`BSP/capture.h:14`）。单线无法测真实方向，运动方向只来自命令与 `inverted`（依据：`BSP/robot.c:457-458`、任务书 §5）。
- 位置计数 `position` 是**有符号边沿累计**，不模一圈（依据：`BSP/robot.h:57`）。
- 输出轴每脉冲角度 = `360/(gear×6)` 度 = `3600/(gear×6)` ddeg（依据：`BSP/robot.c:85`）。

### 7.2 脉冲 → 角度（读回）

```text
angle_ddeg = trunc_toward_zero( pulses × 3600 / (gear × 6) )
```

`gear == 0` 时恒返回 0；结果按 int64 计算后夹到 int32 范围（依据：`BSP/robot.c:81-89`）。C 的整数除法向零截断，因此正负对称、**不是四舍五入**（依据：`BSP/robot.c:85`）。角度寄存器就是把该函数作用在 `position`（`B+0x18`）与 `target`（`B+0x1C`）上得到的（依据：`BSP/robot.c:531-533`）。

每脉冲 ddeg：gear 516 → 1.1628；gear 290 → 2.0690；gear 82 → 7.3171（依据：`BSP/robot.c:85` 代入）。

### 7.3 角度 → 脉冲（MOVE）

```text
steps = trunc_toward_zero( (arg × gear × 6 + (arg > 0 ? +1800 : −1800)) / 3600 )
target = position + steps
```

`±1800` 是半个除数的偏置，等价于**远离零方向的四舍五入**；随后仍是向零截断的整数除法（依据：`BSP/robot.c:450-451`）。`arg` 范围 `−3600..3600` ddeg（±360.0°），越界返回 `R_VALUE`（依据：`BSP/robot.c:441`）；`arg == 0` 直接返回 `R_OK` 且**不产生运动**（依据：`BSP/robot.c:448`）。客户端换算：`arg = round(rad × 180/π × 10)`，且**不得再乘减速比**（依据：任务书 §5、`tools/robot_host.py:279`）。

### 7.4 实例（本地按上述公式计算，非实板实测）

| 场景 | 计算 | 结果 |
|---|---|---|
| gear=516，MOVE +20.0°（arg=200） | `(200×3096+1800)/3600 = 621000/3600` | steps=172；读回角度 `172×3600/3096 = 200` → **+20.0°（无量化误差）** |
| gear=516，MOVE +360.0°（arg=3600） | `(3600×3096+1800)/3600` | steps=3096＝输出轴一整圈（`516×6`） |
| gear=82，MOVE +10.0°（arg=100） | `(100×492+1800)/3600 = 51000/3600` | steps=14；读回 `14×3600/492 = 102` → **+10.2°（量化 +0.2°）** |
| gear=82，MOVE −10.0°（arg=−100） | `(−49200−1800)/3600` | steps=−14；读回 **−10.2°**（向零截断，正负对称） |
| gear=82，MOVE +0.1°（arg=1） | `(492+1800)/3600 = 0` | steps=0 → `target == position`，下一轮立即判 `remaining ≤ 0` 并走 SETTLING→**DONE，但没有转动** |
| 实板只读回包 M2（gear=516，position=928） | `928×3600/3096 = 1079`（余 216） | `B+0x18` 读回 `0x00000437 = 1079` → **+107.9°，与公式一致** |
| 实板只读回包 M4（gear=82，position=−142） | `−142×3600/492 = −1039`（向零截断） | `B+0x18` 读回 `0xFFFFFBF1 = −1039` → **−103.9°，与公式一致** |

量化后果（MUST 知悉）：gear 越小、单脉冲角度越大，`arg` 很小时可能量化为 0 步而报 DONE；客户端 MUST NOT 把 `DONE` 当作“发生了请求的位移”（依据：`BSP/robot.c:450`、`BSP/robot.c:332`、`BSP/robot.c:356`）。

### 7.5 到位容差

静止确认时若 `completing=1` 且 `|position − target| × 3600 > 50 × 6 × gear` → 置 `F_POSITION` 并清 `valid`（依据：`BSP/robot.c:351-355`、`BSP/robot.h:11`）。换算成脉冲门限（首个判超差的 `|误差|`）：

| gear | 容差 ddeg | 首个超差脉冲数 |
|---:|---:|---:|
| 82 | 50（=5.0°） | 7 |
| 290 | 50 | 25 |
| 516 | 50 | 44 |

该检查**只在正常到位（`completing=1`）时执行**；STOP/取消路径 `completing=0`，不判容差（依据：`BSP/robot.c:332`、`BSP/robot.c:90-93`、`BSP/robot.c:354`）。

### 7.6 运动监控超时

```text
timeout_ms = 10000 + 4 × ( distance_ddeg × gear × 60000 / (3600 × rpm) )
```

其中 `distance_ddeg = |arg|`，除法为无符号整数除法（依据：`BSP/robot.c:461-463`）。注释明确这是“宽松的有限超时，不是到位时间承诺”（依据：`BSP/robot.c:461`、任务书 §5）。实例（本地计算）：

| 场景 | 计算 | timeout_ms |
|---|---|---:|
| gear=516，rpm=8000，+20.0° | `200×516×60000/(3600×8000) = 215`；`10000+215×4` | **10860** |
| gear=516，rpm=400，+20.0° | `…/1440000 = 4300`；`10000+4300×4` | **27200** |
| gear=516，rpm=8000，+360.0° | `3600×516×60000/28800000 = 3870`；`10000+3870×4` | **25480** |
| gear=82，rpm=1500，+10.0° | `100×82×60000/(3600×1500) = 91`；`10000+91×4` | **10364** |

超过 `timeout_ms` 置 `F_TIMEOUT` 并清 `valid`（依据：`BSP/robot.c:331`、`BSP/robot.c:94`）。另有独立的反馈超时：RUNNING/DECEL 中 >1000 ms 无边沿 → `F_FEEDBACK`（依据：`BSP/robot.c:330`、`BSP/robot.h:15`）。客户端 MUST 以固件超时为上限设置自己的等待窗口，MUST NOT 用固定“到位时间”承诺（依据：`BSP/robot.c:461`、任务书 §5）。

### 7.7 速度环（客户端只需知道边界）

控制周期 10 ms；剩余角距 < 300 ddeg 进入 DECEL，期望转速按 `400 + ddegrees×(rpm−400)/300` 线性降到爬行转速 400（依据：`BSP/robot.h:16`、`BSP/robot.h:18-19`、`BSP/robot.c:333-338`）。PI：`integral += error/40` 夹在 `0..700`；`demand = desired/16 + error/32 + integral`，夹在 `0..950`，且每控制周期最多增加 20（依据：`BSP/robot.c:339-348`、`BSP/robot.h:20`）。实测 `rpm` 由 FG 周期算出：`rpm = 10000000 / period_us`，>250 ms 无边沿则报 0（依据：`BSP/capture.c:108`、`BSP/capture.c:262`）。

---

## 8. 保存语义（dirty / storage_error / pending_save）

### 8.1 三个标志

| 标志 | 含义 | 置 1 时机 | 清 0 时机 | 依据 |
|---|---|---|---|---|
| `0x0005` dirty | Flash 中的位置记录**不可信**（尚未确立/尚未保存零点语义） | MOVE 首次运动前（`dirty=0` 时）；自检启动前 | CONFIG 成功；ZERO 成功；SAVE 成功；自检结束落盘成功；任何 `persist(...,0)` 成功 | `BSP/robot.c:453-456`、`BSP/robot.c:185`、`BSP/robot.c:400`、`BSP/robot.c:435`、`BSP/robot.c:384`、`BSP/robot.c:299`、`BSP/robot.c:59` |
| `0x0006` storage_error | 最近一次落盘失败 | 任何 `ops.save()` 返回 0（含 CONFIG/ZERO 直接调用） | 任何 `persist()` 成功；CONFIG 成功；ZERO 成功 | `BSP/robot.c:58`、`BSP/robot.c:399`、`BSP/robot.c:434`、`BSP/robot.c:59`、`BSP/robot.c:400`、`BSP/robot.c:435` |
| `0x0007` pending_save | 有位置/状态变化等待落盘 | 捕获错误停机；意外边沿停机；SETTLING 收尾；自检会话结束 | 静止后自动冲刷成功；SAVE；CONFIG；ZERO；自检启动 | `BSP/robot.c:315`、`BSP/robot.c:323`、`BSP/robot.c:357`、`BSP/robot.c:298`、`BSP/robot.c:361`、`BSP/robot.c:384`、`BSP/robot.c:400`、`BSP/robot.c:435`、`BSP/robot.c:192` |

自动冲刷条件：`pending_save && quiet() && !storage_error`（依据：`BSP/robot.c:360-362`）。因此 `pending_save` 可能在运动/余转期间长时间保持 1，客户端 MUST NOT 把它当作错误，也 MUST NOT 在它为 1 时报告“已保存”（依据：`BSP/robot.c:357`、`BSP/robot.c:360`、任务书 §6）。

### 8.2 落盘的前置条件（`save()`）

`ops.save()` 在以下任一情况**直接返回 0**（即不落盘）：整板忙（含自检会话）；任一通道 `duty != 0`；任一通道 FG 计数或错误计数与快照不一致（依据：`BSP/robot_port.c:141-143`）。写完后若发现 FG 计数/错误又变化，则视为“静止窗口被外部扰动”，尝试补写一份 `dirty=1` 的保守记录并返回 0（依据：`BSP/robot_port.c:145-153`）。这解释了为什么 CONFIG/ZERO/SAVE/自检启动都要求 `quiet()`（依据：`BSP/robot.c:382`、`BSP/robot.c:387`、`BSP/robot.c:424`、`BSP/robot.c:430`、`BSP/robot.c:167`）。

落盘发生在**命令处理路径内**（`frame()` → `robot_modbus()` → `robot_write()` → `robot_command()` → `execute()` → `persist()` → `ops.save()`），因此应答会被 Flash 编程/擦除阻塞而延迟，客户端超时必须容纳（依据：`BSP/robot_port.c:156-165`、`BSP/robot.c:55-61`、`tools/robot_host.py:64`）。

### 8.3 Flash 布局与写入策略

| 项 | 值 | 依据 |
|---|---|---|
| 存储基址 | `0x0807F000` | `BSP/robot_store.h:4` |
| 页大小 / 页数 | 2048 B × 2 页 | `BSP/robot_store.h:5`、`BSP/robot_store.h:7` |
| 槽大小 / 槽数 | 256 B，每页 8 槽（共 16 槽） | `BSP/robot_store.h:6`、`BSP/robot_store.c:5`、`BSP/robot_store.c:35` |
| 槽头 | `MAGIC 0x524F4231`、`schema=1`、`sizeof(robot_image_t)`、`sequence`、`dirty(≤1)` | `BSP/robot_store.c:3`、`BSP/robot_store.c:17-18`、`BSP/robot_store.c:47-48` |
| 校验 | CRC32（反射多项式 `0xEDB88320`，初值 `0xFFFFFFFF`，结果取反）+ 末尾 `COMMIT 0xC04D17ED` 字 | `BSP/robot_store.c:4`、`BSP/robot_store.c:9-12`、`BSP/robot_store.c:49` |
| 加载 | 扫描全部槽，取 `sequence` 最新的合法槽，并输出 `dirty` | `BSP/robot_store.c:20-30` |
| 写入 | 优先追加当前页的空槽（全 `0xFF`）；无空槽则翻到另一页并**先擦除**，绝不擦除含最新合法记录的那一页；写后回读校验，最后写 COMMIT 字再校验一次 | `BSP/robot_store.c:33-58` |

**页翻转（擦除）会带来最长的一次性延迟**，这正是客户端 1.5 s 交易超时的原因（依据：`BSP/robot_store.c:41-46`、`tools/robot_host.py:64`）。

### 8.4 上电恢复规则

`robot_init()`：镜像非法则回退默认值（address=1、node=1、pwm_hz=25000、四路 rpm=8000、gear=0、enabled=0）（依据：`BSP/robot.c:66`、`BSP/robot.c:5-10`）。每通道 `valid = (!dirty) && (image.valid[i]==1) && (gear!=0)`，`state = valid ? IDLE : UNREADY`，`brake=1`，`target=position`（依据：`BSP/robot.c:74-76`）。

**关键推论**：若在 `dirty=1` 状态下重启（例如 MOVE 已启动但尚未完成落盘冲刷），四路 `valid` 全部变 0、状态变 UNREADY（依据：`BSP/robot.c:75-76`）。客户端 MUST 在每次上电/重连后重新读取 `valid`，MUST NOT 沿用上次会话的位置有效性（依据：任务书 §2“节点启动必须重新读取”）。

---

## 9. 时间与限制常量表

| 名称 | 值 | 含义 | 依据 |
|---|---:|---|---|
| `ROBOT_CHANNELS` | 4 | 固定四通道 | `BSP/robot.h:5` |
| `ROBOT_SCHEMA` | 1 | Flash 镜像版本（`0x0000`） | `BSP/robot.h:6` |
| `ROBOT_COMMAND_PROTOCOL` | 2 | 单帧命令协议版本（`0x000F`） | `BSP/robot.h:8` |
| `ROBOT_AUTO_SEQUENCE` | 0 | `seq=0` 表示板端自动分配 | `BSP/robot.h:10` |
| `ROBOT_TOLERANCE_DDEG` | 50 | 到位容差（5.0° 输出轴） | `BSP/robot.h:11` |
| `ROBOT_QUIET_MS` | 500 | 静止判定窗口 | `BSP/robot.h:12` |
| `ROBOT_UNEXPECTED_FG_LIMIT` | 10 | 意外边沿累计预算，**第 11 个**触发 | `BSP/robot.h:14`、`BSP/robot.c:36` |
| `ROBOT_FEEDBACK_MS` | 1000 | 运动中无反馈超时 | `BSP/robot.h:15` |
| `ROBOT_CONTROL_MS` | 10 | 控制/斜坡周期 | `BSP/robot.h:16` |
| `ROBOT_CRAWL_RPM` | 400 | 爬行转速下限 | `BSP/robot.h:18` |
| `ROBOT_DECEL_DDEG` | 300 | 减速起始剩余角距（30.0°） | `BSP/robot.h:19` |
| `ROBOT_DUTY_MAX` | 950 | 正常运动占空比上限（低电平千分比 95%） | `BSP/robot.h:20` |
| `ROBOT_SELFTEST_DUTY_MAX` | 100 | 自检占空比上限（10%），不走速度环 | `BSP/robot.h:21-22` |
| `ROBOT_SELFTEST_DUTY_STEP` | 10 | 自检每周期占空比增量 | `BSP/robot.h:23` |
| `ROBOT_SELFTEST_POWER_MS` | 2000 | 自检每程通电上限 | `BSP/robot.h:24` |
| `ROBOT_SELFTEST_EDGES` | 3 | 自检去程目标边沿数 | `BSP/robot.h:25` |
| `ROBOT_SELFTEST_STOP_MS` | 2000 | 自检停止确认上限 | `BSP/robot.h:26` |
| MOVE 参数范围 | −3600..3600 ddeg | ±360.0° 相对 | `BSP/robot.c:441` |
| MOVE 监控超时 | `10000 + 4×(dist×gear×60000/(3600×rpm))` ms | 见第 7.6 节 | `BSP/robot.c:463` |
| 去重缓存 | 16 条环形 | RAM-only | `BSP/robot.h:91-92`、`BSP/robot.c:490` |
| 序号窗口 | `(seq−last_seq) mod 65536 ≥ 0x8000` → 拒绝 | 半范围顺序检查 | `BSP/robot.c:484` |
| `address` | 1..247 | Modbus 站号 | `BSP/robot.c:13` |
| `node` | 1..127 | CAN 节点号 | `BSP/robot.c:13-14` |
| `protocol` | 0..1 | 0=Modbus，1=CAN | `BSP/robot.c:13` |
| `pwm_hz` | 20000..30000（默认 25000） | PWM 频率；驱动层同样夹取该范围 | `BSP/robot.c:14`、`BSP/robot.c:8`、`BSP/pwm.c:23` |
| `enabled` / `inverted` | 0..1 | 布尔 | `BSP/robot.c:17` |
| `role` | 0..7，非零值四路唯一 | 关节角色 | `BSP/robot.c:17`、`BSP/robot.c:20` |
| `gear` | ∈ {0, 82, 290, 516} | 0 仅限未配置通道 | `BSP/robot.c:18` |
| `rpm` | 400..8000（默认 8000） | 减速前巡航转速 | `BSP/robot.c:17`、`BSP/robot.c:9` |
| enabled 的附加约束 | `enabled ⇒ gear≠0 且 role≠0` | 否则整板配置非法 | `BSP/robot.c:19` |
| RTU 帧间隔门限 | 750 µs（脏帧）/ 1750 µs（分帧） | 见第 1.2 节 | `BSP/robot_rtu.c:10-11` |
| RTU 帧长上限 | 256 B（`robot_modbus` 同样拒绝 >256） | 超长整帧丢弃 | `BSP/robot_rtu.h:5`、`BSP/robot_protocol.c:16` |
| 接收环形缓冲 | 512 B；发送缓冲 256 B | 溢出即丢并计数 | `BSP/usart0.c:16`、`BSP/usart0.c:24`、`BSP/usart0.c:84-87` |
| Modbus 读/写数量 | 读 1..125；写 1..123 | 超限异常 03/02 | `BSP/robot_protocol.c:22`、`BSP/robot_protocol.c:33`、`BSP/robot.c:560`、`BSP/robot.c:578` |
| CAN 收发 | 500 kbit/s、DLC 8、标准数据帧、请求 `0x600+node`、应答 `0x580+node`；每轮最多收 3 帧；发送邮箱超时 50 ms | 见第 12 节 | `BSP/can_bus.h:8`、`BSP/can_bus.c:75-81`、`BSP/robot_port.c:188`、`BSP/robot_port.c:195`、`BSP/robot_port.c:201`、`BSP/robot_port.c:204-205` |
| FG 边沿/转 | 6 | 单线 | `BSP/capture.h:14` |
| rpm 新鲜度 | >250 ms 无边沿报 0 | 陈旧即 0 | `BSP/capture.c:262`、`BSP/capture.h:46` |
| duty 语义 | 低电平千分比 0..1000（输出极性 LOW、空闲 HIGH） | 见第 10 节 | `BSP/robot.h:75-76`、`BSP/pwm.c:12-16`、`BSP/pwm.c:40-41` |
| 客户端参考超时 | 交易 1.5 s、帧间 4 ms、重试 3 次、串口读 0.05 s | 下限参考 | `tools/robot_host.py:53`、`tools/robot_host.py:59-64` |

`unexpected_fg` 是 **RAM-only、饱和累加**，且**排除指令性运动**；只有 `robot_init()`（上电）、CLEAR_FAULT、ZERO 会清零（依据：`BSP/robot.h:13`、`BSP/robot.h:70`、`BSP/robot.c:31-37`、`BSP/robot.c:65`、`BSP/robot.c:426`、`BSP/robot.c:437`）。正常 MOVE 期间 `sign≠0` 的边沿计入位置而非意外预算（依据：`BSP/robot.c:318-324`）。**因此预算跨多次运动累积**，长期不清故障的板子可能在第 11 个杂散边沿处报 `F_UNEXPECTED`（依据：`BSP/robot.c:36`、`BSP/robot.c:322-323`）。

---

## 10. 刹车与方向

### 10.1 刹车

- 寄存器 `B+0x13`：1 = 刹车施加（依据：`BSP/robot.c:527`）。
- 硬件极性：**刹车 ON = 低电平，OFF/释放 = 高电平**（依据：`BSP/motor_io.c:58-63`）。
- 引脚：M1 BRAKE = PE13、M2 = PA7、M3 = PD2、M4 = PB5（依据：`BSP/motor_pins.h:61-62`、`BSP/motor_pins.h:71-72`、`BSP/motor_pins.h:81-82`、`BSP/motor_pins.h:91-92`）。
- 上电与任何停机路径都先施加刹车：`robot_init()` 置 `brake=1`（依据：`BSP/robot.c:76`）；`stop()` 置 `duty=0, brake=1`（依据：`BSP/robot.c:93`）；自检启动时四路一律 `duty=0, brake=1`（依据：`BSP/robot.c:197`）。
- `BRAKE(arg=1)` 等价于 STOP；`BRAKE(arg=0)` 仅在该路不忙且无故障时释放（依据：`BSP/robot.c:412-417`）。释放刹车还受 `stop_input` 与 `stop_fault_mask` 互锁（依据：`BSP/robot.c:367-368`、`BSP/robot.c:377-379`）。
- 输出回调的约定：**改 DIR/刹车之前必须先把 PWM 置为停止**（依据：`BSP/robot.h:75-76`、`BSP/robot_port.c:114-117`）。

### 10.2 方向

- 寄存器 `B+0x14` 是 DIR 电平（0/1），`DIRECTION` 命令可直接写它（依据：`BSP/robot.c:528`、`BSP/robot.c:418-421`）。
- **MOVE 会覆盖 DIR**：`direction = (arg < 0) ^ inverted`（依据：`BSP/robot.c:458`）。因此 `DIRECTION` 的写入只对“下一次非 MOVE 的输出状态”有意义，MUST NOT 被当作运动方向的持久设定（依据：`BSP/robot.c:458`、`BSP/robot.c:421`）。
- 自检同样自行计算 DIR：去程 `negative ^ inverted`，回程 `(negative^1) ^ inverted`（依据：`BSP/robot.c:159`、`BSP/robot.c:285`）。
- `inverted` 是每通道配置项（暂存 `B+0x0B`、生效 `B+0x03`），改变它会使该通道 `valid` 失效并清空其自检记录（依据：`BSP/robot.c:394-396`、`BSP/robot.c:402-404`）。
- DIR 引脚：M1 = **PE15**、M2 = PC5、M3 = PD3、M4 = PB8（依据：`BSP/motor_pins.h:59-60`、`BSP/motor_pins.h:69-70`、`BSP/motor_pins.h:79-80`、`BSP/motor_pins.h:89-90`、`BSP/motor_io.c:42-51`、`BSP/motor_io.c:82-85`）。
- `motor_set_dir(motor, reverse)`：`reverse=1` → 置位，`reverse=0` → 复位（依据：`BSP/motor_io.c:38-52`）。**DIR 电平与机械正方向的对应关系未经本轮实测**，`inverted` 的正确取值必须现场核对（依据：任务书 §2“所有关节完整机械坐标和限位仍需确认”）。
- duty 是**低电平千分比**：`pwm_set_permille()` 把 0..1000 映射为比较值（`(period×duty+500)/1000`，上限 1000），输出极性为 LOW、空闲态 HIGH（依据：`BSP/robot.h:75-76`、`BSP/pwm.c:12-16`、`BSP/pwm.c:40-41`）。因此 `B+0x15` 读到的 950 表示 95% 低电平时间，MUST NOT 当作“95% 转速”。

### 10.3 复位与急停

`robot_emergency_stop()` 在复位早期把四路 PWM 引脚拉高（＝0% 低电平＝停止）并把四路 BRAKE 引脚拉低（＝刹车 ON），且**不动 DIR**；注释说明它在 CPU 脱离硬件复位前无法控制引脚，且必须保持纯整数、不依赖时钟/定时器/全局初始化（依据：`BSP/robot_port.c:61-80`、`BSP/pwm.c:40-41`、`BSP/motor_io.c:60-63`）。PA15 按钮急停默认未编译（`BUTTON_ESTOP_ENABLE = 0`），启用后会在每轮更新中调用 `robot_stop_all()` 并置 `stop_input`（依据：`BSP/board_config.h:111`、`BSP/robot_port.c:214-217`、`BSP/robot_port.c:240-259`）。固件**没有通信心跳停机**：失联后已受理的 MOVE 会继续到目标或故障（依据：任务书 §5）。

---

## 11. UID 与协议版本

### 11.1 UID

- 来源是 GD32 的 96 位唯一 ID：`0x1FFFF7E8`、`0x1FFFF7EC`、`0x1FFFF7F0` 三个字，在 `robot_port_init()` 读入（依据：`BSP/robot_port.c:221-222`）。
- 通过 `0x0008`～`0x000D` 六个寄存器读出，**每个 32 位字高字在前**（`a` 为偶数取高 16 位、奇数取低 16 位）（依据：`BSP/robot.c:508`）。
- 客户端拼法：把 6 个寄存器按顺序格式化为 4 位大写十六进制并连接，得到 24 字符 UID（依据：`tools/robot_host.py:162-163`）。本轮实板只读回包 `C2 3B 37 81 15 B0 92 08 38 30 36 4A` → `C23B378115B092083830364A`，与任务书 §2 记录的板一致（依据：`BSP/robot.c:508`、任务书 §2）。
- 清单校验要求 UID 必须是 24 位大写十六进制，且**每一次写入/运动前都要核对**（`--uid` 对除 `status` 外的所有动作强制要求）（依据：`tools/robot_host.py:177-178`、`tools/robot_host.py:248-249`、`tools/robot_host.py:252-254`）。
- **UID 与协议版本都不是固件构建哈希**，MUST NOT 用它们证明板上二进制身份（依据：任务书 §4.1、第 14.1 节）。

### 11.2 三个容易混淆的“版本”

| 寄存器 | 名称 | 值 | 含义 | 依据 |
|---|---|---|---|---|
| `0x0000` | Flash schema | 常量 1 | 持久化镜像结构版本；与命令协议**独立** | `BSP/robot.c:500`、`BSP/robot.h:6-7`、`BSP/robot_store.c:17` |
| `0x0001` | protocol | 0 或 1 | 当前**传输**选择（0=Modbus、1=CAN）；重启后才随 CONFIG 改变 | `BSP/robot.c:501`、`BSP/robot.c:69`、`BSP/robot_port.c:253` |
| `0x000F` | 命令协议版本 | 常量 2 | 单帧命令协议（写 `0x0030` 即执行）；旧固件通常对 `0x000F` 返回非法地址 | `BSP/robot.c:510`、`BSP/robot.h:7-8`、`tools/robot_host.py:99-100` |

客户端 MUST 在下发任何命令前确认 `0x000F == 2`；参考实现在每次 `command()` 中都检查，并提示“固件与客户端需一起升级”（依据：`tools/robot_host.py:99-100`）。

---

## 12. CAN 差异

CAN 是**自定义协议，不是 CANopen**（依据：任务书 §4.1、`BSP/robot_protocol.h:7-11`）。

### 12.1 链路层

| 项 | 规定 | 依据 |
|---|---|---|
| 波特率 / 引脚 | 500 kbit/s，CAN0 PD0=RX / PD1=TX，全重映射，正常模式 | `BSP/can_bus.h:8`、`BSP/can_bus.c:75-81` |
| 位时序 | 60 MHz /(预分频 8 ×(1 + BS1 12 + BS2 2))，SJW=1TQ | `BSP/can_bus.c:75-81` |
| ID | 请求 `0x600 + node`，应答 `0x580 + node` | `BSP/robot_port.c:195`、`BSP/robot_port.c:205` |
| 帧类型 | 只接受 **11 位标准数据帧**且 **DLC 必须为 8**；扩展帧/远程帧/错误帧/其他 ID/其他 DLC 一律丢弃且不回复 | `BSP/robot_port.c:204-205`、`BSP/robot_protocol.h:7` |
| 校验 | 无应用层 CRC（依赖 CAN 链路 CRC） | `BSP/robot_protocol.c:42-54` |
| 串行化 | 上一应答未发出（`can_pending` 或邮箱占用）时**不消费新请求**；邮箱超过 50 ms 未成功即停止并释放 | `BSP/robot_port.c:185-199` |
| 吞吐 | 每轮主循环最多收 3 帧（硬件 FIFO 深度），保证总线负载不会饿死运动控制 | `BSP/robot_port.c:200-201` |

### 12.2 报文格式

| 类型 | 请求 8 字节 | 应答 8 字节 | 依据 |
|---|---|---|---|
| 读（op=1） | `01, tag, addrHi, addrLo, count(1..2), 00, 00, 00` | `81, tag, result, count, d0Hi, d0Lo, d1Hi, d1Lo` | `BSP/robot_protocol.c:46-48`、`BSP/robot_protocol.h:9`、`tools/robot_host.py:142` |
| 写（op=2，单寄存器） | `02, tag, addrHi, addrLo, 00, valHi, valLo, 00` | `82, tag, result, 0,0,0,0,0` | `BSP/robot_protocol.c:49-50`、`tools/robot_host.py:151` |
| 命令（op=3） | `03, seqHi, seqLo, op, ch, argHi, argLo, 00`，arg 为 **int16** | `83, seqHi, result, seqLo, 0,0,0,0` | `BSP/robot_protocol.c:51-52`、`BSP/robot_protocol.h:10-11`、`tools/robot_host.py:156` |

要点：

- **每次读最多 2 个寄存器**，且 `q[5..7]` 必须全 0，否则整帧判 `R_VALUE`（依据：`BSP/robot_protocol.c:46`）。读 6 个 UID 寄存器需要 3 帧，客户端按 2 个一组分块并递增 `tag`（依据：`tools/robot_host.py:137-146`）。`count=1` 时应答的 `d1` 恒为 0（依据：`BSP/robot_protocol.c:44`、`BSP/robot_protocol.c:48`）。
- **每帧只能写 1 个寄存器**，且 `q[4]`、`q[7]` 必须为 0（依据：`BSP/robot_protocol.c:49-50`）。因此一次 CONFIG 需要 4（板级）+ 4×5（通道）= **24 次单寄存器写**，再发 1 条 op=3 的 CONFIG 命令（依据：`BSP/robot.c:580-591`、`BSP/robot.c:386-407`、`tools/robot_host.py:214-219`）。
- CAN **无法**用写报文触发命令：对 `0x0030` 的单寄存器写会因 `writable()` 为假返回 `R_ADDRESS`（结果字节 6）（依据：`BSP/robot_protocol.c:50`、`BSP/robot.c:564-568`、`BSP/robot.c:579`）。命令只能走 op=3 帧（依据：`BSP/robot_protocol.c:51-52`）。
- 命令帧的 `arg` 是 **int16**（`(int16_t)get16(q+5)`），而 Modbus 路径是 int32（依据：`BSP/robot_protocol.c:52`、`BSP/robot.c:575`）。对 MOVE 而言 ±3600 在 int16 范围内，不受影响（依据：`BSP/robot.c:441`）；但客户端 MUST NOT 假定两种传输的参数宽度相同。
- `tag`（读/写）与 `seqHi`（命令）都原样回显在应答第 2 字节；命令帧还额外回显 `seqLo` 于第 4 字节，客户端 MUST 校验（依据：`BSP/robot_protocol.c:45`、`BSP/robot_protocol.c:47`、`BSP/robot_protocol.c:52`、`tools/robot_host.py:128`、`tools/robot_host.py:130-131`）。
- **应答第 3 字节直接是业务结果码 0..6**，不存在 Modbus 异常码；应答总会产生（依据：`BSP/robot_protocol.c:54`、`tools/robot_host.py:132-133`）。
- 客户端过滤条件：ID 必须等于 `0x580+node`、非扩展帧、非远程帧、非错误帧、DLC=8、`p[0] == (请求 op | 0x80)`、`p[1] == 请求 tag/seqHi`；命令还需 `p[3] == 请求 seqLo`（依据：`tools/robot_host.py:126-131`）。
- CAN 无应答时同样 MUST 用**相同序号**重试，MUST NOT 自动新建运动请求（依据：`tools/robot_host.py:135`）。

---

## 13. 客户端实现义务清单

以下把源码中的约束与已知陷阱转成可验收条目。编号供测试用例引用。

**身份与版本**

- **C-01 MUST**：任何写入或运动之前先读 `0x0008`～`0x000D` 拼出 24 位大写 UID，并与清单绑定；不匹配立即拒绝（依据：`tools/robot_host.py:162-163`、`tools/robot_host.py:211-212`、`tools/robot_host.py:248-254`）。
- **C-02 MUST**：发命令前确认 `0x000F == 2`（依据：`tools/robot_host.py:99-100`、`BSP/robot.c:510`）。
- **C-03 MUST NOT**：把 UID、`0x0000` schema、`0x000F` 版本当作固件构建哈希或二进制身份证据（依据：任务书 §4.1、§11）。

**帧与交易**

- **C-04 MUST**：严格半双工、单条未完成事务；收到应答（或超时）之前不发下一帧。固件在发送忙或接收溢出时会**静默丢弃**请求（依据：`BSP/robot_port.c:160-164`）。
- **C-05 MUST**：发送前清空接收缓冲，帧间留 ≥4 ms 静默；帧内 MUST NOT 出现 >750 µs 的空隙（否则整帧被丢弃且无回复）（依据：`tools/robot_host.py:59-61`、`BSP/robot_rtu.c:11`）。
- **C-06 MUST**：帧长 ≤256 字节；读 ≤125 寄存器、写 ≤123 寄存器；`10` 的字节数必须等于 `2×count` 且总长精确（依据：`BSP/robot_protocol.c:16`、`BSP/robot_protocol.c:22`、`BSP/robot_protocol.c:33`）。
- **C-07 MUST**：校验应答 CRC、站号回显、功能码回显，以及 `06/10` 的字段回显；任一不符视为无应答（依据：`tools/robot_host.py:72-79`）。
- **C-08 MUST**：多寄存器读**不得跨未映射空洞**（例如 `0x000D..0x0010`、`B+0x04..B+0x08`、`B+0x1D..B+0x20` 之间要拆分），因为遇错整帧失败（依据：`BSP/robot.c:561`、第 2.1/2.2 节未映射表）。
- **C-09 MUST**：把交易超时设为 ≥1.5 s 以容纳 Flash 页擦除；MUST NOT 用几十毫秒级超时（依据：`tools/robot_host.py:64`、`BSP/robot_store.c:41-46`、`BSP/robot_port.c:156-165`）。

**序号与重放**

- **C-10 MUST**：所有命令使用非零序号（1..65535），由单一主站统一分配；`seq=0` 只可用于人工、不重试的一次性操作（依据：`tools/robot_host.py:97-98`、`BSP/robot.h:103`、任务书 §4.3）。
- **C-11 MUST**：超时重试用**完全相同的字节与相同序号**；**MUST NOT** 换新序号重发 MOVE（依据：`tools/robot_host.py:58-59`、`tools/robot_host.py:81`、`tools/robot_host.py:135`）。
- **C-12 MUST**：用 `0x0036/0x0037` 对账，并区分“异常 03 且 `0x0036` 未变＝序号被拒未执行”与“异常 03 且 `0x0036` 已更新＝参数被拒但序号已消耗”（依据：`BSP/robot.c:480-494`、`BSP/robot_protocol.c:12`）。
- **C-13 MUST NOT**：把 `0x0030`～`0x0034` 的命令快照当作“已执行”的证据——它在验证前就被覆盖（依据：`BSP/robot.c:573-574`）。
- **C-14 MUST**：假定序号只在半窗口内单调；`(seq−last_seq) mod 65536 ≥ 0x8000` 或与 `last_seq` 相等都会被拒（依据：`BSP/robot.c:484-485`）。
- **C-15 MUST**：重连/重启后先读 `0x000E`、`0x0036/0x0037` 与四路状态再决定；MUST NOT 续跑，MUST NOT 把旧 `DONE` 当新成功（依据：`BSP/robot.c:68`、`BSP/robot.c:509`、`BSP/robot.c:515-516`、任务书 §4.3）。
- **C-16 MUST**：把“`dirty=1` 期间发生重启”视为位置有效性丢失（四路 `valid` 会变 0），并视为潜在重放窗口（依据：`BSP/robot.c:75-76`、`BSP/robot.c:453-456`、`BSP/robot.c:360-362`）。

**命令语义**

- **C-17 MUST**：命令只用功能码 `10` 写 `0x0030` 起 5 个寄存器；**MUST NOT** 写 `0x0035`、**MUST NOT** 发送 `A55A` 提交帧、**MUST NOT** 用 `06` 或分段写触发命令（依据：`BSP/robot.c:573-577`、`BSP/robot.c:514`、`BSP/robot.c:564-568`、任务书 §4.1）。
- **C-18 MUST**：MOVE 参数限制在 −3600..3600 ddeg，按 `round(rad×180/π×10)` 换算，且**不得再乘减速比**；量化为 0 步时显式报 NO_OP，不等待新 `DONE`（依据：`BSP/robot.c:441`、`BSP/robot.c:450`、任务书 §5、§6、`tools/robot_host.py:279-281`）。
- **C-19 MUST**：把“受理”“运动完成”“已保存”“位置可信”当四件事分别报告——`state=DONE`、`fault=0`、`brake=1`、`duty=0` 才算完成，`0x0005/0x0006/0x0007` 全 0 才算保存（依据：`BSP/robot.c:356`、`BSP/robot.c:505-507`、`tools/robot_host.py:283`、任务书 §6）。
- **C-20 MUST NOT**：自动执行 ZERO、CLEAR_FAULT、SELFTEST 或 CONFIG；这四类都必须显式授权（ZERO 会改写零点并落盘，SELFTEST 会真实转动电机）（依据：`BSP/robot.c:429-438`、`BSP/robot.c:423-427`、`BSP/robot.c:163-200`、`tools/robot_host.py:242`、任务书 §9）。
- **C-21 MUST**：把 CONFIG 视为整板事务——先写全部暂存区、读回核对、再发一条 CONFIG，且只在整板静止时提交；MUST NOT 每个控制周期提交（依据：`BSP/robot.c:386-407`、`tools/robot_host.py:214-219`、任务书 §5）。
- **C-22 MUST**：知道 CONFIG 会清掉 gear/role/inverted 变化通道的 `valid` 与自检记录，并立即应用新 `pwm_hz`；而 address/node/protocol 只在重启后生效（依据：`BSP/robot.c:394-396`、`BSP/robot.c:402-406`、`BSP/robot.c:69-70`）。
- **C-23 MUST**：配置值严格落在合法域内（gear ∈ {0,82,290,516}、rpm 400..8000、pwm_hz 20000..30000、address 1..247、node 1..127、role 0..7 且非零唯一、enabled/inverted ∈ {0,1}、`enabled ⇒ gear≠0 且 role≠0`），因为**任一非法会使整条 CONFIG 返回 `R_VALUE`**（依据：`BSP/robot.c:11-23`、`BSP/robot.c:388`、`tools/robot_host.py:179-206`）。
- **C-24 MUST**：取消动作发 STOP/STOP_ALL 后等待停止确认（`state` 离开忙集合、`brake=1`、`duty=0`，自检还需 `phase=FINISHED`）；无回包时报告“停止未确认”（依据：`BSP/robot.c:29-30`、`BSP/robot.c:90-98`、`BSP/robot.c:290-291`、任务书 §4.3）。

**状态解读**

- **C-25 MUST NOT**：把 Modbus 异常码与业务结果码、电机故障码混用（异常 04 ≠ 故障 4；异常 03 覆盖 `R_VALUE` 与 `R_SEQUENCE` 两种原因）（依据：`BSP/robot_protocol.c:11-12`、`BSP/robot.h:27`、`BSP/robot.h:29`、任务书 §4.2）。
- **C-26 MUST NOT**：把 `state=UNREADY(0)`、`valid=0` 当作“禁止相对 MOVE”——MOVE 不要求 `valid=1`；同时 MUST NOT 在 `valid=0` 时提供绝对定位或伪造零点（依据：`BSP/robot.h:58`、`BSP/robot.c:443-446`、任务书 §5）。
- **C-27 MUST NOT**：把自检 `phase=FINISHED(5)` 当作通过；MUST 核对 `B+0x20 == PASSED(4)` 且原因为 0、保存标志全 0（依据：`BSP/robot.c:290-291`、`BSP/robot.c:298-303`）。
- **C-28 MUST NOT**：把 `residual`（`B+0x2E/0x2F`）当作通过/失败判据——它只记录、不判失败（依据：`BSP/robot.c:226-230`、`BSP/robot.c:289`）。
- **C-29 MUST**：把 `B+0x16` 的 rpm 理解为**减速前、无符号幅值、>250 ms 无边沿即 0**；需要带符号速度时必须注明方向来自命令而非测量（依据：`BSP/robot.c:530`、`BSP/capture.c:262`、`BSP/capture.h:14`、任务书 §5）。
- **C-30 MUST**：把 `B+0x15` duty 理解为低电平千分比（上限 950，自检上限 100），MUST NOT 当作转速百分比（依据：`BSP/robot.h:20`、`BSP/robot.h:21-22`、`BSP/robot.h:75-76`、`BSP/pwm.c:12-16`）。
- **C-31 MUST**：把 `pending_save=1` 理解为“等待静止后落盘”，不是错误；只有 `storage_error=1` 才是存储故障，且它会阻塞 MOVE/SELFTEST 与自动冲刷（依据：`BSP/robot.c:360-362`、`BSP/robot.c:449`、`BSP/robot.c:168`）。
- **C-32 MUST**：意外脉冲预算按**跨运动累计**处理，只有 CLEAR_FAULT/ZERO/上电才清零；长期运行后要主动核对 `F_UNEXPECTED(4)` 风险（依据：`BSP/robot.h:13`、`BSP/robot.c:31-37`、`BSP/robot.c:426`、`BSP/robot.c:437`）。
- **C-33 MUST NOT**：假定存在硬件急停输入——默认构建 `BUTTON_ESTOP_ENABLE = 0`，`stop_input` 恒 0；也 MUST NOT 把节点退出时的 STOP 当作独立急停（依据：`BSP/board_config.h:111`、`BSP/robot_port.c:240-259`、任务书 §5）。

**串口/资源纪律（部署侧）**

- **C-34 MUST**：只打开清单里记录的稳定路径（如 `/dev/serial/by-path/...`），连接后立刻核对 UID；MUST NOT 依赖裸 `ttyUSBn` 编号（依据：任务书 §2、`tools/robot_host.py:252-254`）。
- **C-35 MUST**：每条串口只有一个所有者线程/调度器；MUST NOT 在 ROS 定时回调里阻塞整个运动，MUST NOT 打开或杀掉其他业务占用中的串口（依据：任务书 §3、[compatibility_audit.md](compatibility_audit.md) 第 4.6 条）。

---

## 14. 本轮未验证项

### 14.1 明确未在实板验证的内容

本轮**没有**向板卡发送任何写命令或运动命令，因此以下内容全部**只来自源码**，未经实板验证：

1. 全部写入与命令路径：CONFIG / ZERO / MOVE / STOP / STOP_ALL / BRAKE / DIRECTION / SAVE / CLEAR_FAULT / SELFTEST / SELFTEST_ALL 的实际受理与拒绝行为（依据：`BSP/robot.c:364-465`）。
2. 全部时间常量的真实时序：500 ms 静止、1000 ms 反馈超时、10 ms 控制周期、2000 ms 自检通电/停止确认、监控超时公式（依据：`BSP/robot.h:11-26`、`BSP/robot.c:463`）。
3. RTU 成帧门限 750/1750 µs 的实际边界、错误字节与溢出丢弃行为、256/512 字节缓冲上限（依据：`BSP/robot_rtu.c:10-15`、`BSP/usart0.c:16`、`BSP/usart0.c:79-87`）。
4. 序号窗口、16 条去重缓存、失败结果入缓存、重启后序号持久化，以及第 4.4 节指出的“`dirty=1` 期间重启”重放窗口（依据：`BSP/robot.c:466-496`、`BSP/robot.c:453-456`）。
5. Flash 落盘与页擦除延迟、`save()` 的静止前置条件、`storage_error` 的置位/清除（依据：`BSP/robot_port.c:139-155`、`BSP/robot_store.c:31-58`）。
6. 到位容差 50 ddeg 的判超差行为、`F_POSITION`/`F_TIMEOUT`/`F_FEEDBACK`/`F_CAPTURE`/`F_UNEXPECTED` 的实际触发（依据：`BSP/robot.c:330-358`）。
7. 自检全流程：占空比斜坡 10/10 ms 上限 100、3 边沿目标、往返与残差、批量中止规则、硬件安全 tick 与意外预算扣减（依据：`BSP/robot.c:149-304`、`BSP/robot_port.c:31-51`、`BSP/robot_port.c:107-109`）。
8. **CAN 传输完全未实测**：本板 `0x0001=0`（Modbus），本轮未连接 CAN，第 12 节全部内容由源码得出（依据：`BSP/robot_protocol.c:42-55`、`BSP/robot_port.c:182-208`、第 14.2 节实测 `0x0001=0`）。
9. DIR 电平与机械正方向的对应关系、`inverted` 的正确取值、各关节机械限位（依据：`BSP/robot.c:458`、任务书 §2“仍需确认”）。
10. 板上二进制与本地源码是否一致：未读取固件哈希，UID 与 `0x000F` 都不能证明（依据：任务书 §4.1）。
11. 左手腕（M4）历史 RPM 异常（设定 1500 却读到约 5737/5847）根因未查明，本轮未复测（依据：任务书 §2“最新实测边界”）。
12. `BUTTON_ESTOP_ENABLE=1` 构建变体的行为（默认构建为 0）（依据：`BSP/board_config.h:111`）。

### 14.2 本轮唯一的实板证据（只读）

通过 `xinhua@192.168.1.113` 上的 `/dev/ttyUSB0`（稳定路径 `platform-fc800000.usb-usb-0:1.2:1.0-port0`，1a86:7523），115200 8N1，**仅发送功能码 03 读帧**，全部应答 CRC 校验通过：

| 读取 | 应答要点 |
|---|---|
| `01 03 00 08 00 06 44 0A` | UID = `C23B378115B092083830364A` |
| `01 03 00 0F 00 01 B4 09` | `0x000F = 2`（命令协议版本 2） |
| `01 03 00 01 00 04 15 C9` | `0x0001..0x0004 = 0, 1, 1, 25000`（Modbus、站号 1、节点 1、25 kHz） |
| `01 03 00 05 00 03 15 CA` | `dirty/storage_error/pending_save = 0/0/0` |
| `01 03 00 36 00 02 24 05` | `0x0036/0x0037 = 144 / 0` |
| `01 03 01 00 00 05 84 35` 等四路 | 生效配置：M1 `1/2/516/0/8000`、M2 `1/1/516/0/8000`、M3 `1/4/290/0/8000`、M4 `1/6/82/0/8000` |
| `01 03 01 10 00 07 04 31` 等四路 | 状态：四路 `fault = 0`；M3 `state=1(IDLE)`、`valid=1`；M1/M2/M4 `state=0(UNREADY)`、`valid=0`；四路 `brake=1`、`duty=0`；M2 `direction=1` |
| `01 03 01 18 00 06 44 33` / `01 03 01 D8 00 06 44 0F` | M2：角度 1079、计数 928、目标 1079；M4：角度 −1039、计数 −142、目标 −1039（与第 7.2 节公式一致） |
| `01 03 01 20 00 08 44 3A` 等四路 | 自检结果块全 0（本次上电后未做过自检） |

这些只读事实**只**证明：该板存在、站号 1、命令协议版本 2、当前无故障、保存标志干净、角度寄存器与 FG 计数满足第 7.2 节的换算式。它们不证明任何写路径、时序、互锁或二进制身份。

### 14.3 源码中未找到 / 无法回答的项

| 项 | 结论 | 依据 |
|---|---|---|
| `0x0010`～`0x001F` 的保留用途 | **源码中未找到**任何用途、命名或注释；读写均返回 `R_ADDRESS`（异常码 02） | `BSP/robot.c:497-556`、`BSP/robot.c:564-568` |
| 自检原因 `ST_RETURN_ERROR = 10` 的产生路径 | **源码中未找到**产生路径；`BSP/robot.h:36` 注释标注“10 保留给历史残差失败”，固件代码未使用该枚举 | `BSP/robot.h:34-36`（固件侧仅枚举定义；客户端名称表 `tools/robot_host.py:22`，阶段 B 产物 `ros2/.../protocol.py` 的 `SelftestReason.RETURN_ERROR` 亦仅作保留） |
| 故障码 `F_STORAGE = 6` 的置位路径 | 枚举存在，但 `BSP/robot.c` 中未找到把 `fault` 置为 `F_STORAGE` 的语句；存储故障通过板级 `0x0006` 与业务结果 `R_STORAGE` 表达 | `BSP/robot.h:29`、`BSP/robot.c:58`、`BSP/robot.c:506` |
| `0x0004` 与 `0x0023` 的一致性 | `0x0004` 读 `config.pwm_hz`（CONFIG 后立即更新），`0x0023` 读 `staged.pwm_hz`；两者在 CONFIG 成功后相等，但 CONFIG 之前可能不同 | `BSP/robot.c:504`、`BSP/robot.c:511-512`、`BSP/robot.c:400` |

### 14.4 与任务书第 4 节的不一致 / 需补充之处

以下是本文与 [ROS2_头部与双臂控制_代码生成执行方案.md](ROS2_头部与双臂控制_代码生成执行方案.md) 第 4 节（含被第 5 节引用的协议内容）逐条对照后发现的问题。**本轮未修改任务书**，仅在此列出：

1. **寄存器表不完整（§4.2）**：未列 `0x0000`（Flash schema）、`0x0001`～`0x0004`（生效 protocol/address/node/pwm_hz）、`0x000E`（have_seq）、`0x0020`～`0x0023`（暂存板级配置）。其中 `0x0020`～`0x0023` 是 CONFIG 事务的必经写入点（§7 的 YAML 与 `tools/robot_host.py:214-215` 都在用），但寄存器表里没有（依据：`BSP/robot.c:500-512`、`BSP/robot.c:566`）。
2. **未区分三个“版本”（§4.1）**：§4.1 只说“协议版本 `0x000F` 必须为 2”，未说明 `0x0000` 是 Flash schema、`0x0001` 是传输选择；§4.2 表里也没有 `0x0001`，容易被误当作“协议版本”（依据：`BSP/robot.c:500-501`、`BSP/robot.c:510`、`BSP/robot.h:6-8`）。
3. **CONFIG 不改变当前地址/节点/传输（§4.1 缺失）**：§4.1 说“固件选择 Modbus 或 CAN 之一”，但没写“**改写 `0x0020`～`0x0022` 并 CONFIG 后必须重启才生效**”，只有 `pwm_hz` 立即生效（依据：`BSP/robot.c:69-70`、`BSP/robot.c:400`、`BSP/robot.c:406`）。这是会直接导致“改了站号后板子失联”的陷阱。
4. **未标注只读/可写与未映射行为（§4.2）**：§4.2 的表没有访问属性，也没说明未映射地址返回异常码 02、以及**多寄存器读跨空洞会整帧失败**（依据：`BSP/robot.c:561`、`BSP/robot.c:564-568`、`BSP/robot.c:579`）。
5. **`0x0030`～`0x0034` 快照语义缺失（§4.2/§4.3）**：该区间可读，但内容在验证前就被覆盖，不能当作执行证据；§4 未提（依据：`BSP/robot.c:513`、`BSP/robot.c:573-574`）。
6. **异常码 03 的双重来源未说明（§4.2）**：§4.2 只强调“异常 04 不是电机故障 4”，但没说明**异常 03 同时覆盖 `R_VALUE` 与 `R_SEQUENCE`**，而两者对序号的影响完全相反（序号被拒不消耗、参数被拒会消耗）。区分方法是看 `0x0036` 是否已更新（依据：`BSP/robot_protocol.c:12`、`BSP/robot.c:480-494`）。
7. **自动序号会跳过缓存中的旧号（§4.3 缺失）**：§4.3 提到 16 条 RAM 缓存与半范围检查，但没提 `seq=0` 时固件会主动跳过缓存里已存在的序号（依据：`BSP/robot.c:471-478`）。
8. **CAN 的写入粒度与序号回显（§4.1 不完整）**：§4.1 写了“CAN 每次读 1～2 寄存器”，但没写 **CAN 每帧只能写 1 个寄存器**（CONFIG 需 24 次写 + 1 条命令帧）、没写命令应答只回显 `seqLo`（`p[3]=q[2]`）、也没写 CAN 应答第 3 字节直接是业务结果码而非异常码（依据：`BSP/robot_protocol.c:46-54`、`tools/robot_host.py:130-133`、`tools/robot_host.py:137-151`）。
9. **CAN 命令参数是 int16（§4.1 已写但未说明后果）**：§4.1 写了 `arg` 为 int16，但未说明它与 Modbus 的 int32 不同宽；虽然 MOVE 的 ±3600 不受影响，客户端仍不应假定两种传输参数宽度一致（依据：`BSP/robot_protocol.c:52`、`BSP/robot.c:575`、`BSP/robot.c:441`）。
10. **`B+0x16` rpm 与 `B+0x15` duty 的语义未标注（§4.2）**：§4.2 表只写“rpm”“duty”。rpm 是**减速前、无符号、>250 ms 无边沿即 0**（§5 提到无符号但未提 250 ms 与减速前）；duty 是**低电平千分比**，上限 950、自检上限 100（§4 完全未提）（依据：`BSP/capture.c:262`、`BSP/capture.h:14`、`BSP/robot.h:20`、`BSP/robot.h:21-22`、`BSP/robot.h:75-76`、`BSP/pwm.c:12-16`）。
11. **RTU 成帧门限未给出（§4.1 缺失）**：§4.1 只写“115200、8N1、二进制、不附 CR/LF”，未给固件实际使用的 **750 µs / 1750 µs** 门限与 256 字节上限——这直接决定客户端的帧间静默要求（依据：`BSP/robot_rtu.c:10-15`、`BSP/robot_rtu.h:5`）。
12. **“无应答”的多义性未展开（§4.3 不足）**：§4.3 说“读请求可有限重试、MOVE 超时先查询对账”，但没列出固件**静默丢弃**请求的全部条件（发送忙、接收溢出、地址不符、CRC 错、长度错、帧内空隙 >750 µs、超 256 字节）（依据：`BSP/robot_port.c:160-173`、`BSP/robot_protocol.c:16`、`BSP/robot_rtu.c:11`、`BSP/robot_rtu.c:14-15`）。
13. **落盘延迟对交易超时的影响未写（§4.1/§4.3 缺失）**：命令处理路径内同步写 Flash（含可能的整页擦除），应答会被延迟；`tools/robot_host.py:64` 用 1.5 s 并注明原因，§4 未提（依据：`BSP/robot_port.c:156-165`、`BSP/robot_store.c:41-46`、`tools/robot_host.py:64`）。
14. **MOVE 会先落盘 `dirty=1`（§4.2/§5 缺失）**：这既是“重启不可重放”的保护，也是 `dirty=1` 时不立即落盘的重放窗口来源；§4 未提（依据：`BSP/robot.c:453-456`、`BSP/robot.c:486-487`、`BSP/robot.c:360-362`）。
15. **急停互锁依赖编译开关（§4/§5 缺失）**：`stop_input` 互锁存在，但默认 `BUTTON_ESTOP_ENABLE = 0`，因此默认构建没有硬件急停输入；§5 只说“固件无通信心跳停机”（依据：`BSP/robot.c:367-368`、`BSP/board_config.h:111`、`BSP/robot_port.c:240-259`）。
16. **自检的硬件安全兜底与预算扣减未提（§4.1 opcode 表过简）**：`robot_selftest_safety_tick()` 是独立于状态机的第二道 2000 ms/边沿预算保护，且余量为 `10 − 已用 unexpected_fg`；§4 只把 SELFTEST 描述为“单路/四路逐一自检”（依据：`BSP/robot_port.c:31-51`、`BSP/robot_port.c:107-109`、`BSP/robot.h:13-14`）。
17. **`0x0035` 与旧 `A55A` 的具体拒绝形式（§4.1 可加强）**：§4.1 说“禁止写 `0x0035`”，本文补充其确切行为：读恒 0，写返回 `R_ADDRESS` → 异常码 **02**（依据：`BSP/robot.c:514`、`BSP/robot.c:564-568`）。
18. **`F_STORAGE = 6` 无置位路径（§4.2 故障表可能误导）**：§4.2 把故障 6 列为“存储”，但源码中未找到把 `fault` 置为 `F_STORAGE` 的语句；存储故障实际通过 `0x0006` 与 `R_STORAGE` 表达。客户端不应等待 `B+0x11 == 6`（依据：`BSP/robot.h:29`、`BSP/robot.c:58`、`BSP/robot.c:506`）。
19. **`ST_RETURN_ERROR = 10` 与 `0x0010`～`0x001F` 应标注为“未找到”**：§4 未涉及；本文第 14.3 节按要求如实记录为“源码中未找到”，未作补全（依据：`BSP/robot.h:34-36`、`BSP/robot.c:497-556`）。
20. **量化为 0 步的 MOVE 会报 DONE（§5/§6 未覆盖）**：`arg≠0` 但 `steps==0` 时（小减速比下的小角度），固件仍进入 RUNNING→SETTLING→DONE 而没有转动；§6 只说“参数量化为 0 时显式返回 NO_OP”，那是客户端义务，固件行为需另行说明（依据：`BSP/robot.c:448`、`BSP/robot.c:450`、`BSP/robot.c:332`、`BSP/robot.c:356`）。

---

## 附：本文的自检

- 全文代码围栏均成对闭合；表格列数自洽；文件为 UTF-8。
- 本文只新增 `docs/protocol_contract.md`，未修改任何其他文件；本轮未执行任何远程命令、未联网、未向板卡发送写命令或运动命令。
- 所有 `（依据：文件:行号）` 指向本次实际阅读过的文件；行号对应上表所列源码文件的本地当前内容（修改时间 2026-09-25 至 2026-09-29）。若源码变动，行号必须重新核对。
- 交叉引用：[ROS 2 旧工程兼容审查](compatibility_audit.md)（本文件的引用方）、[ROS 2 代码生成执行方案](ROS2_头部与双臂控制_代码生成执行方案.md)（任务书）、[Modbus 与 CAN 使用手册](../Robot_Modbus与CAN使用手册.md)、[单帧命令迁移](Robot_单帧命令迁移.md)、[无零点相对运动](../Robot_Modbus与CAN使用手册.md#unreferenced-relative)。本文与这些文档冲突时，**以源码为准**，并应回头修正文档。
