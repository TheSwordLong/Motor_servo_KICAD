# 历史参考：ROS2_双板实机测试方案

> 保留当时方案、观测及验证结果，不是现行操作步骤。历史“当前”仅指记录当天；请使用 [文档索引](../../../GD32F303VET6_MotorControl_ROS2/README.md)。原报文与测量值不改写。

> 后续更新：新固件故障4模拟规则已同步，头部400RPM的+5°及回零已通过真实ROS2执行。旧固定第11脉冲说明已过时，见[新固件测试记录](ROS2_新固件头部运动测试记录.md)。

> 当前日常操作请使用 [ROS2度数接口使用说明](../ROS2_使用与测试手册.md#degrees-interface)：angle_deg直接输入度数，motor_states_deg直接读度数。本文原弧度命令保留为兼容或历史参考，不改写已有实机日志。


> 2026-10-08 更新：已修复停止队列、配置一致性、头部关节白名单及绝对回零，软件回归通过。当前操作以[控制修复与 BRAKE 操作说明](ROS2_控制修复与BRAKE操作说明.md)为准；本次未执行真实电机测试。

<!--
  本文只存在于这个副本里：它于 2026-10-08 直接生成在 GD32F303VET6_MotorControl_ROS2\docs\，
  固件仓库 E:\...\GD32F303VET6_MotorControl\docs\ 下没有对应文件（原件已按用户要求移动至此）。
  因此 refresh_ros2_copy.ps1 之类的“固件仓库 -> 副本”同步不会更新或恢复它；
  改动本文请直接改这个文件。它不参与 colcon 构建，也不会被同步到远程主机。
-->
# ROS 2 双板实机测试方案（板 A 左臂+头部 / 板 B 右臂）

> 本文**只是方案**：生成于源码与文档核对之后，**没有执行其中任何一条命令**，没有修改代码，没有清故障，没有配置板卡，没有让任何电机运动。
> 所有"预期回读"和"判据"都标注了来源等级。执行时**一律以当场实测为准**，本文里的历史数值只作对照。

## 0. 证据分级（每一处结论都必须标其中之一）

| 标记 | 含义 | 本文中的例子 |
|---|---|---|
| **已实测** | 本轮或此前在真实硬件/真实 ROS 图上实际执行并留有原始回包 | 板 A 只读快照（用户提供，2026-10 域 72） |
| **历史记录** | 以前测过，但**不能**当作当前状态 | 板 B 的 by-path 映射、板 B 上次读到的 gear 516/516/0/516 |
| **代码推断** | 由固件或驱动源码推出，未在硬件上验证 | `storage_error` 是 RAM-only、只改 rpm 不清零点 |
| **待确认** | 必须由人/仪器给出，软件无法推断 | 右臂各通道减速比、真实机械正方向、机械限位 |

### 0.1 本文依据的源码事实（均为**代码推断**，行号可复核）

| 事实 | 出处 |
|---|---|
| `storage_error` 时 MOVE 直接返回 `R_STORAGE(4)` | `BSP/robot.c:449` |
| `storage_error` 时 SELFTEST / SELFTEST_ALL 返回 `R_STORAGE(4)` | `BSP/robot.c:168` |
| 有 fault、未使能或 gear=0 的通道 MOVE 返回 `R_UNREADY(3)` | `BSP/robot.c:446` |
| 后台补存条件 `pending_save && quiet(r) && !storage_error`，即 **`storage_error` 置位时永不自动重试** | `BSP/robot.c:360` |
| `persist()` 成功即清 `storage_error`；失败即置位 | `BSP/robot.c:55-61` |
| `robot_store_save()` 只有 5 个失败出口：页擦除、数据字编程、**编程后回读比对不一致**、COMMIT 字编程、最终 `valid()`（MAGIC/SCHEMA/长度/CRC32/COMMIT）校验 | `BSP/robot_store.c:44,51,54,55,57` |
| 上述失败**原因不通过 Modbus 暴露**，只有 `0x0006` 一个位 | `BSP/robot.c:506` |
| `storage_error`、`pending_save` 是 RAM 字段，`robot_init` 的 `memset` 每次上电清零 | `BSP/robot.h:88`、`BSP/robot.c:65` |
| `position[4]`、`valid[4]`、`request_sequence/request_seen` **在 flash 镜像内**，重启后恢复 | `BSP/robot.h:57-58`、`BSP/robot.c:52-53,68` |
| 重启时 `valid = (!dirty && valid[i]==1 && gear!=0)`：**镜像 dirty 则四路零点全丢** | `BSP/robot.c:75` |
| CONFIG 只有 `gear/role/inverted` 变化才清 `valid`；只有 `enabled/gear/role/inverted` 变化才清自检记录 → **只改 rpm 两者都不动** | `BSP/robot.c:394-396` |
| CONFIG 要求整板静止；成功后 `dirty/pending_save/storage_error` 全清；`pwm_hz` 立即生效，`protocol/address/node` 需断电重启 | `BSP/robot.c:387,400,406` |
| MOVE 的板上超时 `10000 + distance*gear*60000/(3600*rpm)*4` ms（**有 10 s 下限**） | `BSP/robot.c:463` |
| `robot_stop_all()` 只作用于**收到该帧的那一块板** | `BSP/robot.c:144-148` |
| `CLEAR_FAULT` 要求整板静止；只清 RAM 的 `fault`、`unexpected_fg` 计数与 `stop_fault_mask` 位；**不写 Flash、不动 `valid`** → 重启后故障会回来 | `BSP/robot.c:423-427` |
| `ZERO` 要求整板静止且 `gear!=0`（否则 UNREADY）；写 Flash，成功时一并清 `dirty/pending_save/storage_error`，并置 `position=0, valid=1, fault=0` | `BSP/robot.c:429-438` |
| MOVE 在被受理前会做一次 pre-write（`persist(...,1)`，即 dirty 记录），失败返回 STORAGE | `BSP/robot.c:453-455` |
| `config.py` 已支持 N 块板：按板隔离的通道唯一性、角色唯一性、`joints_for_board()`、`channel_to_joint(board,ch)`；Modbus 地址**不要求**唯一，只有 CAN node id 查同总线冲突 | `config.py:464-469,493-535,199-221` |
| `config.py` **不检查两块板是否指向同一个 by-path**（缺口） | `config.py:444-478` |
| 节点层只接受一块板：`len(boards) != 1` 抛 ConfigError | `motor_node.py:223-232` |
| launch 硬编码节点名，所有接口是绝对名 `/robot/*`（绝对名不受 `__ns` 重映射影响） | `bringup.launch.py:121`、`motor_node.py:317-326` |
| **fake 后端会采用 YAML 里配置的 UID**，所以 UID 相同不能证明是实板 | `motor_node.py:238` |
| fake 的可判定特征：日志含 `SIMULATED`、`ready: transport=fake` | `motor_node.py:240-242,192` |
| `MotorStates.header.frame_id` = board_name，并带 `uid`，话题可区分板；**服务/Action 名不含板标识** | `motor_node.py:_states_message` |
| 驱动侧新鲜度门限 `STALE_AFTER_SEC=1.0`，`timeout_sec` 合法区间 0.5～300 | `motor_node.py:96-99` |
| 只读模式拒绝**一切**写，包括 STOP | `board_session.py:_guard_write` |
| UNKNOWN 停机期间**只有** STOP/STOP_ALL 可发，BRAKE 也不行；停机成功不清 UNKNOWN | `board_session.py:STOP_OPCODES` |

### 0.2 域号与目录约定

| 域 | 用途 |
|---|---|
| **0** | **禁用**。主机上有在跑的实机控制栈（controller_manager、joint_state_broadcaster、多个 robot_state_publisher） |
| 71 | 现有 fake 节点（`transport=fake, allow_motion=true, commissioning=true`）。不要复用它做本文任何判据 |
| 72 | 板 A 实板 |
| 74 | 板 B 实板 |
| 73 | fake / 双 fake 软件测试 |

证据目录：`/data/ros2/evidence/<阶段>_<板名>_<日期时间>/`，每条命令用 `tee` 原样落盘。**每个终端都要 export 同一个域号**，否则 CLI 与节点不在同一张图上（见阶段 0.4）。

---

## 阶段 0：版本与环境

### 前提
- SSH 到 `xinhua@192.168.1.113` 成功；`/data` 分区有足够空间（历史为 60% 已用，工作区必须留在 `/data`，根分区历史只剩约 1 GB）。
- 不接串口也能做完全阶段 0。

### 命令

```bash
# 0.1 环境（每开一个终端都要做；set +u 是必需的）
set +u; source /opt/ros/humble/setup.bash
source /data/ros2/robot_motor_ros2_ws/install/setup.bash; set -u
export ROS_DOMAIN_ID=73
cd /data/ros2/robot_motor_ros2_ws
{ date -Is; hostname; uname -m; python3 --version; echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID";
  ros2 pkg prefix robot_motor_driver; ros2 pkg prefix robot_motor_bringup;
  ros2 pkg prefix robot_motor_interfaces; } | tee /data/ros2/evidence/S0_env.txt
```

> `set +u` 的原因：`/opt/ros/humble/setup.bash` 引用未定义的 `COLCON_TRACE`，带 `set -u` 会在 source 阶段直接失败（**代码推断**，此前已踩到）。

```bash
# 0.2 源码版本核对：远程 src/ 必须与权威 ros2/ 逐文件一致
find src -type f ! -path '*__pycache__*' ! -name '*.pyc' | sort | \
  while read -r f; do printf "%s  %s\n" "$(sha256sum "$f" | cut -c1-16)" "$f"; done \
  | tee /data/ros2/evidence/S0_src_hashes.txt
```

在本地（Windows）对权威源做同样的事并逐行比对：

```powershell
$ws='E:\Date_save\embedded_code\GD32Projects\GD32F303\GD32F303VET6_MotorControl\ros2'
Get-ChildItem $ws -Recurse -File | Where-Object { $_.FullName -notmatch '__pycache__' -and $_.Extension -ne '.pyc' } |
  ForEach-Object { "{0}  src/{1}" -f (Get-FileHash $_.FullName).Hash.Substring(0,16).ToLower(),
    $_.FullName.Substring($ws.Length+1).Replace('\','/') } | Sort-Object
```

必须包含本轮多板/接口相关的文件（缺任何一个就说明远程是旧包）：

```
c34c4a233982c343  src/robot_motor_interfaces/msg/SelftestChannel.msg
2474f9153af8e257  src/robot_motor_interfaces/action/RunSelfTest.action
f969d28704a1538f  src/robot_motor_interfaces/msg/MotorStates.msg
9c6fb383c4bcf44c  src/robot_motor_interfaces/srv/ApplyConfig.srv
cd211567f05b5b40  src/robot_motor_driver/robot_motor_driver/board_session.py
fc01f364eae18035  src/robot_motor_driver/robot_motor_driver/scheduler.py
e40bf09763143d68  src/robot_motor_driver/robot_motor_driver/motor_node.py
2a3199db713f8b14  src/robot_motor_driver/robot_motor_driver/protocol.py
7799658787d7e37d  src/robot_motor_driver/robot_motor_driver/config.py
504f104c28b3c5e5  src/robot_motor_driver/robot_motor_driver/fake_board.py
```

```bash
# 0.3 构建与测试
~/.local/bin/colcon build --symlink-install 2>&1 | tail -25
~/.local/bin/colcon test 2>&1 | tail -15
~/.local/bin/colcon test-result --verbose 2>&1 | tail -40
```

```bash
# 0.4 接口生成与 Python 导入
ros2 interface list | grep robot_motor_interfaces
ros2 interface show robot_motor_interfaces/msg/SelftestChannel
ros2 interface show robot_motor_interfaces/action/RunSelfTest | grep -nE "batch_completed|all_passed|channels|active_channel"
ros2 interface show robot_motor_interfaces/srv/ApplyConfig    | grep -n "bool changed"
ros2 interface show robot_motor_interfaces/msg/MotorStates    | grep -nE "commissioning|soft_limits_enforced"
python3 -c "from robot_motor_interfaces.msg import SelftestChannel, MotorStates; \
from robot_motor_interfaces.action import RunSelfTest; from robot_motor_interfaces.srv import ApplyConfig; \
print(len(SelftestChannel.get_fields_and_field_types()), RunSelfTest.Result().batch_completed, \
ApplyConfig.Response().changed, MotorStates().soft_limits_enforced)"
python3 -c "import robot_motor_driver.protocol as p, robot_motor_driver.config as c; \
print(p.validate_board_image([p.ChannelConfigValues(True,1,516,0,8000)]*4)[:1]); \
print([m for m in ('joints_for_board','channel_to_joint') if hasattr(c.RobotConfig, m)])"
```

### 预期回读
- `colcon build`：`Summary: 3 packages finished`。`robot_motor_interfaces` 因含 `SelftestChannel.msg` 会重新生成类型（历史耗时约 1 min 28 s），只有 `python_cmake_module` 的 CMP0148 policy 警告。
- `colcon test-result`：`416 tests, 0 errors, 0 failures, 0 skipped`。分包基线：launch 9、can_codec 19、config 89、fake_board 42、node 55、protocol 81、scheduler 43、session 49、transport 20。
- 接口：`SelftestChannel` 13 个字段；`batch_completed/all_passed/channels/active_channel/changed/commissioning/soft_limits_enforced` 均存在。
- 最后一条 python：打印一条 role 重复的拒绝原因，以及 `['joints_for_board', 'channel_to_joint']`（证明多板配置 API 已在）。

### 通过判据
上述四项全部满足，且 `S0_src_hashes.txt` 与本地输出逐行一致。

### 失败后停止条件
- 哈希不一致 → **停止**。远程被改过或没同步；先查清差异来源，不要"就地修一下"，权威源是固件仓库的 `ros2/`。
- build 或 test 失败 → **停止**。**不得把旧 install 当新代码用**：`ros2 interface show` 看不到 `SelftestChannel` 就说明 install 是旧的，此后所有双板/自检结论都无效。
- `ros2 pkg prefix` 指向 `/opt/ros/motor_control_ws` 之类 → 停止，source 顺序错了（旧栈的 install 覆盖了这个工作区）。

### 0.5 `stage_c_realboard_readonly.sh` 的域号问题（必须知道）

脚本在 `src/robot_motor_bringup/scripts/stage_c_realboard_readonly.sh`。第 25 行是 `export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-71}"`（可被外部环境变量覆盖），但第 48-50 行的 launch 命令是：

```bash
ros2 launch robot_motor_bringup bringup.launch.py \
  transport:=modbus_rtu allow_motion:=false maintenance:=false
```

**没有传 `ros_domain_id`**。而 `bringup.launch.py` 里有 `DeclareLaunchArgument('ros_domain_id', default_value='71')` 和 `SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id)` —— launch 会给**它启动的节点进程**重新设置域号，覆盖 shell 里 export 的值。结果：`ROS_DOMAIN_ID=72 bash stage_c...sh` 会让**脚本里的 `ros2 topic echo`/`service call` 在 72，而节点在 71**，所有查询超时或空返回，看起来像"板子没响应"。

处理方式（本文采用第 1 种，不改任何文件）：
1. **不用脚本**，直接在各阶段命令里显式写 `ros_domain_id:=<域号>`。
2. 若必须用脚本：`ROS_DOMAIN_ID=72` 之外，还要把 launch 那行补成 `... maintenance:=false ros_domain_id:="${ROS_DOMAIN_ID}"`。这属于修改仓库文件，需另行授权并重新同步到远程。

`stage_d_fake_acceptance.sh` 同样有问题，而且更严重：第 52/53/123/129 行仍是 `motor_rpm: 400`（板上是 8000 → 新驱动返回 `error_code=111`），第 99 行 launch 没有 `commissioning:=true`（`limits_rad: null` → 返回 `112`）。**该脚本的旧输出不能作为新版通过证据**，必须重跑本文阶段 B/3 的命令。

同理：任何"只 export 域号"的做法都不可靠。**判断自己在哪张图上，用 `ros2 node list` + `ros2 param get /robot_motor_driver transport`，不要靠域号记忆。**

### 记录内容
`S0_env.txt`、`S0_src_hashes.txt`、build/test 完整输出、接口 show 输出、本地哈希比对结果、执行者与时间。

---

## 阶段 1：两块板分别只读

### 前提
- 阶段 0 全绿。
- 两块板都已上电并接好各自的 CH340；**两条 TTL 线各自独立，绝不并成一条 RS-485 总线**（两板 Modbus 地址都是 1，并线会直接冲突）。
- 门锁网关串口全程不碰：历史为 `/dev/serial/by-path/platform-xhci-hcd.5.auto-usb-0:1.2:1.0-port0` → `ttyUSB1`，由 `gateway.kafka_server` 持有（**历史记录**，当场必须重新确认）。
- 本阶段**只读**：`allow_motion:=false`、`maintenance:=false`，驱动进入只读模式，连 `/robot/stop` 都会被拒（设计如此）。

### 命令

```bash
# 1.1 端口普查（不开任何端口）
mkdir -p /data/ros2/evidence/S1_ports && cd /data/ros2/evidence/S1_ports
{ date -Is
  echo "--- by-path"; ls -l /dev/serial/by-path/
  echo "--- by-id  "; ls -l /dev/serial/by-id/ 2>&1
  echo "--- ttyUSB "; ls -l /dev/ttyUSB* 2>&1
  for p in /dev/serial/by-path/*; do echo "$p -> $(readlink -f "$p")"; done
  echo "--- usb devices (CH340 = 1a86:7523, CP210x = 10c4:ea60)"
  lsusb | grep -iE "1a86|10c4|serial" || true
  echo "--- holders"; fuser -v /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2 /dev/ttyUSB3 2>&1
  echo "--- door lock (must NOT be touched)"; readlink -f /dev/door_lock_uart 2>&1
} | tee s1_ports.txt
```

```bash
# 1.2 板 A 只读（域 72）。PORT_A 用 by-path，绝不用裸 ttyUSBn
export ROS_DOMAIN_ID=72
OUT=/data/ros2/evidence/S1_boardA_$(date +%Y%m%d_%H%M%S); mkdir -p "$OUT"; cd "$OUT"
PORT_A=/dev/serial/by-path/platform-fc800000.usb-usb-0:1.2:1.0-port0
{ date -Is; echo "PORT_A=$PORT_A -> $(readlink -f $PORT_A)"; fuser -v "$PORT_A" 2>&1; } | tee preflight.txt
# 若已被占用 -> 停止，不要抢总线
timeout 150 ros2 launch robot_motor_bringup bringup.launch.py \
  transport:=modbus_rtu allow_motion:=false maintenance:=false \
  ros_domain_id:=72 > node.log 2>&1 &
sleep 20
grep -E "ready: transport=|identified|SIMULATED" node.log | tee ident.txt
timeout 40 ros2 param get /robot_motor_driver transport          | tee param_transport.txt
timeout 40 ros2 service call /robot/board_identity std_srvs/srv/Trigger "{}" | tee identity.txt
timeout 40 ros2 topic echo /robot/motor_states --once            > states.yaml
timeout 40 ros2 topic echo /robot/diagnostics_json --once        > diag.txt
timeout 40 ros2 service call /robot/stop robot_motor_interfaces/srv/Stop \
  "{joint_name: 'head_joint'}"                                   | tee stop_refused.txt
for i in 1 2 3; do timeout 30 ros2 topic echo /robot/motor_states --once \
  | grep -E "^uid|^stale|^age_sec|^last_sequence|^dirty|^storage_error|^pending_save" >> series.txt; sleep 3; done
kill %1 2>/dev/null; wait 2>/dev/null; sleep 2
grep -c "TX " node.log | tee tx_count.txt
```

```bash
# 1.3 板 B 只读（域 74）——先确认 by-path 还在，历史值不可信
export ROS_DOMAIN_ID=74
OUT=/data/ros2/evidence/S1_boardB_$(date +%Y%m%d_%H%M%S); mkdir -p "$OUT"; cd "$OUT"
PORT_B=/dev/serial/by-path/platform-fc800000.usb-usb-0:1.4:1.0-port0
{ date -Is; echo "PORT_B=$PORT_B -> $(readlink -f $PORT_B 2>&1)"; fuser -v "$PORT_B" 2>&1; } | tee preflight.txt
timeout 150 ros2 launch robot_motor_bringup bringup.launch.py \
  transport:=modbus_rtu allow_motion:=false maintenance:=false \
  config_file:=/data/ros2/robot_motor_ros2_ws/src/robot_motor_bringup/config/robot_right_arm_UNCONFIRMED_GEARS.yaml \
  ros_domain_id:=74 > node.log 2>&1 &
sleep 20
grep -E "ready: transport=|identified|SIMULATED|does not match" node.log | tee ident.txt
timeout 40 ros2 service call /robot/board_identity std_srvs/srv/Trigger "{}" | tee identity.txt
timeout 40 ros2 topic echo /robot/motor_states --once > states.yaml
timeout 40 ros2 topic echo /robot/diagnostics_json --once > diag.txt
kill %1 2>/dev/null; wait 2>/dev/null; sleep 2
grep -c "TX " node.log | tee tx_count.txt
```

`states.yaml` 的逐通道打表（两块板都用）：

```bash
python3 - <<'PY'
import re
t = open('states.yaml').read()
head = dict(re.findall(r'^(uid|command_protocol|identity_ok|communication_ok|stale|age_sec|last_sequence|last_result|dirty|storage_error|pending_save|read_only|commissioning|soft_limits_enforced): (.*)$', t, re.M))
for k, v in head.items(): print("  %-22s %s" % (k, v))
for b in re.split(r'\n- channel: ', t)[1:]:
    kv = dict(re.findall(r'^\s+(\w+): (.*)$', b, re.M))
    print("  ch%s role=%s en=%s gear=%s rpm=%s inv=%s state=%s fault=%s valid=%s brake=%s duty=%s mrpm=%s angle_deci=%s fg=%s target_deci=%s st_state=%s st_reason=%s"
          % (b.splitlines()[0].strip(), kv.get('role'), kv.get('enabled'), kv.get('gear_ratio'),
             kv.get('configured_rpm'), kv.get('inverted'), kv.get('state'), kv.get('fault'),
             kv.get('position_valid'), kv.get('brake'), kv.get('duty_permille'), kv.get('measured_rpm'),
             kv.get('angle_deci_deg'), kv.get('fg_count'), kv.get('target_deci_deg'),
             kv.get('selftest_state'), kv.get('selftest_reason')))
PY
```

### 预期回读

板 A（**历史记录**，必须当场重测）：`uid=C23B378115B092083830364A`、`command_protocol=2`、`identity_ok=true`、`communication_ok=true`、`stale=false`、`dirty=false`、`storage_error=true`、`pending_save=true`；M1(ch0) role2 gear516 valid=false fault=0；M2(ch1) role1 gear516 valid=false **fault=4**；M3(ch2) role4 gear290 **valid=true** fault=0；M4(ch3) role6 gear82 valid=false **fault=4**；四路 `configured_rpm=8000`；`last_sequence` 历史为 178。

板 B（**历史记录 + 待确认**）：`uid=C23B37C715B092083830364A`、`command_protocol=2`、`last_sequence` 历史为 28、save 标志历史为 0/0/0；板上配置历史为 M1 en=0 role=0 gear=516、M2 en=0 role=0 gear=516、M3 en=0 role=0 **gear=0**、M4 **en=1 role=2(left_upper)** gear=516。注意 M4 的 role=2 是左臂角色，是本地测试留下的，**对右臂是错的**（待确认项，见阶段 2）。

### 通过判据
1. 两块板的 `ident.txt` 都是 `ready: transport=modbus_rtu port=<对应 by-path>`，且**都不含** `SIMULATED`；`param_transport.txt` 为 `modbus_rtu`。
2. `identity.txt` 的 UID 与配置期望**逐字相符**（板 A `C23B3781...`，板 B `C23B37C7...`）。UID 是唯一的板身份绑定手段。
3. `tx_count.txt` = **0**（两块板都是）。非 0 就不是只读运行，本阶段结论作废。
4. `stop_refused.txt`（板 A）为 `accepted=False` 且消息含 `read-only`。
5. `age_sec` 在 5 Hz 轮询下应稳定小于 1.0，`stale=false`。
6. 板 B 的 by-path 存在且解析到的 ttyUSB 与板 A **不是同一个**。

### 失败后停止条件
- **UID 不符 / `identity_ok=false` / 消息含 `does not match`** → 立即停止，**禁止任何写操作**。这说明端口与板对应关系变了（拔插、重新枚举、hub 顺序变化）。重新做 1.1 普查，确认哪块板在哪个口，再改配置里的 UID 或端口；**不要为了让它通过而改期望 UID**。
- 两个 by-path 解析到同一个 ttyUSB，或板 B 的口不存在 → 停止。接线/枚举问题，先解决物理连接。
- 任一端口的 `fuser` 显示被别的进程持有 → 停止，先确认那是谁（可能是门锁网关或别人的会话），**不要 kill 别人的进程**。
- `communication_ok=false` / `stale=true` / `age_sec>1.0` → 停止本板测试。这是链路问题（线缆、hub、波特率、地址），不是板卡业务状态；在链路稳定前做任何写操作都可能变成 UNKNOWN。
- 两次只读之间 `last_sequence` **自行前进** → 立即停止。说明有另一个主站在写这块板（总线争用），任何清故障/配置动作在找出对方之前都不得执行。
- 板 B 出现 `storage_error=true` → 按板 A 同样处理：不安排自检、配置提交或运动。

### 记录内容
`preflight.txt`（含 by-path→ttyUSB 解析与占用者）、`ident.txt`、`param_transport.txt`、`identity.txt`、完整 `states.yaml`、`diag.txt`、`stop_refused.txt`、`series.txt`、`tx_count.txt`、域号、执行者与起止时间。**UID 与 by-path 必须成对记录**，只记 ttyUSB 编号无效。

> 说明：多板节点尚不存在，本阶段用**两个域 + 两个进程**分别只读。这**不是**双板整合测试，不能写成"双板已联调"。整合的准入见阶段 3。

---

## 阶段 2：解除实物测试阻塞项

### 2A 板 A：`storage_error=true` + `pending_save=true` + M2/M4 `fault=4`

#### 前提
阶段 1 板 A 通过；`dirty=false`（若为 true 见下面的停止条件）；本小节全程只读，**不含任何写命令**。

#### 只读能做的排查顺序

```bash
export ROS_DOMAIN_ID=72
OUT=/data/ros2/evidence/S2A_boardA_$(date +%Y%m%d_%H%M%S); mkdir -p "$OUT"; cd "$OUT"
# 2A.1 长时序观测：2 分钟内每 3 s 一次全量快照，期间任何人不得接触机器人
timeout 220 ros2 launch robot_motor_bringup bringup.launch.py \
  transport:=modbus_rtu allow_motion:=false maintenance:=false ros_domain_id:=72 > node.log 2>&1 &
sleep 20
for i in $(seq 1 40); do
  { echo "### sample $i $(date -Is)"
    timeout 20 ros2 topic echo /robot/motor_states --once \
      | grep -E "^uid|^identity_ok|^communication_ok|^stale|^age_sec|^last_sequence|^last_result|^dirty|^storage_error|^pending_save|^unknown_command_state"
    timeout 20 ros2 topic echo /robot/motor_states --once \
      | grep -E "^\s+(channel|state|fault|position_valid|fg_count|angle_deci_deg|duty_permille|brake|measured_rpm):"
  } >> series.txt
  sleep 3
done
kill %1 2>/dev/null; wait 2>/dev/null; sleep 2
grep -c "TX " node.log | tee tx_count.txt
python3 - <<'PY' | tee drift.txt
import re
blocks = re.split(r'### sample ', open('series.txt').read())[1:]
first, last = blocks[0], blocks[-1]
def vals(b): return re.findall(r'fg_count: (-?\d+)', b), re.findall(r'angle_deci_deg: (-?\d+)', b)
print("samples:", len(blocks))
print("first fg/angle:", vals(first))
print("last  fg/angle:", vals(last))
print("last_sequence first/last:", re.findall(r'last_sequence: (\d+)', first)[:1],
      re.findall(r'last_sequence: (\d+)', last)[:1])
print("storage_error set:", set(re.findall(r'storage_error: (\w+)', open('series.txt').read())))
print("pending_save set:", set(re.findall(r'pending_save: (\w+)', open('series.txt').read())))
print("fault values seen:", sorted(set(re.findall(r'fault: (\d+)', open('series.txt').read()))))
PY
```

#### 判读表（**代码推断** + 只读实测）

| 观测 | 结论 | 动作 / 停止条件 |
|---|---|---|
| 四路 `fg_count`、`angle_deci_deg` 全程逐字不变；`last_sequence` 不变；`storage_error`/`pending_save` 恒为 true | 无实时 FG 活动。fault 4 与 storage_error 是**历史锁存**（`storage_error` 置位时后台补存被抑制，`robot.c:360`，所以它会一直挂着，这本身不说明"正在失败"） | 只读手段已到头。进入 2A 的"需要写才能确认"部分（须授权） |
| 某故障通道 `fg_count`/`angle_deci_deg` 在无人接触时自行变化 | 存在实时 FG 边沿源：屏蔽/接地/接线/编码器噪声，或有人动了机构 | **停止全部计划**。不要申请清故障（清了会复发，只会掩盖问题）。记录是哪一路、每秒变化量、当时附近有无其他设备动作，转硬件排查 |
| `last_sequence` 无人发命令却前进 | 另一个主站在写这块板 | **立即停止**。先找出对方进程/设备并断开，确认总线独占后才可继续 |
| `storage_error` 或 `pending_save` 自行清零 | 与 `robot.c:360` 矛盾 | 停止。记录时间戳与前后完整快照，先核对板上固件版本是否与本仓库一致 |
| `dirty` 由 false 变 true | 有命令被受理但终态未落盘 | 停止。此时**断电重启会导致四路零点全丢**（`robot.c:75`），必须先查清命令来源 |
| `identity_ok/communication_ok=false` 或 `stale=true` | 链路问题 | 停止本节，回阶段 1 排查线缆/hub/占用 |

#### 只读**不能**回答、必须靠板上测量或维修的问题
1. **`storage_error` 的具体成因**。`robot_store_save()` 有 5 个失败出口（页擦除失败、数据字编程失败、**编程后回读比对不一致**、COMMIT 字编程失败、最终 `valid()` 校验失败含 CRC32），但 Modbus 只暴露 `0x0006` 一个位（`robot.c:506`）。**只读无法区分它们**。要区分只能：(a) 做一次授权写入并看结果；(b) 在板上加调试输出重编固件（本轮禁止烧录）；(c) 用调试器读 flash 槽位内容。
2. **是否 Flash 介质损坏**。不能只凭 `storage_error=true` 断定芯片坏。同样可能的原因：擦除/编程时序或供电瞬态、写保护位被置、槽位被上次掉电打断的半写记录占满导致每次都要擦页（`robot_store.c:35-46` 的乒乓槽位分配）、以及"保存窗口内有 FG 活动导致固件拒绝擦写"（`robot.c:275-277` 明确写了"绝不在仍有脉冲时擦写"）。**在拿到 (a)/(b)/(c) 任一证据前，记录里只能写"保存失败原因未定"。**
3. **fault 4 的物理来源**。`F_UNEXPECTED` 由累计超过预算的非预期 FG 边沿触发（`robot.c:31-37`），只有上电、`CLEAR_FAULT` 或 `ZERO` 会清。手转关节、EMI、接线破损都会造成，只读无法区分——2A.1 的漂移观测只能排除"现在还在发生"，不能确定"当初是什么"。
4. **M2/M4 的机械与电气健康**。需要万用表/示波器测 FG 与 DIR/BRAKE/PWM 电平，属硬件工作。

#### 关于"断电重启"（**不是**只读操作，须单独授权）
`robot_init` 的 `memset` 会清掉 `storage_error`/`pending_save`（RAM-only），`position[]`/`valid[]`/`last_seq` 从 flash 恢复，且当前 `dirty=false` 意味着**M3 的零点会保留**（`robot.c:75`）。所以一次断电重启很可能让板 A 回到"无 storage_error、M3 valid=1、M2/M4 仍 fault=4（fault 也在 RAM，会一并清掉）"的状态。

但这**会销毁证据**：重启后无法再判断 `storage_error` 是何时、因何置位的，也无法确认 `last_sequence` 是否被成功落盘。因此顺序必须是：**先完成 2A.1 的全部只读记录并归档 → 再授权断电重启 → 重启后立刻重跑阶段 1 板 A 只读并对比 `last_sequence`/`valid`/`fault`/save 标志**。若重启后 `storage_error` 又在下一次写操作时出现，说明是**可复现的**保存失败，必须转 (a)/(b)/(c) 定位，不得继续推进阶段 5。

### 2B 板 B：必须由操作者确认的清单（全部是**待确认**，软件无法推断）

| 项 | 为什么不能推断 | 需要谁/什么来确认 | 记录格式 |
|---|---|---|---|
| 通道 → 关节映射（哪一路是右大臂/右小臂/右手腕） | 左臂是 M1 大臂 / M2 头 / M3 小臂 / M4 手腕；右臂**没有头部电机**，所以槽位分布不一定相同。板上现存 M4 role=2(left_upper) 是本地测试残留，**不是**右臂事实 | 操作者对照接线与机械实物逐路指认；必要时断电后手动轻转某一路，观察哪个关节动（**断电状态下**，不通电） | `ch<n> -> <关节名> <证据:接线照片/手动确认>` |
| 每路实际减速比 | 减速比是机械属性，寄存器里的 gear 只是"曾经被写进去的值"。板 B 历史为 516/516/0/516，与左臂的 516/290/82 不一致 | 读电机/减速器铭牌，或拆检；由操作者书面给出 | `ch<n> gear=<值> 来源=<铭牌/拆检/图纸>` |
| 每路 role 值 | role 决定固件语义与角色唯一性校验（`robot.c:20`）；右臂应为 3/5/7，但要先确定哪一路对应哪个关节 | 同上，映射确定后按 `3=right_upper, 5=right_forearm, 7=right_palm` 填 | `ch<n> role=<0..7>` |
| `inverted`（0/1） | 固件用 `(arg<0) XOR inverted` 决定 DIR 电平（`robot.c:458`），接反则正负角度与机械方向相反 | 必须在**降速后的小角度通电测试**中由人观察确定（阶段 5），不能从寄存器推 | `ch<n> inverted=<0/1> 观察=<正角度往哪动>` |
| 真实机械正方向 | 单线 FG 只给脉冲数，**测不出转向**；`measured_rpm` 是无符号幅值，`direction` 只是 DIR 电平不是实测转向 | 人眼观察 + 独立角度测量（量角器/刻度盘/相机） | `正角度 -> <方向描述>，观察者=<姓名>` |
| 机械限位（`limits_rad`） | 当前是 `null`。驱动不会假装有：`limits_rad: null` 时**默认拒绝运动**（`error_code=112`），只有 `commissioning:=true` 才放行且**无软限位保护** | 手动（断电）把关节走到两个机械极限，用独立方式量角，留出安全余量 | `ch<n> limits_rad=[<min>,<max>] 量法=<工具> 余量=<deg>` |
| 头部槽位（M2）处理 | 右臂无头部电机，该通道可能空接或未接线 | 操作者确认是否接线；若未接，保持 `enabled=0 role=0`，**不要**把"四路自检全通过"当作右臂准入条件 | `ch1 状态=<空接/未接线/其他>，处理方式=<保持禁用>` |

#### 板 B 的准入判据（与左臂不同，**不要求四路**）
只把**实际接入电机的通道**纳入准入。判定方式：断电状态下逐路轻转电机，确认哪个关节跟着动；未接电机的通道保持 `enabled=0`、`role=0`、`gear=0`，并在记录里写明"该通道无电机，不参与准入"。**不得**用 `SELFTEST_ALL` 全通过作为右臂准入条件——空接通道会因无 FG 反馈而失败，那是预期结果，不是缺陷。

#### 停止条件
- 上述任一项没有书面确认 → 板 B 停在只读，不进入阶段 5。
- 板 B 的 `robot_right_arm_UNCONFIRMED_GEARS.yaml` 里三个 `gear_ratio`（516/290/82）是按左臂**推定**的，与板上历史值不符。**在操作者给出实测减速比之前，该文件只能 `allow_motion:=false` 只读使用**；文件名里的 `UNCONFIRMED_GEARS` 就是提醒，改名即代表已确认，是一个需要留痕的动作。
- 减速比填错的后果不是 cosmetic：固件按 `steps=(arg*gear*6±1800)/3600` 换算（`robot.c:450`），若配置值大于真实值，实际转角会**大于**指令角（例如配 516 实为 82 时，5° 指令会变成约 31.7°）。

### 记录内容
`series.txt`、`drift.txt`、`tx_count.txt`、判读表命中的哪一行、2B 清单逐项的确认人与确认方式、板 B 各通道是否接电机。

---

## 阶段 3：多板软件整合的准入测试

> **当前状态（2026-10-08 更新：已实现，但架构与本文原设想不同）**
>
> 多板支持已经落地，采用的**不是**"一个节点驱动两块板"，而是**一块板一个节点进程**：
> - `_select_board()` 支持多板 YAML，用新参数 `board_name` 选板；板数 >1 且未指定 `board_name` 时**明确拒绝启动并拒绝猜测**（消息里列出已声明的板名）；`board_name` 不存在也同样拒绝。
> - 节点接口名全部改为**相对名**（`robot/stop`、`robot/motor_states`、`robot/diagnostics_json`、`joint_states`），只有 `/diagnostics` 保持绝对（它本来就是允许多发布者的聚合话题）。namespace 因此决定最终名字：无 namespace → `/robot/*`（**与历史完全兼容**），`board_namespace:=board_b` → `/board_b/robot/*`。
> - `bringup.launch.py` 新增 `board_name` / `board_namespace`；新增 `dual_board.launch.py`（两个 Node、各自 namespace 与各自策略；`board_b_start` 默认 `false`，两侧 `allow_motion` 默认 `false`）。
> - 新增 `config/robot_dual_UNCONFIRMED_RIGHT_ARM.yaml`：两板声明在同一份文件、`allow_motion: false`，头部写明右臂减速比仍未确认。
> - `config.py` 补上原先缺的校验：**两块板不得共用同一个非空 `port`**（重复 UID 原本已拒；`address` 允许相同，因为两板各走独立串口）。
> - **顺带修掉一个潜伏的危险缺陷**：`_resolve_joint()` 原先从**全局**关节表解析名字。多板配置下，发给板 B 节点却写着板 A 关节名的目标会被解析成通道号、再驱动到板 B 的同号通道——**动的是另一块板上的另一个关节，而结果看起来是成功的**。现在只在本板关节表内解析，跨板关节名被拒（`error_code=121`）且消息指明归属哪块板。
>
> 为什么选"一板一进程"：隔离是**结构性的**，不靠代码自律——每块板有自己的进程、串口所有权线程、序号分配器与去重缓存、自己的 UNKNOWN 停机；一块板卡死、掉线或进入 UNKNOWN 不会妨碍另一块被轮询和被停机。而且每块板可以有**自己的策略**（板 B 只读、板 A 运动），节点级参数做不到这点。代价是**没有**跨板原子停机、跨板同步或聚合成功——这与 3.3 的要求一致，不是妥协。
>
> **已验证（fake，未接硬件）**：一份独立于 pytest 的验证脚本在目标主机上跑通 **30 项检查、0 失败**，覆盖：双板 YAML 加载与关节归属、板 B 的 M2 头部槽位保持未映射、重复端口与重复 UID 被拒而同地址 1 被允许、未指定/错误 `board_name` 均被拒、两节点服务名集合互不相交且无 `/robot/*` 泄漏、接口确为相对名、`motor_states.header.frame_id` 能区分板、跨板关节被拒且**两条总线均无任何帧**、板 B 运动时板 A 命令数为 0 而板 B 恰为 1、A 进入 UNKNOWN 后 B 不受影响且 A 自身运动被拒（109）而 STOP 仍成功、停机后 A 仍保持 UNKNOWN、序号计数各自独立、单板无 namespace 时名字仍为 `/robot/*` 且 `motor_states` 仍在 `/robot/motor_states`。
>
> 因此 3.1–3.3 中"实现必须满足"的条目**已实现并经验证**；3.4 的清单仍需补成常驻自动化 pytest（进行中）。**尚未做的是两块实板同时运行**（阶段 4）——fake 结果不能代替它，阶段 4 的判据与停止条件原样有效。

### 3.1 架构准入（实现必须满足）
1. 每块板一套独立的 `BoardSession` + `BoardWorker`：**独立串口所有权线程、独立序号分配器与去重缓存、独立 `unknown_command_state`**。一块板进入 UNKNOWN 不得影响另一块的调度（`board_session` 的这些状态已是实例级，实现时不得改成模块级/类级共享）。
2. 关节命令按 `(board, channel)` 路由。`config.channel_to_joint(board_name, channel)` 已存在，路由必须以 board 为第一维，不得只按 channel。
3. 启动**原子性**：任一块板身份校验失败（UID 不符 / `command_protocol != 2`），整个节点不得进入"半可用"状态对外提供运动服务；要么全部就绪，要么明确报告哪块板不可用并拒绝该板的所有写。
4. 补 `config.py` 的缺口：**两块板指向同一个 `port` 必须被拒绝**（当前不检查，见 `config.py:444-478`）。同理建议拒绝同一 `(transport, port, address)` 组合重复。
5. 接口命名必须带板标识，否则两板的服务会互相抢请求。可选方案：(a) 每板一个命名空间 `/board_a/robot/*`、`/board_b/robot/*`（需要把绝对名改成相对名或加前缀参数）；(b) 保留 `/robot/*` 但在每个 Request 里加 `board_name` 字段并在服务端校验。**当前接口是 (a)(b) 都没有**，所以两板同域会冲突（见 3.2）。

### 3.2 命名冲突准入测试（实现后必须通过）
- 同域启动两块板 → `ros2 node list` 能区分两个节点名；`ros2 service list` 中每个服务只属于一块板。
- 向板 A 的服务发请求，**板 B 的日志不得出现任何 TX**；反之亦然。
- `ros2 topic echo /robot/motor_states`（或新的带板标识的话题）能明确区分来源：现有 `MotorStates.header.frame_id` 已是 board_name、并带 `uid`，可作为过渡判据，但**服务/Action 没有板标识，必须靠 3.1 第 5 条解决**。
- 两板各发一条运动目标，互不阻塞（各自的 worker 与串口所有权线程独立）。

### 3.3 STOP_ALL 语义准入（必须显式实现并测试）
`robot_stop_all()` 只作用于收到该帧的那一块板（`robot.c:144-148`），两块板是两条独立串口上的两个独立帧。因此：
- **不得宣称跨板原子停机**。实现必须**逐板发送、逐板确认**，并把每块板的结果分开报告：`board_a: accepted/stop_confirmed`、`board_b: accepted/stop_confirmed`。
- 一块板确认失败或 UNKNOWN，**不得**因为有另一块板成功就报"整体已停"；总判据应为"全部板都 `stop_confirmed=true`"，否则报"部分停止，X 板未确认"。
- 某板失联时，另一板的 STOP 仍须发送并确认；失联板报 `unknown`，不得静默忽略。
- STOP 优先级：任一板的 STOP 都必须能插到该板队列最前（现有 `PRIORITY_STOP=0`），且一块板的 STOP 不得排在另一板的长任务之后（因为 worker 独立，天然满足，但要有测试证明）。

### 3.4 双 fake 板准入测试清单（实现后逐条写成自动化测试）
全部在 `transport=fake` 上做，**不接硬件**：
1. **UID 交换**：把两块 fake 板的期望 UID 互换 → 两块都必须身份校验失败，节点不得提供任何写服务。
2. **单板断线**：让 fake 板 B 的所有读失败 → B 报 `communication_ok=false` 并进入 UNKNOWN；**A 必须继续正常轮询**（`age_sec` 保持小、`last_sequence` 不变），且 A 的运动能力不受影响。
3. **配置隔离**：对 A 做 `apply_config`（改 rpm）→ B 的四路配置逐字段不变；反之亦然。
4. **故障隔离**：在 A 上注入 fault → B 的 `fault` 仍为 0，B 的运动不被拒；A 的 UNKNOWN 不阻塞 B 的 STOP。
5. **双板各运动一路**：A 的 ch2 与 B 的 ch2 同时相对运动 → 两条都完成，各自的 `sequence` 独立递增，各自 `persisted=true`。
6. **部分失败**：A 的运动成功、B 的运动被拒（例如 B 的通道未使能）→ 结果必须**分别**报告，**不得把 A 的成功当作整组成功**。
7. **序号独立**：两板的 `last_sequence` 互不影响；一块板的去重缓存不得吞掉另一块的命令。
8. **只读与 commissioning 独立**：A 只读、B 允许运动时，A 的一切写（含 STOP）被拒，B 正常；A 的 `commissioning` 设置不影响 B 的限位策略。

### 3.5 现有代码即可离线验证的隔离性（fake，不接硬件，可现在做）

```bash
cd /data/ros2/robot_motor_ros2_ws
export ROS_DOMAIN_ID=73
python3 - <<'PY' | tee /data/ros2/evidence/S3_two_fake_sessions.txt
# 只验证 BoardSession 层的两板隔离；节点层多板尚未实现，这里不启动任何 ROS 节点。
import time
from robot_motor_driver import protocol as proto
from robot_motor_driver.fake_board import make_fake_stack

a = make_fake_stack(clock=time.monotonic, read_only=False,
                    expected_uid="C23B378115B092083830364A")
b = make_fake_stack(clock=time.monotonic, read_only=False,
                    expected_uid="C23B37C715B092083830364A")
(_, boardA, _, sessA, clkA) = a
(_, boardB, _, sessB, clkB) = b
clkA.advance(1.0); clkB.advance(1.0)
for ch in boardA.channels: ch.last_edge = time.monotonic() - 2.0
for ch in boardB.channels: ch.last_edge = time.monotonic() - 2.0

print("uid A/B:", sessA.uid, sessB.uid, "distinct:", sessA.uid != sessB.uid)
oa = sessA.send_command(proto.Opcode.MOVE, 2, 100)
print("A move seq:", oa.sequence, "B last_seq untouched:", sessB.read_last_sequence()[0])
sessB.mark_unknown("simulated on B only")
print("A halted?", sessA.unknown_command_state, " B halted?", sessB.unknown_command_state)
try:
    sessB.send_command(proto.Opcode.MOVE, 2, 100); print("FAIL: B move allowed while halted")
except proto.ProtocolError as e: print("B move refused while halted:", str(e)[:60])
ob = sessB.send_command(proto.Opcode.STOP_ALL, 0, 0, allow_while_unknown=True)
print("B STOP_ALL while halted: accepted=%s unknown=%s still_halted=%s"
      % (ob.accepted, ob.unknown, sessB.unknown_command_state))
print("A unaffected: halted=%s" % sessA.unknown_command_state)
PY
```

预期：两 UID 不同；A 的 MOVE 不改变 B 的 `last_seq`；B 停机时 A 不受影响；B 在 UNKNOWN 下 MOVE 被拒但 STOP_ALL 可发且**不清** UNKNOWN。

### 通过判据
3.1–3.4 全部有对应自动化测试并通过，`colcon test` 全绿；3.5 输出与预期一致。**软件阶段未通过前不得进入阶段 4 的"双板同域"尝试**（阶段 4 用两个域是临时手段，不是整合验证）。

### 失败后停止条件
- 任一隔离测试失败（尤其 2/4/6）→ 停止，**不得进入实板双板测试**。串扰在 fake 上都测得出来，在实板上测就是拿机械冒险。
- 两板同域出现服务抢占或话题混淆 → 停止，先解决命名（3.1 第 5 条），不要用"小心一点操作"绕过。

### 记录内容
测试清单逐项的通过/失败、3.5 的完整输出、多板实现的 commit/哈希、`colcon test-result` 汇总。

---

## 阶段 4：双板同时只读联测

> **用今天就能做到的方式**：两个节点进程、两个域（72 / 74）、两条独立 by-path。这验证的是"两条串口同时轮询互不干扰"，**不是**多板节点整合（那要等阶段 3）。

### 前提
- 阶段 1 两块板分别只读通过，UID 与端口对应关系已确认并记录。
- 两板各自独立的 CH340 线，**没有并成一条总线**。
- 两个终端，各自 export 对应域号（72 / 74）。

### 命令

```bash
# 终端 1：板 A（域 72）
set +u; source /opt/ros/humble/setup.bash; source /data/ros2/robot_motor_ros2_ws/install/setup.bash; set -u
export ROS_DOMAIN_ID=72
OUT=/data/ros2/evidence/S4_dual_$(date +%Y%m%d_%H%M%S); mkdir -p "$OUT"
timeout 700 ros2 launch robot_motor_bringup bringup.launch.py \
  transport:=modbus_rtu allow_motion:=false maintenance:=false polling_hz:=5.0 \
  ros_domain_id:=72 > "$OUT/A_node.log" 2>&1 &
```

```bash
# 终端 2：板 B（域 74）
set +u; source /opt/ros/humble/setup.bash; source /data/ros2/robot_motor_ros2_ws/install/setup.bash; set -u
export ROS_DOMAIN_ID=74
OUT=$(ls -dt /data/ros2/evidence/S4_dual_* | head -1)
timeout 700 ros2 launch robot_motor_bringup bringup.launch.py \
  transport:=modbus_rtu allow_motion:=false maintenance:=false polling_hz:=5.0 \
  config_file:=/data/ros2/robot_motor_ros2_ws/src/robot_motor_bringup/config/robot_right_arm_UNCONFIRMED_GEARS.yaml \
  ros_domain_id:=74 > "$OUT/B_node.log" 2>&1 &
```

```bash
# 终端 3：并行采样 5 分钟（每 2 s 各一次），同时记录端口占用
OUT=$(ls -dt /data/ros2/evidence/S4_dual_* | head -1); cd "$OUT"
for i in $(seq 1 150); do
  echo "### $i $(date -Is)" >> dual_series.txt
  ( ROS_DOMAIN_ID=72 timeout 15 ros2 topic echo /robot/motor_states --once \
      | grep -E "^uid|^identity_ok|^communication_ok|^stale|^age_sec|^last_sequence|^dirty|^storage_error|^pending_save" \
      | sed 's/^/A /' >> dual_series.txt ) &
  ( ROS_DOMAIN_ID=74 timeout 15 ros2 topic echo /robot/motor_states --once \
      | grep -E "^uid|^identity_ok|^communication_ok|^stale|^age_sec|^last_sequence|^dirty|^storage_error|^pending_save" \
      | sed 's/^/B /' >> dual_series.txt ) &
  wait
  [ $((i % 25)) -eq 0 ] && { echo "--- holders @$i" >> dual_series.txt; fuser -v /dev/ttyUSB* >> dual_series.txt 2>&1; }
  sleep 2
done
python3 - <<'PY' | tee dual_stats.txt
import re, statistics
t = open('dual_series.txt').read()
for board in ('A', 'B'):
    ages = [float(x) for x in re.findall(r'^%s age_sec: ([0-9.eE+-]+)' % board, t, re.M)]
    stale = re.findall(r'^%s stale: (\w+)' % board, t, re.M)
    uids = set(re.findall(r'^%s uid: (\w+)' % board, t, re.M))
    seqs = [int(x) for x in re.findall(r'^%s last_sequence: (\d+)' % board, t, re.M)]
    ok = re.findall(r'^%s communication_ok: (\w+)' % board, t, re.M)
    print("%s: samples=%d uid=%s age[min/median/max]=%.3f/%.3f/%.3f stale_true=%d comm_ok_false=%d seq_unique=%s"
          % (board, len(ages), uids, min(ages), statistics.median(ages), max(ages),
             stale.count('true'), ok.count('False'), sorted(set(seqs))))
PY
```

```bash
# 4.4 断线隔离测试：只拔板 B 的 USB（CH340 端），绝不拔电机电源
#     在终端 3 的采样进行中执行，观察 A 是否受影响
OUT=$(ls -dt /data/ros2/evidence/S4_dual_* | head -1)
{ date -Is; echo "unplugging board B USB now"; } | tee -a "$OUT/disconnect.log"
#   —— 人工动作：拔板 B 的 CH340 USB 头（或从 hub 上拔下该适配器），保持板 B 电机电源不变 ——
sleep 60
{ date -Is; echo "replugging board B USB"; ls -l /dev/serial/by-path/; } | tee -a "$OUT/disconnect.log"
#   —— 人工动作：插回。注意插回后 by-path 不变、ttyUSB 编号可能变 ——
sleep 60
grep -cE "communication lost|reconnect|UNKNOWN" "$OUT/A_node.log" | tee "$OUT/A_loss_events.txt"
grep -cE "communication lost|reconnect|UNKNOWN" "$OUT/B_node.log" | tee "$OUT/B_loss_events.txt"
```

### 预期回读
- 两板的 `uid` 各自稳定且**不同**（A `C23B3781...`，B `C23B37C7...`）——**这是同地址 1 不串扰的直接证据**：若接线或地址冲突，会出现 UID 抖动、CRC 错误或身份校验失败。
- `age_sec`：5 Hz 轮询下中位数应在 0.1～0.3 s 量级，最大值 < 1.0；`stale` 恒为 false。
- `communication_ok` 恒为 true；`last_sequence` 全程**唯一值**（只读不推进序号）。
- 端口占用：每次抽查都显示 A 的 ttyUSB 由 A 的节点 PID 持有、B 的由 B 的持有，**互不重叠**；门锁口的持有者始终是 `gateway.kafka_server`（或原持有者），未变化。
- 拔线后：B 的日志出现 `communication lost` / `reconnect` / UNKNOWN；**A 的日志不得出现这些**，A 的 `age_sec` 与 `last_sequence` 保持正常。
- 插回后：B 重新识别成功（UID 相符），但 **B 必须停在 UNKNOWN**，不自动续跑任何旧请求（现有行为：重连后 `no motion was resumed, state is UNKNOWN until an operator acknowledges it`）。

### 通过判据
1. 5 分钟内两板各自 `communication_ok=true` 比例 100%，`stale=true` 次数为 0。
2. 两板 `uid` 全程唯一且与预期一致，**没有一次交叉**（交叉即串扰）。
3. 两板 `last_sequence` 全程不变，且两个 `tx_count` 均为 0。
4. 拔 B 的 USB 期间，A 的所有指标不受影响；B 恢复后停在 UNKNOWN 且未自动重发任何命令（对比 `last_sequence` 与 B 的 TX 计数）。
5. 两个节点进程各自只持有一条串口。

### 失败后停止条件
- 两板 UID 出现交叉、或任一板报 CRC/身份错误 → **停止**。可能是接线并了、hub 异常、或两个进程抢同一端口。在查清前不得做任何写操作。
- 拔 B 时 A 也出现通信丢失 → **停止**。说明两条链路有共享环节（同一 hub 的供电/带宽、同一 USB 控制器），双板同时运行的前提不成立，需要换 hub/换口再测。
- 拔线后 B **自动恢复运动或自动重发命令** → 停止，这是严重缺陷（失联板不得自动续跑），必须先修驱动。
- `age_sec` 长期接近或超过 1.0 → 停止。轮询跟不上说明总线/主机负载有问题，此时任何写操作都更容易变成 UNKNOWN。
- 门锁口的持有者发生变化，或门锁口出现在任一节点的日志里 → 立即停止全部测试。

> **不得用拔电机电源代替拔通信线**：那测的是"电机掉电后板子怎么反应"，不是通信断线，而且会让 FG 计数与位置语义混入掉电因素，无法解释。

### 记录内容
`dual_series.txt`、`dual_stats.txt`、两个 `*_node.log`、`disconnect.log`（含拔/插的精确时间与拔的是哪一头）、拔插前后的 `ls -l /dev/serial/by-path/`、两板 TX 计数、端口占用抽查结果。

---

## 阶段 5：逐板配置、自检及单关节实物试转

### 前提（**全部满足**才可进入；任一不满足即停）
1. 该板 `storage_error=false`、`pending_save=false`、`dirty=false`（板 A 需先按阶段 2A 处理并复测）。
2. 该板待测通道 `fault=0`；若曾清故障，须有"清后静置 60 s 不复发"的记录。
3. 实际接线、减速比、role、`inverted` 初值、机械正方向、限位——阶段 2B 清单**逐项书面确认**（板 B 尤其；板 A 的限位也仍是 `null`）。
4. 机械空间：目标关节全行程无干涉、无人员障碍；**能立即断电**且操作者知道怎么断。明确软件 STOP **不是**安全功能（默认构建 `BUTTON_ESTOP_ENABLE=0`，板上急停输入恒为 0）。
5. 两人到场：一人盯机构，一人操作终端。
6. 已确认该板的测试速度（见 5.1），**不依赖 Action 的 `motor_rpm` 调速**。
7. 逐项书面授权，一次授权只覆盖一项：写 Flash（5.1）/ 清故障（5.2）/ 设零（5.3）/ 运动（5.4+）/ 自检（5.6）。

### 5.1 配置事务：把测试速度写进板子（暂存→回读→提交→回读→重启复核）

**为什么必须先做这一步**：板上四路 rpm=8000。折算到输出轴（`输出转速 = rpm / gear`，`°/s = 输出转速 × 6`）：

| 通道 | gear | 8000 rpm 时输出速度 | 5° 用时 | 400 rpm 时输出速度 | 5° 用时 |
|---|---|---|---|---|---|
| M1/M2 | 516 | 15.5 rpm ≈ 93 °/s | 54 ms | 0.78 rpm ≈ 4.7 °/s | 1.07 s |
| M3 | 290 | 27.6 rpm ≈ 165 °/s | 30 ms | 1.38 rpm ≈ 8.3 °/s | 0.60 s |
| M4 | 82 | 97.6 rpm ≈ 585 °/s | **8.5 ms** | 4.88 rpm ≈ 29 °/s | 0.17 s |

8000 rpm 下 M4 的 5° 只要 8.5 ms，人根本来不及观察或干预。**先在板上保存 400 rpm 再读回**，是唯一能让首次实物试转可控的办法。`MoveJoint.motor_rpm` **不是**调速字段：它只是对板上已存速度的断言，填 400 而板上是 8000 会被拒（`error_code=111`），填 0 表示"用板上的值"。

**写 Flash 的影响（授权前必须念给操作者）**：CONFIG 会擦写配置扇区；要求整板静止 500 ms，否则返回 `BUSY(2)`（`robot.c:387`）；成功后 `dirty/pending_save/storage_error` 全清（`robot.c:400`）；`pwm_hz` 立即生效，而 `protocol/address/node` **要断电重启才生效**（`robot.c:406`），所以本步不改这三项；**只改 rpm 不会清零点、也不会清自检记录**（`robot.c:394-396` 只在 `gear/role/inverted`（零点）与 `enabled/gear/role/inverted`（自检记录）变化时才清）。反之，**改 gear/role/inverted 会使该路 `position_valid` 变 false**，必须重新设零。

```bash
export ROS_DOMAIN_ID=72      # 板 B 用 74，并换 config_file
OUT=/data/ros2/evidence/S5_boardA_$(date +%Y%m%d_%H%M%S); mkdir -p "$OUT"; cd "$OUT"
timeout 400 ros2 launch robot_motor_bringup bringup.launch.py \
  transport:=modbus_rtu maintenance:=true allow_motion:=false ros_domain_id:=72 > node.log 2>&1 &
sleep 20
grep -E "ready: transport=|SIMULATED" node.log | tee ident.txt   # 必须 modbus_rtu，且无 SIMULATED

# (a) 只暂存、不提交：人工核对整块暂存区
timeout 60 ros2 service call /robot/apply_config robot_motor_interfaces/srv/ApplyConfig \
  "{channels: [{channel: 2, enabled: true, role: 4, gear_ratio: 290, inverted: 0, rpm: 400}], commit: false}" \
  | tee cfg_stageonly.txt
# (b) 核对无误后提交（role/gear/inverted/enabled 必须填阶段 1 读到的现值，填错就会清零点）
timeout 60 ros2 service call /robot/apply_config robot_motor_interfaces/srv/ApplyConfig \
  "{channels: [{channel: 2, enabled: true, role: 4, gear_ratio: 290, inverted: 0, rpm: 400}], commit: true}" \
  | tee cfg_commit.txt
timeout 40 ros2 topic echo /robot/motor_states --once | tee cfg_states.yaml
# (c) 重启复核：断电→上电→只读重跑阶段 1，确认 rpm=400 仍在、valid 未丢
```

判据：(a) `accepted=true`、`changed=true`、消息含 `staged only`、生效区未变；(b) `accepted=true`、`changed=true`、`result_code=0`、`persisted=true`，`applied` 中该路 `rpm=400` 且其余三路逐字段不变，`storage_error/pending_save/dirty` 全 false，该路 `position_valid` **不变**；(c) 重启后 `configured_rpm` 仍为 400，`valid` 与重启前一致（`dirty=false` 时零点应保留）。

停止条件：`result_code=4`（STORAGE）或 `persisted=false` → **保存失败可复现**，停止全部后续步骤，回阶段 2A 的 (a)/(b)/(c) 定位，不得继续试转。`result_code=2`（BUSY）→ 整板未静止，等 5 s 重试一次，仍 BUSY 则停并查 FG 活动。`changed=false` → 你填的值与板上相同，说明阶段 1 的记录有误，回阶段 1 重读。`result_code=108`（暂存回读不一致）→ 停止，CONFIG 未发送，查暂存路径。

> 一次只改**一路**的 rpm，跑完整流程再改下一路。多路一起改会放大出错面，且失败时难以归因。

### 5.2 清故障（授权写；逐路，且必须做复发观测）

```bash
timeout 60 ros2 service call /robot/clear_fault robot_motor_interfaces/srv/ClearFault \
  "{joint_name: 'head_joint'}" | tee clear_m2.txt        # head_joint = 协议 ch1 = M2
sleep 2; timeout 40 ros2 topic echo /robot/motor_states --once | tee clear_m2_states.yaml
sleep 60; timeout 40 ros2 topic echo /robot/motor_states --once | tee clear_m2_after60s.yaml
```

判据：`accepted=true`、`fault_after=0`；消息会提示 CLEAR_FAULT **不恢复** `position_valid`（这是正常的，零点靠 5.3）。60 s 后 `fault` 仍为 0。

两条必须知道的固件语义（`robot.c:423-427`）：**CLEAR_FAULT 是 RAM-only**，它只清 `fault`、`unexpected_fg` 累计计数和 `stop_fault_mask` 位，不写 Flash、不动 `valid`，所以**断电重启后故障会回来**（这不是"没清干净"，是设计如此）；它要求**整板静止 500 ms**，若正在返回 `BUSY(2)`，那本身就是"有实时 FG 活动"的证据，应记入 2A 的判读表而不是重试。

停止条件：60 s 内 fault **复发为 4** → 不是手转造成，是实时 FG 边沿源（干扰/接线/编码器）→ **停止本阶段**，转硬件排查。不得用"再清一次"绕过。`fault_after` 仍非 0，或返回 `BUSY(2)` 且 2A.1 已显示有漂移 → 同样停止。

### 5.3 设零（授权写；可选）

```bash
timeout 60 ros2 service call /robot/set_zero robot_motor_interfaces/srv/SetZero \
  "{joint_name: 'left_forearm_joint'}" | tee zero_m3.txt
```

判据：`accepted=true`、`position_valid=true`、`persisted=true`；随后只读复核该路 `state=1(IDLE)`、`fg_count=0`、`angle_deci_deg=0`。

固件语义（`robot.c:429-438`）：ZERO 要求整板静止且该路 `gear!=0`（否则返回 `UNREADY(3)`——**板 B 的 ch2 历史 gear=0，在填对减速比之前对它设零必然失败**）；它写 Flash，成功时把该路 `position=0, valid=1, fault=0` 并一并清掉 `dirty/pending_save/storage_error`。所以 ZERO 也能当作 Flash 健康测试，但代价是**用当前姿态覆盖掉原有零点**，因此 5.1 的"只改 rpm"仍是首选（它同样清 `storage_error`，却不消耗任何零点，见 `robot.c:394-396,400`）。ROS 侧**没有** `/robot/save` 服务，能触发一次成功 `persist()` 从而清掉 `storage_error` 的只有 CONFIG（`apply_config`）与 ZERO（`set_zero`）；MOVE 的 pre-write 也会 `persist`，但 MOVE 本身已被 `storage_error` 挡住（`robot.c:449`），构成死锁——这正是必须先做 5.1 的原因。

注意：若机械参考姿态尚未确定，**跳过本步**，改走"无零点相对运动"（5.4 加 `allow_unreferenced_relative:=true`，全程禁用绝对运动）。已经有 `valid=1` 的通道（板 A 的 M3）**不要为了流程完整而重新设零**——那会用当前姿态覆盖掉现有参考。设零会写 Flash，`dirty` 期间断电将导致四路零点全丢（`robot.c:75`）。

### 5.4 首次单关节小角度试转（一次只动一个关节）

```bash
kill %1 2>/dev/null; wait 2>/dev/null; sleep 2
timeout 400 ros2 launch robot_motor_bringup bringup.launch.py \
  transport:=modbus_rtu allow_motion:=true allow_unreferenced_relative:=false \
  commissioning:=true max_commissioning_step_deg:=5.0 ros_domain_id:=72 > move_node.log 2>&1 &
sleep 20
grep -E "ready: transport=|commissioning mode is ON" move_node.log | tee move_ident.txt
# 2° = 0.0349065850 rad；motor_rpm 必须填 0（用板上的 400）或填 400（与板上一致）
timeout 60 ros2 action send_goal /robot/move_joint robot_motor_interfaces/action/MoveJoint \
  "{joint_name: 'left_forearm_joint', mode: 0, angle_rad: 0.0349065850, motor_rpm: 0, timeout_sec: 30.0}" \
  --feedback | tee move_plus2.txt
timeout 40 ros2 topic echo /robot/motor_states --once | tee move_plus2_states.yaml
# 人工：用独立方式（量角器/刻度盘/相机）测量实际转角，并目视确认方向
timeout 60 ros2 action send_goal /robot/move_joint robot_motor_interfaces/action/MoveJoint \
  "{joint_name: 'left_forearm_joint', mode: 0, angle_rad: -0.0349065850, motor_rpm: 0, timeout_sec: 30.0}" \
  --feedback | tee move_minus2.txt
timeout 40 ros2 topic echo /robot/motor_states --once | tee move_minus2_states.yaml
```

参数依据（**代码推断**，`robot.c:450,463`）：`timeout_sec` 必须大于板上自己的超时 `10000 + distance*gear*60000/(3600*rpm)*4` ms（`distance` 用 0.1° 单位）。2° → `distance=20`，gear290、400 rpm：`10000 + 20*290*60000/(3600*400)*4 = 10000 + 967 ≈ 11.0 s`；gear82 同角度约 10.3 s；gear516 约 12.3 s。**注意公式有 10 s 下限**，所以小角度移动的板上超时几乎恒为 10～13 s，`timeout_sec` 一律取 30 s。若 `timeout_sec` 给得比板上超时短，驱动会先判超时转 UNKNOWN，而板子其实还在正常走——这会白白制造一次需要人工确认的停机。

`commissioning:=true` 是必需的，因为 `limits_rad=null`；它**不提供软限位**，只提供 `max_commissioning_step_deg=5.0` 的单次步长上限。日志会打印 `commissioning mode is ON`，`/robot/motor_states` 的 `commissioning=true`、`soft_limits_enforced=false`，诊断里也会有"无软限位保护"的条目。

判据（正反转各一次，全部满足才算通过）：
- `error_code=0`、`motion_completed=true`、`persisted=true`；消息含 `no soft-limit check was possible`（如实声明未做限位核对，不是错误）。
- 该路 `state=5(DONE)`、`fault=0`、`brake=true`、`duty_permille=0`、`measured_rpm=0`（到位后）。
- `angle_deci_deg` 与目标 20 的差能被 FG 量化解释：gear290 每步 `3600/(290×6)=2.07` 个 0.1°，gear516 每步 1.16，gear82 每步 7.32。
- **外部实测角度**与 `angle_deci_deg` 一致（在量化误差内）。这是唯一能验证减速比与方向的手段，**不能省**。
- 反向后回到 `angle_deci_deg≈0`，无新 fault。
- `dirty/storage_error/pending_save` 全 false。
- `last_sequence` 恰好增加"发出的命令条数"。

停止条件（任一即停，且**不得重试第二次**，先做 5.5 的停机确认）：`error_code=2`（BUSY，整板未静止 500 ms）、`3`（UNREADY，未使能/gear=0/有故障）、`4`（STORAGE，保存失败）、`102/103`（UNKNOWN/超时）、`111`（rpm 断言不符，说明 5.1 没做或配置又被改）、`112`（忘了开 commissioning）、`105`（超出步长上限或限位）。机构出现异响、卡滞、发热、明显偏差 → 立即断电。`measured_rpm` 明显偏离 400（例如 >1000）→ 停止该路，转测速排查（左手腕历史上有过设定 1500 读到约 5737/5847 的未解问题）。

### 5.5 停止与取消路径（必须先于任何连续动作验证）

```bash
timeout 90 ros2 action send_goal /robot/move_joint robot_motor_interfaces/action/MoveJoint \
  "{joint_name: 'left_forearm_joint', mode: 0, angle_rad: 0.0698, motor_rpm: 0, timeout_sec: 30.0}" \
  --feedback > stop_long.log 2>&1 &
sleep 1
timeout 40 ros2 service call /robot/stop robot_motor_interfaces/srv/Stop \
  "{joint_name: 'left_forearm_joint'}" | tee stop_during_motion.txt
wait; tail -20 stop_long.log | tee stop_goal.txt
# 超时→UNKNOWN→仍可停机→需人工确认恢复
timeout 60 ros2 action send_goal /robot/move_joint robot_motor_interfaces/action/MoveJoint \
  "{joint_name: 'left_forearm_joint', mode: 0, angle_rad: 0.0698, motor_rpm: 0, timeout_sec: 2.0}" \
  | tee timeout_unknown.txt
timeout 40 ros2 topic echo /robot/motor_states --once --field unknown_command_state
timeout 40 ros2 service call /robot/stop robot_motor_interfaces/srv/Stop "{}" | tee stop_while_halted.txt
timeout 40 ros2 action send_goal /robot/move_joint robot_motor_interfaces/action/MoveJoint \
  "{joint_name: 'left_forearm_joint', mode: 0, angle_rad: 0.01, motor_rpm: 0, timeout_sec: 20.0}" \
  | tee move_while_halted.txt
timeout 40 ros2 service call /robot/acknowledge_unknown std_srvs/srv/Trigger "{}" | tee ack.txt
```

判据：`stop_during_motion.txt` 为 `accepted=true` **且 `stop_confirmed=true`**；`timeout_unknown.txt` 为 `error_code=103`、`unknown=true`、消息含 `No new sequence was issued`；`unknown_command_state=true`；**`stop_while_halted.txt` 仍为 `accepted=true` 且 `stop_confirmed=true`**（UNKNOWN 停机期间只有 STOP/STOP_ALL 放行）；`move_while_halted.txt` 被拒 `error_code=109`；`ack.txt` 之后才恢复；全程 `last_sequence` 只按实际发出的命令数递增（**没有换新序号重发**）。

停止条件：`stop_confirmed=false` 或 `unknown=true` → 按**未停止**处理，立即断电，记录当时全部快照。停机后 `unknown_command_state` 自动变 false → 这是缺陷，停止并报告（停机不等于确认）。

### 5.6 单路自检（最后做，且先单路不整板）

```bash
timeout 120 ros2 action send_goal /robot/selftest robot_motor_interfaces/action/RunSelfTest \
  "{joint_name: 'left_forearm_joint', scope: 0, direction: 0, timeout_sec: 60.0}" --feedback \
  | tee selftest_single_m3.txt
```

判据：`completed=true`、`channels[]` 长度为 1、`state=4(PASSED)`、`outbound_pulses>=3`、`residual_pulses` 记录在案；消息含"只证明抓到反馈脉冲，**不证明**接线、方向或准确回原位"。

停止条件：`state=5(FAILED)` → 记录 `reason`（1=no_pulses、2=too_few、3=stop_timeout、…），停止该路，**不要**重复自检试图"刷过"。`state=6(CANCELLED)` → 查是谁停的。整板自检（`scope: 1`）**只在四路单路都通过之后**才考虑，且要按 `channels[]` 逐路核对；`state=0 且 executed=true` 表示"被选中但从未通电"，不是通过。板 B **不得**用整板自检做准入（空接通道必然失败，见 2B）。

### 5.7 逐路顺序
板 A：**M3(ch2, gear290, 已有零点) → M1(ch0, gear516) → M4(ch3, gear82) → M2(ch1, head)**。head 放最后：它的"命令 M1 却动了头"的角色/物理对应关系尚未解决，且靠近人体。M4 的 gear82 即使 400 rpm 也有 29 °/s，首次角度建议再减半到 1°。
板 B：先按 2B 确认的**实际接入通道**排序，从减速比最大（最慢）的那一路开始。
每路都要独立走完 5.1→5.6，独立授权、独立记录。

### 记录内容
每条命令原文与完整回包、`last_sequence` 前后值、逐通道 `state/fault/position_valid/brake/duty_permille/measured_rpm/configured_rpm/gear_ratio/role/enabled`、`angle_deci_deg/fg_count/target_deci_deg`（**原始值**；`angle_rad` 在 `position_valid=false` 时被驱动置 0，不能当姿态）、三个保存标志、**外部实测角度与实测方向**（含观察者姓名）、TX 计数、异常与停止条件命中情况。

---

## 阶段 6：双板联合运动与故障隔离

### 前提
- 阶段 3（多板软件整合）**全部通过**。若尚未实现多板，本阶段**不能执行**：两个独立进程 + 两个域无法构成"联合运动"，也无法验证跨板停机语义。用两个域分别运动**不得**记作双板联合测试。
- 两块板各自的阶段 5 全通道通过：单关节往返、停止/取消/超时、单路自检都有实测记录。
- 两板各自 `storage_error=false`、`fault=0`、测试速度已保存并读回（400 rpm 或经确认的值）。
- 机械空间同时满足两板：两条臂各自的全行程无干涉、**彼此之间**也无干涉，两人都能看到。
- 明确并已告知所有人：**首版不保证两板同步轨迹，也不保证同时到位**。两块板是两条串口上的两个独立 worker，没有跨板时钟同步、没有插补、没有协调停止。

### 命令（多板实现完成后）

```bash
export ROS_DOMAIN_ID=75      # 双板联合用独立域，避免与单板测试的残留节点混淆
OUT=/data/ros2/evidence/S6_dual_motion_$(date +%Y%m%d_%H%M%S); mkdir -p "$OUT"; cd "$OUT"
# 启动方式取决于阶段 3 的实现：单节点多板，或每板一节点+带板标识的接口名。
# 无论哪种，先确认能分别寻址两块板：
ros2 node list | tee nodes.txt
ros2 service list | tee services.txt
ros2 action list | tee actions.txt
timeout 40 ros2 topic echo /robot/motor_states --once > states_both.yaml   # 或分别 echo 两板话题
grep -cE "uid: C23B3781" states_both.yaml; grep -cE "uid: C23B37C7" states_both.yaml
```

```bash
# 6.1 双板各一路、低速小角度、同时下发（用带板标识的目标字段/命名空间，按阶段 3 的实现填写）
( timeout 90 ros2 action send_goal <板A的move_joint> robot_motor_interfaces/action/MoveJoint \
    "{joint_name: 'left_forearm_joint', mode: 0, angle_rad: 0.0349065850, motor_rpm: 0, timeout_sec: 30.0}" \
    --feedback > A_move.log 2>&1 ; date -Is >> A_move.log ) &
( timeout 90 ros2 action send_goal <板B的move_joint> robot_motor_interfaces/action/MoveJoint \
    "{joint_name: '<板B已确认的关节>', mode: 0, angle_rad: 0.0349065850, motor_rpm: 0, timeout_sec: 30.0}" \
    --feedback > B_move.log 2>&1 ; date -Is >> B_move.log ) &
wait
timeout 40 ros2 topic echo /robot/motor_states --once > states_after.yaml
```

```bash
# 6.2 跨板停机：两板都在动时发一次"全停"，要求逐板回执
( timeout 90 ros2 action send_goal <板A的move_joint> ... angle_rad: 0.0698 ... > A_long.log 2>&1 ) &
( timeout 90 ros2 action send_goal <板B的move_joint> ... angle_rad: 0.0698 ... > B_long.log 2>&1 ) &
sleep 1
timeout 40 ros2 service call <全停服务> ... | tee stop_all_both.txt
wait
```

```bash
# 6.3 故障隔离：只让板 B 失联（拔 B 的 USB，不拔电机电源），板 A 必须继续可控
#     —— 人工动作：拔板 B 的 CH340 USB 头 ——
sleep 30
timeout 40 ros2 service call <板A的stop> ... "{joint_name: 'left_forearm_joint'}" | tee A_stop_during_B_loss.txt
timeout 60 ros2 action send_goal <板A的move_joint> ... angle_rad: 0.0349 ... | tee A_move_during_B_loss.txt
#     —— 人工动作：插回板 B ——
sleep 30
timeout 40 ros2 topic echo /robot/motor_states --once > states_after_replug.yaml
```

### 预期回读
- 6.1：两个 Action 各自 `error_code=0`、`motion_completed=true`、`persisted=true`；两板各自的 `last_sequence` 独立递增且互不影响；两路 `angle_deci_deg` 都到位；**开始/完成时间不同是正常的**（无同步保证）。
- 6.2：**逐板**报告 `accepted` 与 `stop_confirmed`；两板都 `stop_confirmed=true` 才算"已停"。任何一板未确认 → 整体结论必须是"部分停止"，并按未停止处理。
- 6.3：B 失联期间 A 的 STOP 仍 `accepted=true`、`stop_confirmed=true`，A 的新运动目标能正常执行；B 报 `communication_ok=false` 且进入 UNKNOWN；B 插回后**停在 UNKNOWN、不自动续跑**任何旧请求，需人工 `acknowledge_unknown`。

### 通过判据
1. 双板各一路运动都独立完成，结果**分别**可追溯（各自的 UID/序号/角度/时间）。
2. 跨板停机给出**逐板**回执；总判据是"全部板 `stop_confirmed=true`"。
3. **部分失败不得被报成整组成功**：B 失败而 A 成功时，聚合结论必须明确写"A 成功 / B 失败"，任何"整体成功"字段都必须为 false。
4. 单板失联不影响另一板的只读轮询、STOP 与运动能力；失联板恢复后停在 UNKNOWN。
5. 全程两板 UID 无交叉、无 CRC 错误、门锁端口占用者未变。

### 失败后停止条件
- 跨板停机出现"整体成功但某板未确认" → **停止**，这是最危险的缺陷（会把没停的机构报成已停）。
- 一板失联导致另一板也报错/停摆 → 停止，说明隔离没做好，回阶段 3.4 第 2/4 条。
- B 插回后自动续跑旧请求 → 停止，严重缺陷。
- 两板序号互相影响、或一板的命令出现在另一板日志里 → 停止，路由串扰。
- 任一机械异常（异响、卡滞、两臂接近干涉）→ 立即断电，停止本阶段。

### 记录内容
两板各自的：UID、by-path、关节与物理通道、命令原文、序号、回包、开始/完成/停止时间戳（各自独立记录，**不要**只记一个"整组完成时间"）、逐通道状态与故障、保存标志、外部实测角度与方向、逐板 STOP 回执、失联与恢复的精确时刻、聚合结论是如何得出的（哪个字段说明部分失败）。

---

## 验收记录表

每条命令一行。**"证据等级"列必须填**：`已实测` / `历史记录` / `代码推断` / `待确认`。空白或写"应该通过"一律视为未验证。

### 表 1：命令级记录

| # | 时间(ISO) | 阶段 | 板名 | UID | by-path → ttyUSB | 固件/ROS 版本 | 域号 | 关节名 | 物理通道 | 实际 gear | 板上 rpm | 命令原文 | last_seq 前→后 | 回包关键字段 | state | fault | position_valid | dirty/storage_error/pending_save | TX 计数 | 实测方向 | 外部实测角度 | 证据等级 | 通过? | 备注/停止条件 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|

填写要求：
- **固件/ROS 版本**：固件填 `S0_src_hashes.txt` 里的 BSP 哈希或板上读到的 schema/协议版本（`command_protocol`）；ROS 填 `ros2 pkg prefix` + 工作区源码哈希（阶段 0.2）。
- **回包关键字段**：至少 `accepted`、`result_code`/`error_code`、`completed`/`motion_completed`、`persisted`、`stop_confirmed`、`unknown`、`message` 摘要。
- **实测方向 / 外部实测角度**：只能由人填写，注明观察者和量具。`measured_rpm` 是无符号幅值、`direction` 只是 DIR 电平，**都不能当作实测转向**。
- **TX 计数**：只读阶段必须为 0；写阶段必须等于发出的命令条数。
- **通过?**：只填 `通过` / `失败` / `未执行`。`未执行` 必须写原因。

### 表 2：板级准入清单（每块板各一份，全部为"是"才可进入下一阶段）

| 项 | 板 A | 板 B | 证据等级 | 依据（文件/行号或记录编号） |
|---|---|---|---|---|
| UID 与 by-path 对应关系已当场确认 | | | | |
| `command_protocol=2`、`identity_ok=true` | | | | |
| `storage_error=false`、`pending_save=false`、`dirty=false` | | | | |
| 待测通道 `fault=0`，且清故障后静置 60 s 不复发 | | | | |
| 待测通道 `enabled=1`、`gear` 为实测确认值 | | | | |
| 通道→关节映射由操作者书面确认 | | | | |
| `inverted` 初值与实测方向一致 | | | | |
| 机械限位已测量（或明确记录 `limits_rad=null` + 只用 commissioning） | | | | |
| 测试速度已写入板子并读回（不靠 Action 调速） | | | | |
| 该串口只被本节点持有；门锁端口未被触碰 | | | | |
| 断电手段就绪，两人到场 | | | | |
| 只读阶段 TX 计数为 0 | | | | |

### 表 3：待确认项登记（软件无法推断，必须人来给）

| 项 | 板/通道 | 当前值 | 需要谁确认 | 确认方式 | 确认人 | 日期 | 状态 |
|---|---|---|---|---|---|---|---|
| 右臂通道→关节映射 | B / ch0-3 | 未确认（板上 ch3 role=2 是左臂残留） | 操作者 | 接线对照 + 断电轻转 | | | 待确认 |
| 右臂各通道减速比 | B / ch0,2,3 | YAML 推定 516/290/82；板上历史 516/516/0/516 | 操作者 | 铭牌/拆检/图纸 | | | 待确认 |
| 右臂 role 值 | B / ch0,2,3 | 预期 3/5/7，取决于映射 | 操作者 | 映射确定后填 | | | 待确认 |
| 右臂 M2 槽位是否有电机 | B / ch1 | 预期无（右臂无头部） | 操作者 | 目视/接线 | | | 待确认 |
| 两板 `inverted` | A,B / 全部 | 0（未验证） | 操作者 | 5.4 小角度通电观察 | | | 待确认 |
| 两板机械限位 | A,B / 全部 | `null` | 操作者 | 断电走到极限 + 独立量角 | | | 待确认 |
| 真实机械正方向 | A,B / 全部 | 未知 | 操作者 | 5.4 目视 + 外部量角 | | | 待确认 |
| 板 A `storage_error` 成因 | A | 只知为 true | 板上测量/维修 | 授权写入观察，或调试器读槽位 | | | 待确认 |
| 板 A M2/M4 fault 4 成因 | A | 只知 fault=4 | 板上测量 | 2A.1 漂移观测 + 电气检查 | | | 待确认 |
| M1/head 角色与物理对应关系 | A / ch0,ch1 | 寄存器 M1=left_upper、M2=head，但历史记录称"命令 M1 动了头" | 操作者 | 断电轻转逐路指认 | | | 待确认 |

### 表 4：未验证项声明（每轮结束时必须填写，不得留空）

| 未验证项 | 为什么没验证 | 需要什么才能验证 |
|---|---|---|
| 双板实物联合运动 | 多板节点尚未实现（`_select_board()` 只接受一块板） | 完成阶段 3 的实现与准入测试 |
| 板 B 任何运动 | 减速比/映射/方向/限位全部待确认 | 完成表 3 中板 B 各项 |
| 板 A 运动与自检 | `storage_error=true`，MOVE 与 SELFTEST 都会被固件拒（`robot.c:449,168`） | 完成阶段 2A |
| 真实机械方向与外部角度 | 本轮未运动 | 阶段 5.4 |
| Flash 保存失败根因 | Modbus 不暴露原因码（`robot.c:506`） | 阶段 2A 的 (a)/(b)/(c) |

---

## 全程禁止事项

1. 禁止在 `storage_error=true` 或 `pending_save=true` 时安排该板的运动、自检或配置提交（固件必返 `4`，只会污染记录并可能再次锁存）。
2. 禁止用 `MoveJoint.motor_rpm` 当调速字段。它是断言：填 400 而板上是 8000 → `error_code=111` 且一帧不发。要改速度只能走 5.1 的 CONFIG（写 Flash，需授权）。
3. 禁止把 fake 结果当实板证据。**fake 后端会采用 YAML 里的实板 UID**（`motor_node.py:238`），所以 UID 相符不证明是实板；判据是 `transport` 参数 + 日志 `SIMULATED`/`ready: transport=` + 串口占用。
4. 禁止把两个 CH340 的 TTL 线并成一条 RS-485 总线。两板地址都是 1，并线必然冲突。
5. 禁止在域 0 上运行任何东西；禁止"只 export 域号"而不给 launch 传 `ros_domain_id:=`（launch 会用默认值 71 覆盖节点进程的域）。
6. 禁止触碰门锁网关串口（历史为 `platform-xhci-hcd.5.auto-usb-0:1.2:1.0-port0` → `ttyUSB1`，由 `gateway.kafka_server` 持有）。每次开工都要重新确认它是谁、在哪。
7. 禁止按 `ttyUSBn` 编号认设备。同一物理口的编号历史上有过 ttyUSB2→1→0 漂移，一律用 by-path 并核对 UID。
8. 禁止在 UID 不符、端口交换、链路 stale、或发现有其他主站在写板时执行任何写操作。
9. 禁止把"清故障成功"当成故障已排除。清故障只是清 RAM 位（`robot.c` 中 `CLEAR_FAULT` 不动 `valid`），复发说明是实时干扰源。
10. 禁止把 `SELFTEST` 的 PASSED 解读为接线、方向或回位正确。它只证明抓到了 ≥3 个反馈脉冲。
11. 禁止把批量自检中 `state=0(untested) 且 executed=true` 读成"通过"。那是"被选中但从未通电"（`robot.c:102-106`）。
12. 禁止用拔电机电源代替拔通信线做断线测试。
13. 禁止在 `dirty=true` 时断电重启：四路零点会全丢（`robot.c:75`）。
14. 禁止把"单板失败 + 单板成功"报成整组成功（阶段 6 判据 3）。
15. 禁止烧录固件、禁止修改固件命令编号/寄存器布局/Flash 格式。本文全程不涉及这些。
16. 禁止虚构未执行的结果。任何一行记录都必须能追溯到落盘的原始回包。

## 授权门（每个门单独授权，一次授权只覆盖一项）

| 门 | 内容 | 是否写 Flash | 前置 |
|---|---|---|---|
| G0 | 阶段 0 版本与环境 | 否 | — |
| G1 | 阶段 1 两板只读 | 否 | G0 |
| G2 | 阶段 2A.1 长时序只读观测 | 否 | G1 |
| G3 | 板 A 断电重启（会销毁 storage_error 现场证据） | 否（但清 RAM） | G2 记录归档 |
| G4 | 阶段 3 多板实现与 fake 准入测试 | 否 | G1 |
| G5 | 阶段 4 双板同时只读 + 拔通信线 | 否 | G1（G4 可选） |
| G6 | 5.1 CONFIG 改 rpm | **是** | G2/G3 完成，save 标志正常 |
| G7 | 5.2 CLEAR_FAULT | 否（清 RAM 位） | G6 |
| G8 | 5.3 SET_ZERO | **是** | G7，且机械参考姿态已确定 |
| G9 | 5.4/5.5 单关节小角度运动与停机验证 | **是**（MOVE 有 pre-write） | G6–G8，两人到场 |
| G10 | 5.6 单路自检 | **是** | G9 |
| G11 | 阶段 6 双板联合运动 | **是** | G4 + 两板各自的 G10 |
