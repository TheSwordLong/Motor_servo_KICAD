# ROS2 电机 Web 管理界面

无需登录，在局域网访问 `http://192.168.1.113:8443/`。默认读取状态，不自动启动电机驱动节点、不试转、不设零、不释放刹车、不写板上配置。

用户于2026-10-12明确取消登录要求。因此没有账号、密码或登录API；匿名会话与CSRF令牌只用于同源检查和唯一控制权，并非身份认证。不要将端口转发到互联网。HTTP默认开启，HTTPS可选；不是默认HTTPS端口语义。

## 架构

浏览器 → REST/WebSocket → 本包rclpy桥接 → 现有ROS2节点 → Modbus/CAN → 固件。

本包不打开串口。每块板仍只有一个电机驱动worker；网页后端只有一个进程和一个ROS桥接执行线程。自定义动作和服务保持兼容，固件没有变更。网页支持停稳后的DIR高低电平指令切换，不提供直接PWM设置；PWM是低电平占空比，只读。0为持续高电平停止，正常运动上限950‰。

## 安装、构建与启动（xinhua）

```bash
source /opt/ros/humble/setup.bash
cd /data/ros2/robot_motor_ros2_ws
python3 -m venv --system-site-packages .web-venv
.web-venv/bin/python -m pip install -r src/robot_motor_web/requirements.txt
source .web-venv/bin/activate
colcon build --packages-select robot_motor_web --cmake-args \
  -DPython3_EXECUTABLE=$PWD/.web-venv/bin/python \
  -DPYTHON_EXECUTABLE=$PWD/.web-venv/bin/python
source install/setup.bash
export ROS_DOMAIN_ID=173
ros2 run robot_motor_web robot_motor_web serve \
  --config /data/ros2/robot_motor_ros2_ws/review_logs/head_back10_config.yaml \
  --ip 192.168.1.113 --port 8443
```

后台启动时使用nohup，并记下启动PID及日志；禁止启动第二个网页进程，也不要同时启动同板的两组电机节点。`.web-venv`隔离安装，不升级系统ROS依赖。完整依赖版本以已验证的`requirements.lock.txt`为准。

可选HTTPS：先执行`ros2 run robot_motor_web robot_motor_web certificate --ip 192.168.1.113`，再以`serve --https`启动。将`~/.local/share/robot_motor_web/server.crt`导入浏览器信任库；私钥保留在权限600的本机文件中。IP变化时更新`--ip`，HTTPS证书也需要重新签发。

启动前查端口：`ss -ltnp | grep :8443`。访问其他IP或主机名时用`--ip`允许实际设备地址，拒绝未知Host以降低DNS重绑定风险。HTTP/HTTPS都没有登录保护，仅适用于受控局域网。

## 已部署后如何开启网页

当前访问地址：**http://192.168.1.113:8443/**。电脑或手机应与xinhua处于可达的局域网；无需登录。API说明：**http://192.168.1.113:8443/api/docs**。

先在xinhua终端检查服务，已运行就直接打开浏览器，不要重复启动：

```bash
ss -ltnp | grep ':8443'
curl --max-time 5 http://127.0.0.1:8443/api/v1/boards
```

若服务未运行，日常启动不需要重新安装或构建：

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

这是前台启动，保持该终端运行；另一个终端或浏览器执行操作。若希望退出SSH后仍运行，可用以下后台方式，**前台和后台二选一**：

```bash
source /opt/ros/humble/setup.bash
cd /data/ros2/robot_motor_ros2_ws
source install/setup.bash
source .web-venv/bin/activate
export ROS_DOMAIN_ID=173
mkdir -p review_logs
nohup ros2 run robot_motor_web robot_motor_web serve \
  --config /data/ros2/robot_motor_ros2_ws/review_logs/head_back10_config.yaml \
  --ip 192.168.1.113 --port 8443 \
  > review_logs/web_server.log 2>&1 < /dev/null &
echo $! > review_logs/web_server.pid
```

查看日志：`tail -n 80 /data/ros2/robot_motor_ros2_ws/review_logs/web_server.log`。PID文件只记录当次启动器；停止前核对进程，不能盲目使用历史PID。服务未配置为系统开机自启，主机重启后按上述步骤启动。

网页启动后不会自动启动A/B驱动节点。后端重启时恢复草稿，但每板的实际启动权限强制回到只读，不续跑旧任务。首次启动驱动按下一节操作。

## 从只读切换到网页控制

1. 打开页面，确认板名、UID、通信状态、标签及物理接口。A板`main_board / board_a`包括头部和左臂，B板`right_board / board_b`包括右臂；以实际配置和回读为准。
2. 点击顶部“取得控制权”。同一时间一个浏览器会话可写；无需账号密码。控制权不等于节点已允许运动。
3. 点击对应板的“启动 / 参数”。先保持“权限模式”为“只读”，点击“保存文件草稿”，等待成功，再点击“应用并重启本板节点”。等待UID正确、通信新鲜及关节状态更新。
4. 若提示外部节点或串口占用，先确认机构停稳并锁刹车，再在原启动终端停止正确的同板节点。网页不会杀掉未知进程。一块板只允许一个串口控制节点，不要同时运行网页自有节点与CLI启动节点。
5. 需要控制时重新进入“启动 / 参数”，选择下表模式，保存草稿后应用重启。保存草稿没有修改正在运行的节点，也没有写板上Flash。
6. 普通模式重启前要求整板停止并锁刹车、手调关闭、dirty/storage_error/pending_save均正常；存储标志未清时仅可选择下文的受限保存恢复模式，其他停止保护不变。

| 模式 | 页面设置 | 可做的操作 |
|---|---|---|
| 只读 | `read_only` | 身份、角度、FG、PWM、BRAKE及诊断查询；ROS拒绝所有板端写入，包括STOP |
| 保存恢复 | `save_recovery` | 仅STOP及重试SAVE；不运动、不设零、不清故障、不写配置、不手调 |
| 维护 | `maintenance` | 停止、刹车、手调、设零、清故障和配置；网页拒绝自动运动及试转自检 |
| 运动 | `motion` | 在白名单、限位、位置与通信条件满足时执行运动和自检；其他写操作仍受原互锁 |

仅测试头部时，运动白名单填写`head_joint`；单次步长上限先填`10`。已确认机械范围时填写关节`limits_deg`；不知道范围时不能随意编造。仅在现场允许调试无软限位运动时启用`commissioning`，单步上限不代表累计行程限位。默认不要勾选`allow_unreferenced_relative`；绝对运动始终要求有效零点。

关节JSON中的`channel`是物理接口1～4。编辑`limits_deg`时需删除同关节的`limits_rad`，两者不能同时存在。UID、串口、减速比、反相及RPM期望值来自现用配置，不要用示例模板覆盖实机值。

B板右手腕目前软件期望M2、板端旧配置可能仍在M4。绑定冲突会标红并拒绝对应写操作，不能通过修改白名单绕过，也不会自动迁移旧零点。

## 关节功能与操作顺序

### 查询与运动

打开关节卡片即可按标签查询，无需选择数组下标。头部在顶部，下面是左臂和右臂。展开“原始状态 / FG / 保存信息”核对原始0.1°角度、FG计数、位置有效标志、故障与保存信息。位置无效时显示“无有效零点”，不是可信机械0°。

以头部相对+5°为例，先完成运动模式及白名单设置，然后在头部卡片填写：

| 字段 | 示例 | 含义 |
|---|---|---|
| 运动模式 | 相对运动 | 在当前计数基础上增加角度；绝对运动是到指定零点坐标 |
| 输出轴角度 | `5` | +5°；`-5`是逻辑反向，实际前后方向需现场确认 |
| RPM校验（0沿用） | `0` | 沿用板上保存转速；非零只校验，不会临时改变板上转速 |
| 监控超时秒 | `30` | 等待期限，不是指定30秒到位 |

点击“发送一次运动”，只发送一次。查看“请求与结果”，展开原始ROS应答，分别核对`motion_completed`、`persisted`、`position_valid`及`error_code`。HTTP202或动作被接受不等于运动完成和保存成功。不确定结果时先查询，不能重复发送相对运动。

零点无效时绝对运动拒绝；默认相对运动也拒绝。只有现场确认当前机械姿态应作为0°时，才点击“当前姿态设零”，并核对位置有效、原始角度0及保存成功。设零改变坐标，不会让电机转回机械原位。

### 刹车与人工调整

- “锁刹车”：对该关节制动，核对回读BRAKE及停止状态。
- “释放刹车”：普通释放，故障4检测仍有效；不能用它代替自由拖动模式。
- “进入手调”：先支撑负载。PWM保持停止指令、释放刹车；进入即位置无效，该通道暂停故障4计数，整板自动运动和自检受互锁限制。手动FG无法给出可信有符号位置。
- “退出手调”：立即制动，等待静止及保存完成。退出不自动恢复零点；调整后是否设零由操作者确认。

不要通过反复清故障和释放刹车进行拖动。手调也受捕获错误、存储错误等其他保护约束。

### 自检、故障与配置

“往返自检”检测该关节，“整板自检”按固件批量流程检测；都会真实试转。必须确认机械活动空间、停止条件和固件支持后才点击。读取最终自检及往返结果，不能把接受命令当作有响应。

“清故障”用于排查原因并停稳后的人工清除，不会自动恢复零点。“确认UNKNOWN”用于恢复通信、核对原请求与机构状态后解除软件锁，不重发原运动，也不自动继续。

“板上配置 / 保存”与“启动 / 参数”分开：前者操作板端配置。先查看完整四路前后差异；“预览差异 / 暂存回读”会调用ROS配置暂存，但不提交Flash；“确认提交并保存Flash”才提交保存。均要等待任务最终结果及回读确认。要改保存转速为400RPM，应在板上配置中修改该通道`rpm`为400并提交，再同步启动文件中的预期RPM。

## 软件停止按钮与ROS2命令

| 网页按钮 | 作用 |
|---|---|
| 顶部红色“■ 双板停止” | 分别向A、B板发送STOP_ALL，分别报告结果，不是跨板原子停止 |
| 板卡“整板停止” | 停止该板四路 |
| 关节“停止” | 停止该关节 |
| “退出手调” | 结束对应手调并制动 |

目前没有键盘急停快捷键。停止请求优先排队，但仍依赖ROS节点、串口/CAN及板子工作正常。停止按钮不会因为结果未知就显示已停；核对`stop_confirmed: true`及刹车、PWM和静止状态。只读节点拒绝发送STOP，所以只读查询不能提供板端停机能力。按Ctrl+C关闭动作客户端不等于发送STOP。

另一个终端可使用现有ROS2停止服务；先加载同一工作区和域：

```bash
source /opt/ros/humble/setup.bash
source /data/ros2/robot_motor_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
# 仅停止头部
ros2 service call /board_a/robot/stop robot_motor_interfaces/srv/Stop \
  "{joint_name: head_joint}"
# A板全部停止
ros2 service call /board_a/robot/stop robot_motor_interfaces/srv/Stop \
  "{joint_name: ''}"
# B板全部停止
ros2 service call /board_b/robot/stop robot_motor_interfaces/srv/Stop \
  "{joint_name: ''}"
```

浏览器关闭、交还控制或心跳失联5秒，后端尝试停止并制动网页操作过的关节；不要通过关闭浏览器代替主动停止。通信失败显示“停止未确认”。后台崩溃、主机掉电时不能保证发送STOP，网页和ROS2停止均不能代替硬件停机/断电。控制期间不要同时用CLI或另一套桥接程序发送写指令。

结束操作时先停止、退出手调并确认锁刹车及保存，再点击“交还控制权”。前台服务可在启动终端Ctrl+C；关闭网页服务会终止它拥有的节点，不终止外部ROS程序。

## DIR电平换向按钮

用户于2026-10-13确认需要直接切换DIR电平，因此网页将原相对角度符号按钮替换为关节卡片“DIR换向：低→高 / 高→低”。点击后会调用新的ROS2服务，再发送固件已有DIRECTION操作码7；无需修改或重新烧录固件。

1. 停止运动、退出整板手调，并锁定目标关节刹车；等待至少500ms无FG。确认电机不会自行转动，再进行接线电平调试。
2. 网页取得控制权，节点处于维护或运动模式。只读、保存恢复、UNKNOWN、存储错误、故障、绑定冲突、通信过期及运行/排队运动时拒绝切换。
3. 按钮根据最新回读显示低→高或高→低，点击后再次确认。请求明确指定目标电平high=true/false，不使用服务器重复“toggle”，避免请求重放导致来回翻转。
4. 展开结果：accepted=true、direction_confirmed=true、unknown=false，且direction等于请求high，才表示固件DIR指令位回读一致。它不是GPIO或接口电压采样；芯片脚/电机方向线电压仍需实测。
5. 本按钮不释放刹车、不输出运行PWM、不设零、不更改inverted、不写Flash。姿态和零点不因本命令主动改变。

**下一次MOVE会按角度正负与inverted重新设置DIR，手工切换电平不会永久改变正负角度对应方向。** 正常运动反向仍使用负角度；要永久反相需单独配置inverted并处理失效零点。

REST：`POST /api/v1/boards/{board}/joints/{joint}/direction`，请求`{"high":true}`或`{"high":false}`，沿用控制权、CSRF、请求ID及任务查询机制。超时/UNKNOWN不自动切换第二次。

ROS2：`/<namespace>/robot/set_direction`，类型`robot_motor_interfaces/srv/SetDirection`。手工调用示例（执行会改变DIR指令）：

```bash
source /opt/ros/humble/setup.bash
source /data/ros2/robot_motor_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
ros2 service call /board_a/robot/set_direction \
  robot_motor_interfaces/srv/SetDirection \
  "{joint_name: left_upper_arm_joint, high: true}"
```

high=true对应高电平输出命令，false对应低电平，与当前inverted无关。实际映射见固件BSP/motor_io.c: motor_set_dir。需要更新接口、驱动、网页并重启对应软件以加载服务；客户端重新source，不重复启动串口节点。

## 存储错误：网页重试保存

板卡新增“重试保存”，对应`POST /api/v1/boards/{board}/save`（请求`{"confirm":true}`）以及ROS2服务`/<namespace>/robot/retry_save`，类型`robot_motor_interfaces/srv/RetrySave`。

1. 停止整板，退出人工调整，确认四路刹车锁定、PWM0、转速0且通信/UID新鲜。按钮不主动停止机构，也不自动退出手调。
2. 点击“取得控制权”。普通维护/运动节点已运行时，可直接点击“重试保存”，即使storage_error=1也允许这一次恢复请求。
3. 若需要重启加载新程序，而普通模式被“dirty/存储/待保存未清”拦截，在“启动 / 参数”选择**保存恢复（仅STOP/SAVE）**，保存草稿后应用重启。该模式只豁免存储标志导致的重启拒绝，仍检查身份、通信、UNKNOWN、任务、整板停止/制动和静止；没有放宽普通模式的重启规则。
4. 等待板卡显示“保存恢复”，点击“重试保存”，确认后发送一次。固件仍要求连续500ms无FG，新的FG活动可能导致BUSY或保存失败。
5. 展开最终结果，只有`accepted=true`、`persisted=true`、`flags_valid=true`，且`dirty/storage_error/pending_save`全部false，才表示保存已确认。`flags_valid=false`时那些false默认值不能当成实际回读值。超时或unknown不自动重试。
6. SAVE保存当前已应用配置及当前的位置/有效标志，不提交配置暂存、不清故障、不设零、不运动。原本无效的位置保存后仍无效，原电机故障仍需单独排查。
7. 保存成功后，才能按普通规则切换维护/运动模式并应用重启；再处理原FG故障。保存失败继续保持停止，检查FG活动和Flash写入，不以设零代替恢复。

只读仍拒绝SAVE。受限模式在驱动worker中拦截其他所有写操作，CLI同样不能绕过；不要把它当普通维护模式。启动参数`save_recovery_only:=true`会强制关闭自动运动、仅允许停止与RetrySave，默认false。后端重启后仍默认只读，不自动恢复该模式。

终端手动调用示例（只有显式发送才会写Flash）：

```bash
source /opt/ros/humble/setup.bash
source /data/ros2/robot_motor_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
ros2 service call /board_a/robot/retry_save \
  robot_motor_interfaces/srv/RetrySave "{confirm: true}"
```

如果提示服务不可用，确认更新后的接口包、驱动包及网页包已构建，终端重新source且对应节点已加载新程序。需要更新四个包：`robot_motor_interfaces robot_motor_driver robot_motor_bringup robot_motor_web`，已有首次安装命令的packages-select改为这四个名字。不要重复启动同板节点。

## REST与WebSocket

浏览器自动处理匿名Cookie、CSRF及请求ID。命令行示例（仅查询/取得会话）：

```bash
curl -c /tmp/motor-web.cookies http://192.168.1.113:8443/api/v1/session
curl http://192.168.1.113:8443/api/v1/boards
curl http://192.168.1.113:8443/api/openapi.json
```

写请求使用Cookie及`X-CSRF-Token`，每个新操作使用唯一`X-Request-ID`。相同ID+相同内容返回原任务，相同ID不同内容拒绝。需先POST`/api/v1/control/acquire`；然后POST`/api/v1/control/heartbeat`每秒续期。任务在SQLite保存，网关重启将未结束记录标为UNKNOWN，不自动重放。

| 接口（前缀/api/v1） | 功能 |
|---|---|
| GET /boards；GET /boards/{board}/state | 板信息、配置、通信及存储 |
| GET /boards/{board}/joints/{joint}/state | 按标签读取，含物理接口与绑定冲突 |
| POST .../joints/{joint}/move | 度数动作，返回任务ID |
| POST .../joints/{joint}/stop；POST /boards/{board}/stop | 单路/整板停止 |
| POST /boards/{board}/save | 显式重试SAVE，返回保存回读标志 |
| POST /stop-all | 分别停止两板，返回两个独立结果 |
| POST .../direction | 停稳制动后设置DIR指令高/低，下一次MOVE会覆盖 |
| POST .../brake、manual-adjust、zero、clear-fault | 刹车、手调、设零、清故障 |
| POST /boards/{board}/selftest | 单路/整板往返自检 |
| POST .../config/preview、config/apply | 暂存预览/提交保存 |
| POST .../acknowledge-unknown | 现场核对后解除软件锁，不恢复运动 |
| GET /jobs、GET /jobs/{id}、POST /jobs/{id}/cancel | 原始结果和动作取消 |
| GET/PUT .../runtime-config、POST .../restart | 文件草稿/自有节点重启 |
| GET /parameters | 参数作用、单位、范围、源码出处 |

完整字段由 `/api/docs` 本地API参考和 `/api/openapi.json` 给出，页面没有CDN。WebSocket `/events?csrf=...`使用同一Cookie，只推送状态/任务，不通过消息直接驱动电机。客户端在实际连接上时才发送控制心跳。

兼容：原ROS `/move_joint`及`angle_rad`仍存在；网页HTTP动作仅提供度数，不能通过切换单位绕过限制。标准joint_states仍为rad。

## 参数与代码出处

本包`models.py`增加网页输入范围校验；`runtime.py`生成安全参数列表，无shell=True；`gateway.py`处理控制权、互锁、断线制动；`bridge.py`只调用ROS客户端；`api.py`提供OpenAPI与参数出处。启动参数和固件范围仍以主使用手册第17节为准。

本地维护：固件工程`ros2/robot_motor_web`；独立工程`src/robot_motor_web`；远程`/data/ros2/robot_motor_ros2_ws/src/robot_motor_web`。服务状态目录默认`~/.local/share/robot_motor_web`，含runtime.json、修改前备份、每板启动文件/日志及jobs.sqlite3，不进入Git。

右手腕：软件M2与旧板端M4可能冲突，页面必须以实读标红，禁止相应关节写入。未完成配置迁移和零点确认，不得试转。

## 排错与停止服务

结果记录点击整行标题或▶展开；实时刷新会保留展开状态。若仍使用旧页面，请先确认电机停稳，再Ctrl+F5强制刷新。刷新可能使控制会话短暂失联，重新连接不自动续跑。


- 未显示板：核对ROS_DOMAIN_ID=173、命名空间、驱动节点及串口，节点存在不等于UID正确。
- 只读拒绝：先以网页自有只读节点接管，再在静止条件下切换维护/运动。
- 外部节点占用：用pgrep/fuser核对，手动停止正确进程，不重复启动。
- UNKNOWN/109：先核对原请求结果和机构停稳，再确认UNKNOWN；不自动重发。
- 104：位置无效，不自动设零；绝对运动必须建立可信零点。
- 105：区分单次步长与机械终点限制；上限不能当累计行程限位。
- RPM mismatch：动作填0或与实读一致；改速走配置保存，不是修改动作字段。
- 状态过期、UID错误、绑定冲突：写按钮拒绝；原始零值不能当真实测量。
- 保存失败：检查原始persisted及三个保存标志，不能只看任务被接受。
- 停止未确认：现场制动/断电处理后回读，不把网络恢复当作已停。

停止网页：先交还控制、确认制动，然后在服务终端Ctrl+C；后台服务使用记录的PID。只终止网页拥有的节点，不停止其他ROS程序。删除软件时保留或备份状态目录与审计记录；不清除板Flash。

写操作验证使用模拟板；部署只查询真实板身份、状态和配置。实机运动、手调和Flash事务需单独现场授权。

## 双板串口与连接检查

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
