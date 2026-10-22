/*
 * robot.h — 不依赖 GD32 寄存器的四路机器人状态机，硬件操作由 robot_ops_t 注入。
 * 主循环串行调用命令/更新接口；角度用输出轴 0.1 度，位置用有符号 FG 脉冲，RPM 为减速前转速。
 * 负责运动、自检、手调、互锁、故障及保存；命令接受成功不表示运动完成或位置已保存。
 */
#ifndef ROBOT_H
#define ROBOT_H
#include <stdint.h>
#include <stddef.h>
#define ROBOT_CHANNELS 4U
#define ROBOT_SCHEMA 1U
/* Wire command revision, independent of the persistent Flash schema. */
#define ROBOT_COMMAND_PROTOCOL 2U
/* Manual commands: allocate a fresh nonzero identity for EVERY submission. */
#define ROBOT_AUTO_SEQUENCE 0U
/* 到位余转误差上限50个0.1度=5度；不是正常停止的提前量。 */
#define ROBOT_TOLERANCE_DDEG 50
#define ROBOT_QUIET_MS 500U
/* 累计非预期活动；初始化、清故障、设零、手调进入及故障4事件恢复路径会重置。 */
/* 非预期累计活动上限100个0.1度=10度；超过而非达到才算事件。 */
#define ROBOT_UNEXPECTED_FG_DDEG 100U
#define ROBOT_FG_PULSES_PER_MOTOR_REV 6U
#define ROBOT_FAULT4_RECOVERY_MS 5000U
/* 能力位bit0=手动调整，bit1=故障4次数窗口自动恢复。 */
#define ROBOT_CAPABILITIES 3U
/* 运行无反馈1000ms；控制周期10ms；减速区300个0.1度=30度；占空比上限950/1000。 */
#define ROBOT_FEEDBACK_MS 1000U
#define ROBOT_CONTROL_MS 10U
/* Initial commissioning parameters; must be tuned with the actual load. */
#define ROBOT_CRAWL_RPM 400U
#define ROBOT_DECEL_DDEG 300U
#define ROBOT_DUTY_MAX 950U
/* Response probe only: low-time duty /1000, never use the normal speed PI. */
/* 自检独立参数：最大10%低占空比，每10ms增加1%，每程最多2000ms，去程至少3脉冲。 */
#define ROBOT_SELFTEST_DUTY_MAX 100U
#define ROBOT_SELFTEST_DUTY_STEP 10U
#define ROBOT_SELFTEST_POWER_MS 2000U
#define ROBOT_SELFTEST_EDGES 3U
#define ROBOT_SELFTEST_STOP_MS 2000U
/* 返回码从0依次为成功、参数、忙、未就绪、存储、序号、地址错误；不代表最终运动结果。 */
enum { R_OK, R_VALUE, R_BUSY, R_UNREADY, R_STORAGE, R_SEQUENCE, R_ADDRESS };
/* 运行状态编号是通信接口的一部分：0未就绪、1空闲、2运行、3减速、4等待静止、5完成、6故障、7自检、8手调。 */
enum { S_UNREADY, S_IDLE, S_RUNNING, S_DECEL, S_SETTLING, S_DONE, S_FAULT, S_SELFTEST, S_MANUAL };
/* 故障编号：0无、1无反馈、2总超时、3捕获、4非预期活动、5位置、6存储保留、7自检。 */
enum { F_NONE, F_FEEDBACK, F_TIMEOUT, F_CAPTURE, F_UNEXPECTED, F_POSITION, F_STORAGE, F_SELFTEST };
/*
 * 命令编号固定：1配置提交、2设零、3相对运动、4单路停、5全停、6刹车、7DIR、8保存、9清故障。
 * 10单路自检、11整板扫描、12手调；编号属于线上协议，不可随意重排。
 */
enum { CMD_CONFIG=1, CMD_ZERO, CMD_MOVE, CMD_STOP, CMD_STOP_ALL,
       CMD_BRAKE, CMD_DIRECTION, CMD_SAVE, CMD_CLEAR_FAULT, CMD_SELFTEST, CMD_SELFTEST_ALL, CMD_MANUAL_ADJUST };
/*
 * 自检状态0～7：未检测/未执行、排队、通电试转、等待静止、通过、失败、取消、跳过。
 * 结果只表示最近一次自检，不是持续在线检测。
 */
enum { ST_UNTESTED, ST_QUEUED, ST_RUNNING, ST_SETTLING, ST_PASSED,
       ST_FAILED, ST_CANCELLED, ST_SKIPPED };
/*
 * 原因0～10：无、无脉冲、脉冲不足、捕获错误、其他路活动、停止超时、取消、存储错、位置溢出、回程超时、历史残差错。
 * 当前残差只记录，编号10保留兼容；通过与保存成功是两个独立结论。
 */
enum { ST_REASON_NONE, ST_NO_PULSES, ST_TOO_FEW_PULSES, ST_CAPTURE_ERROR,
       ST_OTHER_ACTIVITY, ST_STOP_TIMEOUT, ST_CANCEL, ST_STORAGE_ERROR, ST_POSITION_OVERFLOW,
       ST_RETURN_TIMEOUT, ST_RETURN_ERROR }; /* 10 reserved for legacy residual failures. */
/*
 * 阶段0～5：无、去程通电、去程停车等待、回程通电、回程停车等待、结束。
 */
enum { SP_NONE, SP_OUTBOUND, SP_OUTBOUND_SETTLING, SP_RETURN, SP_RETURN_SETTLING, SP_FINISHED };
/* 自检结果仅RAM保存：时间ms、计数为FG脉冲；residual为有符号去回差。 */
typedef struct {
    /*
     * state：ST_*自检状态；reason：ST_*原因，0无异常；两者均为协议编码。
     */
    uint8_t state, reason;
    /*
     * powered_ms：去程实际通电ms，截为uint16；sequence：本次请求标识，供轮询匹配结果。
     */
    uint16_t powered_ms, sequence;
    /*
     * pulses：去程+回程含余转累计无符号FG数；completed_ms：完成时系统运行毫秒，可回绕。
     */
    uint32_t pulses, completed_ms;
    /*
     * phase：SP_*行程阶段；return_ms：回程通电毫秒，不含停止等待。
     */
    uint16_t phase, return_ms;
    /*
     * 分别记录去程/回程含余转的脉冲活动量；不是有符号角度。
     */
    uint32_t outbound_pulses, return_pulses;
    /*
     * residual=最终计数位置-起始计数位置，符号随逻辑去程方向；只是诊断，不据此失败或强行归零。
     */
    int32_t residual;
} robot_selftest_result_t;
/* 整板自检会话：一次只驱动一路；stop_fault_mask记录停止确认超时互锁。 */
typedef struct {
    /*
     * active：批次独占锁；current：0～3被测路；negative：命令逻辑方向。
     * aborting：不再开始下一路；terminal_state：停车后的预期最终状态。
     * stop_fault_mask：各通道停止确认超时位，批次结束仍会阻止新运动，须静止后确认清除。
     */
    uint8_t active, current, negative, aborting, terminal_state, stop_fault_mask;
    uint8_t scan_all; /* Batch probes all physical outputs without configuration. */
    /*
     * 当前行程起动、物理停止及最近占空比递增时间，单位ms；切回程会重新设置。
     */
    uint32_t started_ms, stopped_ms, ramp_ms;
    /*
     * 当前行程目标脉冲：去程3；回程为去程含余转总数，由硬件保护与主循环共用。
     */
    uint32_t leg_target;
    /*
     * 自检开始时有符号位置，计算残差用；不代表它一定有有效零点。
     */
    int32_t origin_position;
} robot_selftest_session_t;
/* 通道持久配置：启用、角色0～7、减速比、方向反相0/1、减速前RPM400～8000。 */
typedef struct { uint32_t enabled, role, gear, inverted, rpm; } robot_config_t;
/* Flash镜像布局必须兼容ROBOT_SCHEMA；position是有符号FG数，valid为零点可信标志。 */
typedef struct {
    /*
     * protocol：0=Modbus USART0、1=CAN；address：1～247；node：1～127；pwm_hz：四路共用Hz。
     */
    uint32_t protocol, address, node, pwm_hz;
    robot_config_t channel[4];
    int32_t position[4]; /* signed FG edges, not modulo a revolution */
    uint32_t valid[4]; /* trusted zero reference; not a relative MOVE prerequisite */
    /*
     * 保存请求高水位及是否见过请求；它不是完整16项去重缓存，重启后仍需重新建立通信状态。
     */
    uint32_t request_sequence, request_seen;
} robot_image_t;
/* 一致反馈快照：累计边沿数、减速前RPM、累计捕获错误；count/errors通过差分消费。 */
typedef struct {
    /*
     * count：ISR累计FG；rpm：减速前速度；errors：捕获错误累计数，主循环比较前后快照检测新增。
     */
    uint32_t count, rpm, errors;
} robot_feedback_t;
/* 运行态只存RAM；duty为低电平千分比，position/target为脉冲，所有*_ms为系统毫秒。 */
typedef struct {
    /*
     * state：S_*；fault：F_*；valid：零点可信，而非物理在线标志。
     * brake：1=制动低电平、0=释放高；direction：期望DIR0/1，均不是引脚电压采样。
     */
    uint8_t state, fault, valid, brake, direction;
    /*
     * 当前受控行程的逻辑计数方向+1/-1；停止确认后归0，手调始终为0。
     */
    int8_t sign;
    /*
     * 期望低电平千分比，正常0～950、自检0～100、停止0；不代表实测占空比。
     */
    uint16_t duty;
    /*
     * 有符号FG位置与目标，不按每圈取模；显示角度需减速比换算并检查valid。
     */
    int32_t position, target;
    /*
     * count/errors：已消费ISR累计值；rpm：反馈速度。
     * last_edge_ms：最近FG时间，起动时也重置；started_ms：命令开始时间；timeout_ms：整次运动上限。
     */
    uint32_t count, errors, rpm, last_edge_ms, started_ms, timeout_ms;
    uint32_t unexpected_fg; /* RAM-only, saturating; excludes commanded motion. */
    uint8_t manual, recovery_events; /* manual: 0 off, 1 active, 2 exiting */
    /*
     * 故障4当前5秒事件窗口起点，使用无符号差判断到期以适应时钟回绕。
     */
    uint32_t recovery_ms;
    /*
     * 正常速度控制积分项，每个控制周期加error/40并限制0～700；新MOVE清0。
     */
    int32_t integral;
    /*
     * 1表示因达到目标而停车，静止后需检查5度容差；普通STOP/故障/手调不作为正常到位。
     */
    uint8_t completing;
} robot_motor_t;
typedef struct {
    /* output must apply PWM=stop before changing DIR/brake. duty is low-time /1000. */
    void (*output)(uint8_t, uint16_t, uint8_t, uint8_t);
    /*
     * 保存回调：返回1确认写入，0失败；必须由硬件适配层实现静止和写前后FG检查。
     */
    int (*save)(const robot_image_t *, uint32_t dirty);
    /*
     * 设置四路共同PWM频率，单位Hz；当前实现先停所有PWM，调用者必须保证静止。
     */
    void (*frequency)(uint32_t);
} robot_ops_t;
/* config为生效配置、staged为待提交草稿；通信身份在初始化时锁定，修改后须重启。 */
typedef struct {
    robot_image_t config, staged;
    robot_motor_t motor[4];
    /* RAM-only; robot_image_t / Flash schema deliberately unchanged. */
    robot_selftest_result_t selftest[4];
    robot_selftest_session_t probe;
    robot_ops_t ops;
    /*
     * uid为芯片96位身份；now_ms为当前业务快照时间；control_ms记录上次闭环更新时刻。
     */
    uint32_t uid[3], now_ms, control_ms;
    /*
     * dirty：运动/手调未干净保存；pending_save：等待静止统一保存；storage_error：保存失败锁存。
     * protocol/address/node为本次开机通信身份；stop_input为启用的停机输入采样，非默认启用。
     */
    uint8_t dirty, pending_save, storage_error, protocol, address, node, stop_input;
    /*
     * command[5]保存最近完整命令seq/op/ch/arg高/arg低；last_seq/result为已记录请求及业务返回码。
     */
    uint16_t command[5], last_seq, last_result;
    /*
     * 是否存在有效请求高水位，用于首条命令和重启后的序号检查。
     */
    uint8_t have_seq;
    /*
     * 最近16条请求的身份、载荷与接受结果；同序号同内容返回缓存，不再次执行。
     */
    struct { uint16_t seq, op, ch; int32_t arg; uint16_t result; } cache[16];
    /*
     * cache_used最多16；cache_next循环覆盖最旧槽，均仅RAM保存。
     */
    uint8_t cache_used, cache_next;
} robot_t;
/**
 * @brief 生成出厂镜像：通道禁用、无角色/减速比，RPM=8000，地址/节点=1，PWM=25 kHz。
 *
 * 输入：可写robot_image_t指针，调用者保证非NULL。
 * 过程：整个镜像清0，仅补地址1、节点1、PWM25000及四路RPM8000。
 * 结果：protocol=0，enabled/role/gear/valid均0；只写RAM，不保存Flash。
 */
void robot_defaults(robot_image_t *image);
/**
 * @brief 检查协议、地址、PWM频率、各通道范围及非零角色唯一性；合法返回 1，否则 0。
 *
 * 输入：待校验镜像指针；不修改镜像。
 * 检查：protocol0/1，Modbus1～247，CAN1～127，PWM20～30kHz；RPM400～8000。
 * 角色：非零role不能重复，即使该通道禁用也参与重复检查；enabled通道须有role和gear。
 * 限制：不校验物理接线、位置可信性或机械限位；返回1仅说明配置字段符合规则。
 */
int robot_config_valid(const robot_image_t *image);
/**
 * @brief 恢复合法配置和位置；dirty 恢复时零点无效，上电全部制动，不继续上次动作。
 *
 * 输入：ops为有效硬件回调，p可NULL，uid须3个32位字，now单位ms，dirty为上次未完整保存标志。
 * 过程：清RAM状态，使用合法p或默认值，复制staged及通信身份，恢复请求高水位。
 * 位置：dirty非0时所有有效零点失效；保存计数仍可用于诊断。
 * 输出：四路制动后设置PWM频率；缓存/手调/自检结果重置，不续跑旧运动。
 */
void robot_init(robot_t *, const robot_ops_t *, const robot_image_t *, int dirty,
                const uint32_t uid[3], uint32_t now);
/**
 * @brief 主循环传入同一时刻的四路反馈快照和 ms 时钟；处理事件、闭环、停止确认及统一保存。
 *
 * 输入：now为单调可回绕ms，f[4]为一致的累计计数/速度/错误快照。
 * 顺序：自检独占则推进probe；否则处理捕获错/FG增量，再故障4恢复，再速度/角度控制。
 * 控制：约10ms一次，无补跑循环；超时优先于到位，受控余转仍计入当前位置。
 * 完成：静止500ms后检查5度容差，置DONE/FAULT等，满足条件再保存。
 * 限制：同一robot_t不能被多个线程/ISR并发推进，调用者须串行化。
 */
void robot_update(robot_t *, uint32_t now, const robot_feedback_t feedback[4]);
/* Includes the whole exclusive probe session, including inter-channel quiet time. */
/**
 * @brief 任一通道忙或整个自检会话未结束即返回 1，包括通道切换间隙。
 *
 * 过程：先检查probe.active，再扫描四路busy。
 * 作用：自检停止后等待/切通道期间仍不放行Flash；手调也算忙。
 * 限制：返回0仍不保证500ms静止，需quiet和硬件反馈保护配合。
 */
int robot_is_busy(const robot_t *);
/* Maximum tolerated unsolicited pulses, UINT32_MAX disables F_UNEXPECTED. */
/**
 * @brief 返回允许的非预期 FG 数，严格超过才触发；516/290/82 返回 86/48/13，无有效减速比返回 UINT32_MAX。
 *
 * 输入：已提交生效减速比gear；不是上位机草稿值。
 * 换算：floor(10度*6*gear/360)，返回容许的整数脉冲上限。
 * 阈值：516第87、290第49、82第14个累计脉冲才超限；未知gear返回UINT32_MAX禁用故障4。
 */
uint32_t robot_unexpected_fg_limit(uint32_t applied_gear);
/* Also cancels a probe, without consuming the host's request sequence. */
/**
 * @brief 取消自检并停止四路，手调转退出等待；不消耗上位机命令序号。
 *
 * 过程：自检活动时先取消当前批次，然后对四路执行stop。
 * 结果：PWM0、制动；手调退出等待，故障不自动清除。
 * 限制：不是硬件异常ISR入口，最终静止/保存要继续主循环。
 */
void robot_stop_all(robot_t *);
/* seq=0 is not replay-safe. Read 0x36 for the allocated identity/result at 0x37. */
/**
 * @brief seq=0分配新序号；非零序号查询16项缓存避免重执行，同序号不同内容拒绝；结果表示接受情况。
 *
 * 输入：seq非零用于重试去重，0表示每次分配新身份；op/ch/arg必须保持重试内容一致。
 * 去重：16项RAM缓存命中返回原结果；相同seq不同参数、陈旧/相同高水位拒绝。
 * 执行：先更新请求序号，再execute，并缓存结果；未接受的业务结果也可缓存。
 * 重启：缓存丢失，保存的序号用于阻止旧请求重执行；不能把seq=0用作可靠重发。
 */
uint16_t robot_command(robot_t *, uint16_t seq, uint16_t op, uint16_t ch, int32_t arg);
/**
 * @brief 连续读取1～125个寄存器到调用方缓冲；遇未定义地址返回错误，缓冲可能已填部分数据。
 *
 * 输入：a起始地址，n=1～125，v至少n个uint16元素。
 * 过程：先防地址溢出，再逐项read_one；任一项不合法立即返回错误。
 * 限制：失败可能已填部分缓冲，但协议层不会把它当正常完整响应发送。
 */
int robot_read(const robot_t *, uint16_t address, uint16_t count, uint16_t *values);
/**
 * @brief 0x30起连续5字原子提交即执行；其余合法写入仅改 staged，旧0x35提交帧不再可写。
 *
 * 输入：v为n个寄存器，完整命令必须a=0x30且n=5。
 * 命令：字段为seq/op/ch/arg高16/arg低16，验证交robot_command，收到即执行，无A55A提交。
 * 配置：先检查整个区间都可写，再只改staged；需要CMD_CONFIG才保存并应用。
 * 限制：部分命令写及旧0x35写拒绝；普通暂存成功不表示值已通过完整配置校验。
 */
int robot_write(robot_t *, uint16_t address, uint16_t count, const uint16_t *values);
/**
 * @brief 将有符号 FG 计数换为输出轴 0.1度；减速比为0返回0，结果溢出时饱和，channel须有效。
 *
 * 输入：通道0～3和有符号脉冲数；本函数不检查通道越界。
 * 公式：pulses*3600/(gear*6)，结果为输出轴0.1度，整数除法向0截断。
 * 返回：gear=0为0；超int32范围饱和。此函数不检查valid，调用者必须同时显示位置有效标志。
 */
int32_t robot_angle(const robot_t *, unsigned channel, int32_t pulses);
#endif
