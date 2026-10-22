/*
 * robot.c — 不依赖 GD32 寄存器的四路机器人状态机，硬件操作由 robot_ops_t 注入。
 * 主循环串行调用命令/更新接口；角度用输出轴 0.1 度，位置用有符号 FG 脉冲，RPM 为减速前转速。
 * 负责运动、自检、手调、互锁、故障及保存；命令接受成功不表示运动完成或位置已保存。
 */
#include "robot.h"
#include <string.h>
#include <limits.h>

/**
 * @brief 生成出厂镜像：通道禁用、无角色/减速比，RPM=8000，地址/节点=1，PWM=25 kHz。
 *
 * 输入：可写robot_image_t指针，调用者保证非NULL。
 * 过程：整个镜像清0，仅补地址1、节点1、PWM25000及四路RPM8000。
 * 结果：protocol=0，enabled/role/gear/valid均0；只写RAM，不保存Flash。
 */
void robot_defaults(robot_image_t *p)
{
    memset(p, 0, sizeof(*p));
    p->address=1; p->node=1; p->pwm_hz=25000;
    for(unsigned i=0;i<4;i++) p->channel[i].rpm=8000;
}
/**
 * @brief 检查协议、地址、PWM频率、各通道范围及非零角色唯一性；合法返回 1，否则 0。
 *
 * 输入：待校验镜像指针；不修改镜像。
 * 检查：protocol0/1，Modbus1～247，CAN1～127，PWM20～30kHz；RPM400～8000。
 * 角色：非零role不能重复，即使该通道禁用也参与重复检查；enabled通道须有role和gear。
 * 限制：不校验物理接线、位置可信性或机械限位；返回1仅说明配置字段符合规则。
 */
int robot_config_valid(const robot_image_t *p)
{
    /* 协议0=Modbus、1=CAN；配置仅允许516/82/290或未配置0，已启用通道必须有角色和减速比。 */
    if(p->protocol>1 || p->address<1 || p->address>247 || p->node<1 ||
       p->node>127 || p->pwm_hz<20000 || p->pwm_hz>30000) return 0;
    for(unsigned i=0;i<4;i++) {
        const robot_config_t *c=&p->channel[i];
        if(c->enabled>1 || c->role>7 || c->inverted>1 || c->rpm<400 || c->rpm>8000 ||
           (c->gear!=0 && c->gear!=516 && c->gear!=82 && c->gear!=290) ||
           (c->enabled && (!c->gear || !c->role))) return 0;
        for(unsigned j=0;j<i;j++) if(c->role && c->role==p->channel[j].role) return 0;
    }
    return 1;
}
/**
 * @brief 把单路软件期望输出交给硬件回调；i 必须为 0～3，不是实际电平回读。
 *
 * 输入：r与有效通道i；读取m->duty/direction/brake交给ops.output。
 * 副作用：可能立刻写GPIO/PWM；不独立检查互锁，不读取物理电平。
 */
static void output(robot_t *r, unsigned i)
{
    robot_motor_t *m=&r->motor[i];
    r->ops.output((uint8_t)i,m->duty,m->direction,m->brake);
}
/**
 * @brief 手调、运行、减速、等待静止及自检均视为忙；用于互锁和保存保护。
 *
 * 输入：一个电机状态对象；不修改状态。
 * 返回：manual非0或状态处于RUNNING/DECEL/SETTLING/SELFTEST时为真。
 * 注意：刹车释放本身不属于busy，故障或DONE本身也不属于busy。
 */
static int busy(const robot_motor_t *m)
{ return m->manual || m->state==S_RUNNING || m->state==S_DECEL || m->state==S_SETTLING || m->state==S_SELFTEST; }
/**
 * @brief 返回允许的非预期 FG 数，严格超过才触发；516/290/82 返回 86/48/13，无有效减速比返回 UINT32_MAX。
 *
 * 输入：已提交生效减速比gear；不是上位机草稿值。
 * 换算：floor(10度*6*gear/360)，返回容许的整数脉冲上限。
 * 阈值：516第87、290第49、82第14个累计脉冲才超限；未知gear返回UINT32_MAX禁用故障4。
 */
uint32_t robot_unexpected_fg_limit(uint32_t gear)
{
    if(gear!=516U && gear!=290U && gear!=82U) return UINT32_MAX;
    return (uint32_t)((uint64_t)ROBOT_UNEXPECTED_FG_DDEG *
                      ROBOT_FG_PULSES_PER_MOTOR_REV * gear / 3600U);
}
/**
 * @brief 饱和累计非预期脉冲并检查超限；累计的是活动量，不是有方向的净偏移。
 *
 * 输入：delta为本周期新增无符号FG数，gear为生效减速比。
 * 过程：delta=0立即返回0；否则饱和累加，严格大于容许数返回1。
 * 限制：函数本身不清故障或制动；未知gear虽可累计但不会超过UINT32_MAX触发。
 */
static int unexpected_edges(robot_motor_t *m, uint32_t delta, uint32_t gear)
{
    if(!delta) return 0;
    if(delta>UINT32_MAX-m->unexpected_fg) m->unexpected_fg=UINT32_MAX;
    else m->unexpected_fg+=delta;
    return m->unexpected_fg>robot_unexpected_fg_limit(gear);
}
/**
 * @brief 任意通道手调中或退出等待中返回 1，用于整板自动运动互锁。
 *
 * 过程：遍历四路manual，发现1（手调）或2（退出等待）即返回1。
 * 影响：用于整板禁止自动运动/配置等；不是单个关节的使能位。
 */
static int manual_any(const robot_t *r)
{
    for(unsigned i=0;i<4;i++) if(r->motor[i].manual) return 1;
    return 0;
}
/**
 * @brief 清空故障4事件窗口和累计活动量；不修改位置、输出或 Flash。
 *
 * 输入：单路运行状态；清unexpected_fg、recovery_events、recovery_ms。
 * 限制：只复位计数器，故障位、位置有效位和输出需调用者另行处理。
 */
static void recovery_reset(robot_motor_t *m)
{ m->unexpected_fg=0; m->recovery_events=0; m->recovery_ms=0; }
/**
 * @brief 首事件或第3事件开启新的 5秒窗口并记为1；第2事件只累加，不延长窗口。
 *
 * 输入：now为可回绕系统毫秒，m为故障4的运行状态。
 * 规则：首次记1；第二次记2；第三次把它当新窗口首次，记1并更新时间。
 * 限制：这里处理已成立的超限事件，不能按每个普通FG脉冲调用。
 */
static void recovery_event(robot_motor_t *m, uint32_t now)
{
    if(!m->recovery_events || ++m->recovery_events>=3) {
        m->recovery_events=1; m->recovery_ms=now;
    }
}
/**
 * @brief 任一通道忙或整个自检会话未结束即返回 1，包括通道切换间隙。
 *
 * 过程：先检查probe.active，再扫描四路busy。
 * 作用：自检停止后等待/切通道期间仍不放行Flash；手调也算忙。
 * 限制：返回0仍不保证500ms静止，需quiet和硬件反馈保护配合。
 */
int robot_is_busy(const robot_t *r)
{
    if(r->probe.active) return 1;
    for(unsigned i=0;i<4;i++) if(busy(&r->motor[i])) return 1;
    return 0;
}
/**
 * @brief 要求四路都不忙，且每路距最近 FG 至少 500 ms；用于保存/配置等整板操作。
 *
 * 输入：r->now_ms须已更新，读取各路last_edge_ms及busy。
 * 返回：四路都非忙且分别连续至少500ms无FG才为1。
 * 限制：不检查storage_error或配置有效性；这些由各命令单独判断。
 */
static int quiet(const robot_t *r)
{
    for(unsigned i=0;i<4;i++) if(busy(&r->motor[i]) ||
       (uint32_t)(r->now_ms-r->motor[i].last_edge_ms)<ROBOT_QUIET_MS) return 0;
    return 1;
}
/**
 * @brief 将运行位置、零点有效标志和最近请求序号复制到待保存镜像。
 *
 * 输入：r的当前状态和待填镜像p。
 * 过程：只覆写position[4]、valid[4]、request_sequence/request_seen；保留p原配置字段。
 * 用途：可同时用于当前配置保存及候选配置事务，不能把staged草稿意外当作生效值。
 */
static void image_positions(robot_t *r, robot_image_t *p)
{
    for(unsigned i=0;i<4;i++) { p->position[i]=r->motor[i].position; p->valid[i]=r->motor[i].valid; }
    p->request_sequence=r->last_seq; p->request_seen=r->have_seq;
}
/* Complete events are counted even while a selftest is waiting to stop. */
/**
 * @brief 故障4锁存后按每个完整超限活动量统计事件；单个 FG 不会重复计为事件。
 *
 * 输入：delta为本次新增FG，gear已生效，now为本次快照时间。
 * 首发：无其他故障时累计并返回1通知调用者停机；其他故障存在则不新增故障4。
 * 锁存后：按limit+1分组算完整事件，保留余数，一批可处理多个事件；第3事件刷新窗口。
 * 限制：锁存后的处理返回0，因为无需再次触发首发停机路径。
 */
static int fault4_edges(robot_motor_t *m, uint32_t delta, uint32_t gear, uint32_t now)
{
    /* 锁存后按 limit+1 个脉冲分组；余数留到下次，防止把一次大批量反馈误当单个事件。 */
    if(m->fault!=F_UNEXPECTED) return !m->fault && unexpected_edges(m,delta,gear);
    uint32_t limit=robot_unexpected_fg_limit(gear);
    if(limit!=UINT32_MAX) {
        uint64_t activity=(uint64_t)m->unexpected_fg+delta;
        uint32_t events=(uint32_t)(activity/(limit+1U));
        m->unexpected_fg=(uint32_t)(activity%(limit+1U));
        if(events) {
            uint32_t previous=m->recovery_events;
            if(!previous || (uint64_t)previous+events>=3) m->recovery_ms=now;
            m->recovery_events=(uint8_t)(1U+(previous ?
                ((uint64_t)previous+events-1U)%2U:(events-1U)%2U));
        }
    }
    return 0;
}
/**
 * @brief 窗口满5秒自动清故障4，保持停止/制动/位置无效；不恢复运动，不主动写 Flash。
 *
 * 条件：非手调、当前恰为故障4、有事件窗口且已过5000ms。
 * 恢复：清故障和事件计数，保持valid=0、duty=0、brake=1，状态置UNREADY并输出。
 * 限制：不自动设零/重放命令/写Flash，也不清其他故障或存储错误。
 */
static void fault4_recover(robot_t *r, unsigned i)
{
    robot_motor_t *m=&r->motor[i];
    if(!m->manual && m->fault==F_UNEXPECTED && m->recovery_events &&
       (uint32_t)(r->now_ms-m->recovery_ms)>=ROBOT_FAULT4_RECOVERY_MS) {
        recovery_reset(m); m->fault=F_NONE; m->valid=0;
        m->duty=0; m->brake=1; m->state=S_UNREADY; m->sign=0;
        output(r,i);
    }
}
/**
 * @brief 补齐位置并调用保存回调；成功更新 dirty，失败锁存 storage_error，返回 1/0。
 *
 * 输入：候选镜像p、dirty=0干净或1运行未保存；调用者负责静止等互锁。
 * 过程：补当前位置和请求序号，调用ops.save。
 * 返回：失败0并锁storage_error；成功1、清storage_error并更新r->dirty，不自动清pending_save。
 */
static int persist(robot_t *r, robot_image_t *p, unsigned dirty)
{
    image_positions(r,p);
    if(!r->ops.save(p,dirty)) { r->storage_error=1; return 0; }
    r->storage_error=0; r->dirty=(uint8_t)dirty;
    return 1;
}
/**
 * @brief 恢复合法配置和位置；dirty 恢复时零点无效，上电全部制动，不继续上次动作。
 *
 * 输入：ops为有效硬件回调，p可NULL，uid须3个32位字，now单位ms，dirty为上次未完整保存标志。
 * 过程：清RAM状态，使用合法p或默认值，复制staged及通信身份，恢复请求高水位。
 * 位置：dirty非0时所有有效零点失效；保存计数仍可用于诊断。
 * 输出：四路制动后设置PWM频率；缓存/手调/自检结果重置，不续跑旧运动。
 */
void robot_init(robot_t *r, const robot_ops_t *ops, const robot_image_t *p,
                int dirty, const uint32_t uid[3], uint32_t now)
{
    memset(r,0,sizeof(*r)); r->ops=*ops;
    if(p && robot_config_valid(p)) r->config=*p; else robot_defaults(&r->config);
    r->staged=r->config; r->dirty=(uint8_t)(dirty!=0);
    r->last_seq=(uint16_t)r->config.request_sequence; r->have_seq=(uint8_t)(r->config.request_seen==1);
    r->protocol=(uint8_t)r->config.protocol; r->address=(uint8_t)r->config.address;
    r->node=(uint8_t)r->config.node; r->now_ms=now; r->control_ms=now;
    memcpy(r->uid,uid,sizeof(r->uid));
    for(unsigned i=0;i<4;i++) {
        robot_motor_t *m=&r->motor[i];
        m->position=r->config.position[i]; m->target=m->position;
        m->valid=(uint8_t)(!dirty && r->config.valid[i]==1 && r->config.channel[i].gear!=0);
        m->state=m->valid ? S_IDLE:S_UNREADY; m->brake=1; m->last_edge_ms=now;
        output(r,i);
    }
    r->ops.frequency(r->config.pwm_hz);
}
/**
 * @brief 将有符号 FG 计数换为输出轴 0.1度；减速比为0返回0，结果溢出时饱和，channel须有效。
 *
 * 输入：通道0～3和有符号脉冲数；本函数不检查通道越界。
 * 公式：pulses*3600/(gear*6)，结果为输出轴0.1度，整数除法向0截断。
 * 返回：gear=0为0；超int32范围饱和。此函数不检查valid，调用者必须同时显示位置有效标志。
 */
int32_t robot_angle(const robot_t *r, unsigned i, int32_t pulses)
{
    uint32_t gear=r->config.channel[i].gear;
    if(!gear) return 0;
    int64_t a=(int64_t)pulses*3600/(gear*6);
    if(a>INT32_MAX) return INT32_MAX;
    if(a<INT32_MIN) return INT32_MIN;
    return (int32_t)a;
}
/**
 * @brief 单路 PWM清零并制动；有故障则位置失效，活动通道进入等待静止；不直接保存 Flash。
 *
 * 输入：fault=0表示不新增故障，complete=1表示预计正常到位。
 * 过程：duty0/brake1；故障时锁存并使零点失效；活动状态转SETTLING。
 * 手调：转manual=2，sign清0，始终不把退出当正常到位。
 * 限制：fault=0不会清已有故障；最终余转判断/保存由robot_update完成。
 */
static void stop(robot_t *r, unsigned i, unsigned fault, int complete)
{
    robot_motor_t *m=&r->motor[i];
    if(m->manual) {
        if(m->manual==1) m->last_edge_ms=r->now_ms;
        m->manual=2; m->sign=0; m->valid=0; complete=0;
    }
    m->duty=0; m->brake=1; m->completing=(uint8_t)complete;
    if(fault) {
        m->fault=(uint8_t)fault; m->valid=0;
        if(fault==F_UNEXPECTED) { recovery_reset(m); recovery_event(m,r->now_ms); }
    }
    if(busy(m)) m->state=S_SETTLING;
    else m->state=m->fault ? S_FAULT:(m->valid ? S_IDLE:S_UNREADY);
    output(r,i);
}

/**
 * @brief 中止批次中尚未试转的通道，标为未执行并记录原因和时间。
 *
 * 输入：reason为自检原因码；只处理ST_QUEUED项。
 * 结果：改为ST_UNTESTED并写完成时刻；已试转/已完成通道保留自身结果。
 */
static void probe_drop_queue(robot_t *r, uint8_t reason)
{
    for(unsigned i=0;i<4;i++) if(r->selftest[i].state==ST_QUEUED) {
        r->selftest[i].state=ST_UNTESTED; /* selected, but never energized */
        r->selftest[i].reason=reason;
        r->selftest[i].completed_ms=r->now_ms;
    }
}
/**
 * @brief 停止自检当前行程并记录通电时间；批量仅无/弱反馈允许继续，其余异常中止后续通道。
 *
 * 输入：terminal为最终通过/失败/取消，reason为原因；只针对当前probe.current。
 * 过程：记录当前行程通电时间，转对应SETTLING阶段，PWM0并制动；余转继续收集。
 * 批量：仅无脉冲/不足脉冲可继续扫描，其余失败和取消清后续队列。
 * 限制：等待阶段的取消可覆盖临时成功；保存并不在这里执行。
 */
static void probe_stop(robot_t *r, uint8_t terminal, uint8_t reason)
{
    robot_selftest_session_t *p=&r->probe;
    unsigned i=p->current;
    robot_motor_t *m=&r->motor[i];
    robot_selftest_result_t *result=&r->selftest[i];
    if(m->state==S_SELFTEST) {
        uint32_t elapsed=r->now_ms-p->started_ms;
        uint16_t duration=(uint16_t)(elapsed>UINT16_MAX ? UINT16_MAX:elapsed);
        if(result->phase==SP_RETURN) {
            result->return_ms=duration; result->phase=SP_RETURN_SETTLING;
        } else {
            result->powered_ms=duration; result->phase=SP_OUTBOUND_SETTLING;
        }
        p->stopped_ms=r->now_ms;
        result->state=ST_SETTLING;
        p->terminal_state=terminal;
        result->reason=reason;
    } else if(terminal!=ST_PASSED && !p->aborting) {
        /* A cancel/error during coast must override provisional success. */
        p->terminal_state=terminal; result->reason=reason;
    }
    if(terminal!=ST_PASSED) {
        m->valid=0;
        if(terminal==ST_FAILED) m->fault=F_SELFTEST;
        /* Missing/weak feedback is an expected discovery result. All safety
         * failures and cancellations still abort the remaining batch. */
        if(!p->scan_all || terminal!=ST_FAILED ||
           (reason!=ST_NO_PULSES && reason!=ST_TOO_FEW_PULSES)) {
            p->aborting=1;
            probe_drop_queue(r,reason);
        }
    }
    m->duty=0; m->brake=1; m->completing=0; m->state=S_SETTLING;
    output(r,i);
}
/**
 * @brief 取消自检并停止四路，手调转退出等待；不消耗上位机命令序号。
 *
 * 过程：自检活动时先取消当前批次，然后对四路执行stop。
 * 结果：PWM0、制动；手调退出等待，故障不自动清除。
 * 限制：不是硬件异常ISR入口，最终静止/保存要继续主循环。
 */
void robot_stop_all(robot_t *r)
{
    if(r->probe.active) probe_stop(r,ST_CANCELLED,ST_CANCEL);
    for(unsigned i=0;i<4;i++) stop(r,i,F_NONE,0);
}
/**
 * @brief 开始一路自检去程，从0占空比起步；记录原始位置、方向及3脉冲目标。
 *
 * 输入：i为下一待测通道0～3；会话已持有整板互锁且dirty已保存。
 * 过程：保存原位置，设置去程3脉冲目标，以negative XOR inverted决定DIR，释放刹车。
 * 输出：初始duty0，后续probe_update才递增；该步骤不写Flash。
 */
static void probe_begin_channel(robot_t *r, unsigned i)
{
    robot_selftest_session_t *p=&r->probe;
    robot_motor_t *m=&r->motor[i];
    p->current=(uint8_t)i; p->started_ms=r->now_ms; p->ramp_ms=r->now_ms;
    p->terminal_state=ST_PASSED;
    r->selftest[i].state=ST_RUNNING;
    r->selftest[i].phase=SP_OUTBOUND;
    p->origin_position=m->position; p->leg_target=ROBOT_SELFTEST_EDGES;
    m->sign=p->negative ? -1:1;
    m->direction=(uint8_t)(p->negative ^ r->config.channel[i].inverted);
    m->state=S_SELFTEST; m->duty=0; m->brake=0; m->completing=0;
    output(r,i);
}
/**
 * @brief 校验静止/故障/保存条件并先写 dirty；整板扫描包含未配置通道，单路要求启用及减速比。
 *
 * 输入：op=单路10或批量11；单路ch0～3；arg=0正向或1负向。
 * 前置：整板静止500ms、无存储错误、四路无故障；单路须已启用且gear非0。
 * 批量：直接选全部物理通道，包括未配置/禁用通道，按M1到M4试转。
 * 保存：本次序号和dirty必须先写成功；失败返回R_STORAGE且不释放任何通道。
 */
static uint16_t probe_start(robot_t *r, uint16_t op, uint16_t ch, int32_t arg)
{
    if(arg!=0 && arg!=1) return R_VALUE;
    if(op==CMD_SELFTEST && ch>=4) return R_VALUE;
    if(!quiet(r)) return R_BUSY;
    if(r->storage_error) return R_STORAGE;
    /* 批量扫描物理M1～M4，不依赖启用/减速比；任何现存通道故障会拒绝启动。 */
    uint8_t mask=0;
    for(unsigned i=0;i<4;i++) {
        if(r->motor[i].fault) return R_UNREADY;
        if(op==CMD_SELFTEST_ALL || (ch==i && r->config.channel[i].enabled &&
           r->config.channel[i].gear)) mask|=(uint8_t)(1U<<i);
    }
    if(!mask) return R_UNREADY;
    for(unsigned i=0;i<4;i++) {
        if(op!=CMD_SELFTEST_ALL && ch!=i) continue;
        memset(&r->selftest[i],0,sizeof(r->selftest[i]));
        r->selftest[i].sequence=r->last_seq;
        r->selftest[i].state=(mask&(1U<<i)) ? ST_QUEUED:ST_SKIPPED;
        if(r->selftest[i].state==ST_SKIPPED) r->selftest[i].completed_ms=r->now_ms;
    }
    /* Always write the current request sequence, even if boot restored dirty.
     * An un-zeroed channel may be probed but must not bypass replay protection. */
    if(!persist(r,&r->config,1)) {
        for(unsigned i=0;i<4;i++) if(mask&(1U<<i)) {
            r->selftest[i].state=ST_FAILED; r->selftest[i].reason=ST_STORAGE_ERROR;
            r->selftest[i].completed_ms=r->now_ms;
        }
        return R_STORAGE; /* No channel has moved; preserve existing validity. */
    }
    r->pending_save=0;
    memset(&r->probe,0,sizeof(r->probe));
    r->probe.active=1; r->probe.negative=(uint8_t)arg;
    r->probe.scan_all=(uint8_t)(op==CMD_SELFTEST_ALL);
    /* Non-test channels remain braked, including those previously released. */
    for(unsigned i=0;i<4;i++) { r->motor[i].duty=0; r->motor[i].brake=1; output(r,i); }
    for(unsigned i=0;i<4;i++) if(mask&(1U<<i)) { probe_begin_channel(r,i); break; }
    return R_OK;
}
/**
 * @brief 推进去程、余转、回程和批次；回程目标含去程余转，残差仅诊断，不自动纠正位置。
 *
 * 输入：整板同一时刻反馈f[4]，不能只更新被测一路。
 * 去程：累计3脉冲即停车；不足时2秒截止；停车后至少500ms且整板无FG才进入回程。
 * 回程：以去程含余转总数为目标、反向计数；残差只记录，不再作为失败条件。
 * 异常：捕获错、其他路超限、位置溢出、停止超时中止批次；弱反馈批量可继续。
 * 结束：最后一路静止后统一保存；保存失败单独报告，不伪造运动失败/成功已保存。
 */
static void probe_update(robot_t *r, const robot_feedback_t f[4])
{
    robot_selftest_session_t *p=&r->probe;
    unsigned current=p->current;
    robot_motor_t *m=&r->motor[current];
    robot_selftest_result_t *result=&r->selftest[current];
    /* Consume the entire feedback snapshot before deciding to advance. */
    for(unsigned i=0;i<4;i++) {
        robot_motor_t *other=&r->motor[i];
        uint32_t delta=f[i].count-other->count;
        int error=f[i].errors!=other->errors;
        other->count=f[i].count; other->errors=f[i].errors; other->rpm=f[i].rpm;
        if(delta) other->last_edge_ms=r->now_ms;
        if(i==current) {
            uint32_t *leg=(result->phase==SP_RETURN || result->phase==SP_RETURN_SETTLING)
                ? &result->return_pulses:&result->outbound_pulses;
            if(UINT32_MAX-*leg<delta) {
                *leg=UINT32_MAX; probe_stop(r,ST_FAILED,ST_POSITION_OVERFLOW);
            } else *leg+=delta;
            if(UINT32_MAX-result->pulses<delta) {
                result->pulses=UINT32_MAX; probe_stop(r,ST_FAILED,ST_POSITION_OVERFLOW);
            } else result->pulses+=delta;
            int64_t pos=(int64_t)m->position+(int64_t)m->sign*delta;
            if(pos<INT32_MIN || pos>INT32_MAX) probe_stop(r,ST_FAILED,ST_POSITION_OVERFLOW);
            else m->position=(int32_t)pos;
            int64_t residual=(int64_t)m->position-p->origin_position;
            if(residual<INT32_MIN || residual>INT32_MAX) {
                result->residual=residual<0 ? INT32_MIN:INT32_MAX;
                probe_stop(r,ST_FAILED,ST_POSITION_OVERFLOW);
            } else result->residual=(int32_t)residual;
            if(error) probe_stop(r,ST_FAILED,ST_CAPTURE_ERROR);
        } else {
            int unexpected=fault4_edges(other,delta,r->config.channel[i].gear,r->now_ms);
            if(!unexpected && !error) continue;
            probe_stop(r,ST_FAILED,unexpected ? ST_OTHER_ACTIVITY:ST_CAPTURE_ERROR);
            other->valid=0; stop(r,i,unexpected ? F_UNEXPECTED:F_CAPTURE,0);
            /* Identify the unexpected source even if it was not selected. */
            r->selftest[i].state=ST_FAILED;
            r->selftest[i].reason=unexpected ? ST_OTHER_ACTIVITY:ST_CAPTURE_ERROR;
            r->selftest[i].sequence=result->sequence;
            r->selftest[i].completed_ms=r->now_ms;
        }
    }
    if(m->state==S_SELFTEST) {
        uint32_t elapsed=r->now_ms-p->started_ms;
        uint32_t pulses=result->phase==SP_RETURN ? result->return_pulses:result->outbound_pulses;
        /* 自检优先判断脉冲到达，再判断2秒上限；回程不足报告回程超时。 */
        if(pulses>=p->leg_target) probe_stop(r,ST_PASSED,ST_REASON_NONE);
        else if(elapsed>=ROBOT_SELFTEST_POWER_MS)
            probe_stop(r,ST_FAILED,result->phase==SP_RETURN ? ST_RETURN_TIMEOUT:
                       (pulses ? ST_TOO_FEW_PULSES:ST_NO_PULSES));
        else if((uint32_t)(r->now_ms-p->ramp_ms)>=ROBOT_CONTROL_MS) {
            p->ramp_ms=r->now_ms;
            uint16_t duty=(uint16_t)(m->duty+ROBOT_SELFTEST_DUTY_STEP);
            m->duty=duty>ROBOT_SELFTEST_DUTY_MAX ? ROBOT_SELFTEST_DUTY_MAX:duty;
            output(r,current);
        }
    }
    if(m->state!=S_SETTLING) return;
    /* The hardware output callback can record a physical stop one tick after
     * this feedback snapshot. Do not treat that small negative age as wrap. */
    if((int32_t)(r->now_ms-p->stopped_ms)<0) return;
    int all_quiet=1;
    for(unsigned i=0;i<4;i++)
        if((uint32_t)(r->now_ms-r->motor[i].last_edge_ms)<ROBOT_QUIET_MS) all_quiet=0;
    if((uint32_t)(r->now_ms-p->stopped_ms)>=ROBOT_SELFTEST_STOP_MS &&
       !all_quiet) {
        /* Keep the motion/session lock until actual quiet is observed. */
        p->terminal_state=ST_FAILED; result->reason=ST_STOP_TIMEOUT;
        m->fault=F_SELFTEST; m->valid=0; p->aborting=1;
        p->stop_fault_mask|=(uint8_t)(1U<<current);
        probe_drop_queue(r,ST_STOP_TIMEOUT);
    }
    if((uint32_t)(r->now_ms-p->stopped_ms)<ROBOT_QUIET_MS ||
       (uint32_t)(r->now_ms-m->last_edge_ms)<ROBOT_QUIET_MS) return;
    /* All other channels must also be quiet; never erase/write while an
     * unexpected external motion is still producing pulses. */
    if(!all_quiet) return;
    if(!p->aborting && p->terminal_state==ST_PASSED && result->phase==SP_OUTBOUND_SETTLING) {
        /* Only after consuming all coast counts and confirming all four quiet.
         * The output callback atomically arms a fresh ISR baseline and target. */
        /* 去程余转也加入回程目标；这只按FG计数尝试返回，不保证机械位置无误差。 */
        p->leg_target=result->outbound_pulses;
        p->started_ms=p->ramp_ms=r->now_ms;
        result->phase=SP_RETURN; result->state=ST_RUNNING;
        m->sign=p->negative ? 1:-1;
        m->direction=(uint8_t)((p->negative^1U)^r->config.channel[current].inverted);
        m->state=S_SELFTEST; m->duty=0; m->brake=0;
        output(r,current); return;
    }
    /* Residual is diagnostic only; retain measured position without correcting it. */
    /* 残差保留实测计数差，不以残差超限判失败，也不把位置强行改为起点。 */
    result->state=p->terminal_state; result->completed_ms=r->now_ms;
    result->phase=SP_FINISHED;
    m->sign=0; m->state=m->fault ? S_FAULT:(m->valid ? S_IDLE:S_UNREADY);
    if(!p->aborting) {
        for(unsigned i=current+1;i<4;i++) if(r->selftest[i].state==ST_QUEUED) {
            probe_begin_channel(r,i); return; /* no intermediate Flash writes */
        }
    }
    p->active=0; r->pending_save=1;
    if(persist(r,&r->config,0)) r->pending_save=0;
    else {
        /* Keep a successful response result; storage has its own board flag. */
        if(result->reason==ST_REASON_NONE) result->reason=ST_STORAGE_ERROR;
    }
}
/**
 * @brief 主循环传入同一时刻的四路反馈快照和 ms 时钟；处理事件、闭环、停止确认及统一保存。
 *
 * 输入：now为单调可回绕ms，f[4]为一致的累计计数/速度/错误快照。
 * 顺序：自检独占则推进probe；否则处理捕获错/FG增量，再故障4恢复，再速度/角度控制。
 * 控制：约10ms一次，无补跑循环；超时优先于到位，受控余转仍计入当前位置。
 * 完成：静止500ms后检查5度容差，置DONE/FAULT等，满足条件再保存。
 * 限制：同一robot_t不能被多个线程/ISR并发推进，调用者须串行化。
 */
void robot_update(robot_t *r, uint32_t now, const robot_feedback_t f[4])
{
    r->now_ms=now;
    if(r->probe.active) {
        probe_update(r,f);
        for(unsigned i=0;i<4;i++) fault4_recover(r,i);
        return;
    }
    int control=(uint32_t)(now-r->control_ms)>=ROBOT_CONTROL_MS;
    if(control) r->control_ms=now;
    for(unsigned i=0;i<4;i++) {
        robot_motor_t *m=&r->motor[i];
        uint32_t delta=f[i].count-m->count; m->count=f[i].count;
        m->rpm=f[i].rpm;
        if(f[i].errors!=m->errors) { m->errors=f[i].errors; stop(r,i,F_CAPTURE,0); r->pending_save=1; }
        /* 单线FG不提供方向；仅受控运动/余转按指令方向积分，手拖不更新有符号位置。 */
        if(delta) {
            m->last_edge_ms=now;
            if(m->manual) {
                /* One FG wire cannot determine the direction of a human drag. */
            } else if(busy(m) && m->sign) {
                int64_t pos=(int64_t)m->position+(int64_t)m->sign*delta;
                if(pos>INT32_MAX || pos<INT32_MIN) stop(r,i,F_POSITION,0);
                else m->position=(int32_t)pos;
            } else if(fault4_edges(m,delta,r->config.channel[i].gear,now)) {
                stop(r,i,F_UNEXPECTED,0); r->pending_save=1;
                recovery_reset(m); recovery_event(m,now);
            }
        }
        /* 先处理本周期FG事件再检查5秒结束，避免边界处第3次事件被提前清除。 */
        fault4_recover(r,i);
        if(m->state==S_RUNNING || m->state==S_DECEL) {
            int64_t remaining=((int64_t)m->target-m->position)*m->sign;
            uint32_t gear=r->config.channel[i].gear;
            int64_t ddegrees=remaining*3600/(6*gear);
            /* 正常运动：超过1秒无新FG触发故障1；与5秒故障4自动恢复规则相互独立。 */
            if((uint32_t)(now-m->last_edge_ms)>ROBOT_FEEDBACK_MS) stop(r,i,F_FEEDBACK,0);
            else if((uint32_t)(now-m->started_ms)>m->timeout_ms) stop(r,i,F_TIMEOUT,0);
            else if(remaining<=0) stop(r,i,F_NONE,1);
            else if(control) {
                uint32_t desired=r->config.channel[i].rpm;
                /* 剩余小于30度即线性降至400RPM；短距离命令可能从第一周期就进入减速区。 */
                if(ddegrees<ROBOT_DECEL_DDEG) {
                    m->state=S_DECEL;
                    desired=ROBOT_CRAWL_RPM+(uint32_t)ddegrees*(desired-ROBOT_CRAWL_RPM)/ROBOT_DECEL_DDEG;
                }
                /* 转速前馈 + 比例误差 + 有限积分；单位为减速前RPM，输出为低电平千分比。 */
                int32_t error=(int32_t)desired-(int32_t)m->rpm;
                m->integral+=error/40;
                if(m->integral<0) m->integral=0;
                if(m->integral>700) m->integral=700;
                int32_t demand=(int32_t)desired/16+error/32+m->integral;
                if(demand<0) demand=0;
                if(demand>(int32_t)ROBOT_DUTY_MAX) demand=ROBOT_DUTY_MAX;
                /* Bounded slew, including during acceleration. */
                /* 每个控制周期仅限制上升不超过20千分点；下降可立即响应，不是对称斜率限制。 */
                if(demand>(int32_t)m->duty+20) demand=m->duty+20;
                m->duty=(uint16_t)demand; output(r,i);
            }
        }
        /* 制动后仍收集余转；连续500ms无FG才判断最终到位误差和安排保存。 */
        if(m->state==S_SETTLING && (uint32_t)(now-m->last_edge_ms)>=ROBOT_QUIET_MS) {
            int64_t error=(int64_t)m->position-m->target;
            if(error<0) error=-error;
            if(m->completing && error*3600>(int64_t)ROBOT_TOLERANCE_DDEG*6*r->config.channel[i].gear)
                { m->fault=F_POSITION; m->valid=0; }
            m->state=m->fault ? S_FAULT:(m->completing ? S_DONE:(m->valid ? S_IDLE:S_UNREADY));
            m->sign=0; m->manual=0; r->pending_save=1;
        }
    }
    /* 自动保存仅在静止且没有锁存存储错误时尝试；失败后需显式SAVE等恢复操作。 */
    if(r->pending_save && quiet(r) && !r->storage_error) {
        if(persist(r,&r->config,0)) r->pending_save=0;
    }
}
/**
 * @brief 执行新命令的实际互锁和业务操作，返回 R_*；此层不负责请求序号去重。
 *
 * 输入：op为命令号，ch为物理0～3，arg随命令解释；由robot_command调用。
 * 顺序：STOP_ALL优先；处理手调进入退出；手调/停机输入/自检互锁后执行其他操作。
 * 返回：R_OK仅表示本次请求被接受；MOVE与自检仍由后续update完成。
 * 限制：此函数不检查重复序号；不可绕过robot_command直接对外开放。
 */
static uint16_t execute(robot_t *r, uint16_t op, uint16_t ch, int32_t arg)
{
    /*
     * 命令5：整板停止，优先于后面的手调/自检等互锁。
     * 不检查ch/arg；终止自检和手调，四路PWM停止并制动。
     * 已有故障不清除，余转及保存仍要通过后续主循环完成。
     */
    if(op==CMD_STOP_ALL) { robot_stop_all(r); return R_OK; }
    /*
     * 命令12：arg=1进入手调，arg=0退出，ch必须0～3。
     * 退出立即制动，连续静止500ms后关闭模式；重复进入已在手调的通道直接成功。
     * 进入要求无停机输入/自检/停止超时互锁/存储错误；目标启用且减速比有效。
     * 四路需无非手调忙状态并静止500ms；目标只容许无故障或故障4。
     * 首路释放前保存dirty，位置立即无效；不改DIR，不按旧方向积分人工FG。
     */
    if(op==CMD_MANUAL_ADJUST) {
        if(ch>=4 || (arg!=0 && arg!=1)) return R_VALUE;
        robot_motor_t *m=&r->motor[ch];
        if(!arg) { if(m->manual==1) stop(r,ch,F_NONE,0); return R_OK; }
        if(m->manual==1) return R_OK;
        if(r->stop_input || r->probe.active) return R_BUSY;
        if(r->probe.stop_fault_mask) return R_UNREADY;
        if(r->storage_error) return R_STORAGE;
        const robot_config_t *c=&r->config.channel[ch];
        if(!c->enabled || !c->gear || (m->fault && m->fault!=F_UNEXPECTED)) return R_UNREADY;
        for(unsigned i=0;i<4;i++) {
            const robot_motor_t *other=&r->motor[i];
            if(other->manual==2 || (!other->manual && busy(other)) ||
               (uint32_t)(r->now_ms-other->last_edge_ms)<ROBOT_QUIET_MS) return R_BUSY;
        }
        /* 先让拟保存镜像中的零点失效，再写dirty；写失败恢复原有效标志且不释放刹车。 */
        uint8_t previous_valid=m->valid; m->valid=0;
        if(!manual_any(r) && !persist(r,&r->config,1)) {
            m->valid=previous_valid; return R_STORAGE;
        }
        /* 只有已有故障4允许进入手调时自动清除；手调没有自动倒计时退出。 */
        recovery_reset(m); m->fault=F_NONE; m->manual=1; m->state=S_MANUAL;
        m->sign=0; m->completing=0; m->duty=0; m->brake=0; output(r,ch); return R_OK;
    }
    /* 任一路处于手调/退出等待时，整板只放行这里明确列出的停机操作。 */
    if(manual_any(r) && op!=CMD_STOP && !(op==CMD_BRAKE && arg==1)) return R_BUSY;
    if(r->stop_input && (op==CMD_MOVE || op==CMD_SELFTEST || op==CMD_SELFTEST_ALL ||
       op==CMD_DIRECTION || (op==CMD_BRAKE && arg==0))) return R_BUSY;
    if(r->probe.active) {
        if(ch<4 && (op==CMD_STOP || (op==CMD_BRAKE && arg==1))) {
            probe_stop(r,ST_CANCELLED,ST_CANCEL); stop(r,ch,F_NONE,0); return R_OK;
        }
        return R_BUSY;
    }
    /* A stop-confirmation timeout is a board-wide motion interlock, even
     * after quiet eventually returns. Acknowledge it only while quiet. */
    if(op==CMD_MOVE || op==CMD_DIRECTION || (op==CMD_BRAKE && arg==0)) {
        if(r->probe.stop_fault_mask) return R_UNREADY;
    }
    if(op==CMD_SELFTEST || op==CMD_SELFTEST_ALL) return probe_start(r,op,ch,arg);
    /*
     * 命令8：显式保存当前生效配置及所有位置，不提交staged草稿。
     * 整板quiet后才执行；允许用于重试锁存的storage_error。
     * 保存成功清pending_save和dirty；失败返回R_STORAGE，不能显示已持久化。
     */
    if(op==CMD_SAVE) {
        if(!quiet(r)) return R_BUSY;
        if(!persist(r,&r->config,0)) return R_STORAGE;
        r->pending_save=0; return R_OK;
    }
    /*
     * 命令1：验证并保存整份staged配置；不是单路提交，ch/arg在本分支不使用。
     * gear/role/inverted变化使该路零点失效，enabled单独变化不主动清valid。
     * enabled/gear/role/inverted任意变化会清该路历史自检结果；只改RPM不清自检。
     * 保存失败不暴露新配置；成功更新PWM频率，通信协议/地址/node下次重启才生效。
     */
    if(op==CMD_CONFIG) {
        if(!quiet(r)) return R_BUSY;
        if(!robot_config_valid(&r->staged)) return R_VALUE;
        /* 配置事务先保存候选镜像，成功后才替换生效配置；协议/地址/node仍须重启加载。 */
        robot_image_t next=r->staged;
        uint8_t changed=0;
        image_positions(r,&next);
        for(unsigned i=0;i<4;i++) {
            const robot_config_t *a=&r->config.channel[i],*b=&next.channel[i];
            if(a->gear!=b->gear || a->role!=b->role || a->inverted!=b->inverted) next.valid[i]=0;
            if(a->enabled!=b->enabled || a->gear!=b->gear || a->role!=b->role || a->inverted!=b->inverted)
                changed|=(uint8_t)(1U<<i);
        }
        /* Commit the candidate before exposing any changed configuration. */
        if(!r->ops.save(&next,0)) { r->storage_error=1; return R_STORAGE; }
        r->config=next; r->staged=next; r->dirty=0; r->pending_save=0; r->storage_error=0;
        for(unsigned i=0;i<4;i++) {
            if(changed&(1U<<i)) memset(&r->selftest[i],0,sizeof(r->selftest[i]));
            r->motor[i].valid=(uint8_t)next.valid[i];
            r->motor[i].state=r->motor[i].fault ? S_FAULT:(next.valid[i] ? S_IDLE:S_UNREADY);
        }
        r->ops.frequency(next.pwm_hz); return R_OK;
    }
    if(ch>=4) return R_VALUE;
    robot_motor_t *m=&r->motor[ch];
    const robot_config_t *c=&r->config.channel[ch];
    /*
     * 命令4：停止指定ch；PWM0并制动，不清故障、不设零。
     * 若原来忙，先进入SETTLING确认余转，确认完成后由更新路径安排统一保存。
     */
    if(op==CMD_STOP) { stop(r,ch,F_NONE,0); return R_OK; }
    /*
     * 命令6：arg=1锁刹车，调用统一stop；arg=0释放。
     * 释放要求目标非忙且无故障，还受前面的整板互锁/停机输入限制。
     * 释放只是改brake标志，不启用手调、不暂停故障4、不自动使零点失效。
     */
    if(op==CMD_BRAKE) {
        if(arg!=0 && arg!=1) return R_VALUE;
        if(arg) stop(r,ch,F_NONE,0);
        else { if(busy(m) || m->fault) return R_BUSY; m->brake=0; output(r,ch); }
        return R_OK;
    }
    /*
     * 命令7：arg是物理逻辑电平0/1，不是相对角度，也不异或inverted。
     * 要求该路非忙且距上次FG至少500ms；前面的整板互锁仍适用。
     * 只改变direction并输出；下次MOVE重新按角度符号和inverted覆盖它。
     */
    if(op==CMD_DIRECTION) {
        if(arg!=0 && arg!=1) return R_VALUE;
        if(busy(m) || (uint32_t)(r->now_ms-m->last_edge_ms)<ROBOT_QUIET_MS) return R_BUSY;
        m->direction=(uint8_t)arg; output(r,ch); return R_OK;
    }
    /*
     * 命令9：整板quiet才允许确认指定通道故障。
     * 清该通道停止超时互锁位、故障4计数和fault；根据原valid回到IDLE/UNREADY。
     * 不恢复失效零点、不清storage_error，不直接写Flash。
     */
    if(op==CMD_CLEAR_FAULT) {
        if(!quiet(r)) return R_BUSY;
        r->probe.stop_fault_mask&=(uint8_t)~(1U<<ch);
        recovery_reset(m);
        m->fault=0; m->state=m->valid ? S_IDLE:S_UNREADY; return R_OK;
    }
    /* 显式设零才建立可信参考；仅清故障或完成相对运动不会自动设零。 */
    /*
     * 命令2：把该通道当前机械姿态定义为0，需要上位机/用户确认基准。
     * 整板quiet且该路gear非0；先用候选镜像保存position=0、valid=1。
     * 保存成功后才更新RAM位置和目标、清该路故障/故障4统计/停止超时互锁。
     * 这不是自动回到机械零点，不产生运动；保存失败不把RAM位置伪装成已设零。
     */
    if(op==CMD_ZERO) {
        if(!quiet(r)) return R_BUSY;
        if(!c->gear) return R_UNREADY;
        robot_image_t next=r->config; image_positions(r,&next);
        next.position[ch]=0; next.valid[ch]=1;
        if(!r->ops.save(&next,0)) { r->storage_error=1; return R_STORAGE; }
        r->config=next; r->dirty=0; r->pending_save=0; r->storage_error=0;
        r->probe.stop_fault_mask&=(uint8_t)~(1U<<ch);
        recovery_reset(m);
        m->position=0; m->target=0; m->valid=1; m->fault=0; m->state=S_IDLE; return R_OK;
    }
    /*
     * 命令3：参数arg为有符号输出轴0.1度相对位移，仅接受-3600～3600。
     * 目标=当前FG位置+舍入后的位移脉冲；本固件入口不提供绝对角度MOVE。
     * 要求目标无故障、已启用、gear有效、静止500ms和无存储错误；不强制valid为1。
     * dirty未置位时先保存dirty，成功才释放刹车；未知零点的相对运动不会自动变有效。
     * arg=0在前置目标检查后直接成功，不实际起动；非零运动最终结果要读状态。
     */
    if(op!=CMD_MOVE) return R_VALUE;
    if(arg < -3600 || arg > 3600) return R_VALUE;
    if(busy(m)) return R_BUSY;
    /* MOVE is relative to the current counter, not an absolute zero.
     * An unknown zero remains unknown through motion and persistence. Faults
     * still require explicit acknowledgement; never reset/rebase here. */
    if(m->fault || !c->enabled || !c->gear) return R_UNREADY;
    if((uint32_t)(r->now_ms-m->last_edge_ms)<ROBOT_QUIET_MS) return R_BUSY;
    if(!arg) return R_OK;
    if(r->storage_error) return R_STORAGE;
    /* 输出轴0.1度换算减速前FG数；加减半分母实现正负对称舍入，64位中间值避免溢出。 */
    int64_t steps=((int64_t)arg*c->gear*6+(arg>0 ? 1800:-1800))/3600;
    int64_t target=(int64_t)m->position+steps;
    if(target<INT32_MIN || target>INT32_MAX) return R_VALUE;
    if(!r->dirty) {
        if(!quiet(r)) return R_BUSY;
        if(!persist(r,&r->config,1)) return R_STORAGE;
    }
    m->target=(int32_t)target; m->sign=(arg>0 ? 1:-1);
    /* 每次MOVE根据角度符号与方向反相重设DIR，覆盖此前独立DIR命令。 */
    m->direction=(uint8_t)((arg<0)^c->inverted);
    m->state=S_RUNNING; m->brake=0; m->duty=0; m->integral=0; m->completing=0;
    /* 无反馈保护从命令起动计时，当前没有独立的首脉冲启动宽限时间。 */
    m->started_ms=r->now_ms; m->last_edge_ms=r->now_ms;
    /* Generous finite timeout; not an arrival-time guarantee. */
    uint32_t distance=(uint32_t)(arg<0 ? -arg:arg);
    m->timeout_ms=10000U+(uint32_t)((uint64_t)distance*c->gear*60000U/(3600U*c->rpm))*4U;
    output(r,ch); return R_OK;
}
/**
 * @brief seq=0分配新序号；非零序号查询16项缓存避免重执行，同序号不同内容拒绝；结果表示接受情况。
 *
 * 输入：seq非零用于重试去重，0表示每次分配新身份；op/ch/arg必须保持重试内容一致。
 * 去重：16项RAM缓存命中返回原结果；相同seq不同参数、陈旧/相同高水位拒绝。
 * 执行：先更新请求序号，再execute，并缓存结果；未接受的业务结果也可缓存。
 * 重启：缓存丢失，保存的序号用于阻止旧请求重执行；不能把seq=0用作可靠重发。
 */
uint16_t robot_command(robot_t *r,uint16_t seq,uint16_t op,uint16_t ch,int32_t arg)
{
    /* 零序号每次分配新请求，重复发送可能重复执行；需要重试去重时使用同一非零序号。 */
    if(seq==ROBOT_AUTO_SEQUENCE) {
        seq=r->have_seq ? (uint16_t)(r->last_seq+1U):1U;
        if(!seq) seq=1;
        /* Explicit forward jumps can leave small IDs cached across wrap.
         * Never replay one of them for an automatic request (at most 16 IDs). */
        for(unsigned i=0;i<r->cache_used;) {
            if(r->cache[i].seq==seq) {
                seq=(uint16_t)(seq+1U); if(!seq) seq=1;
                i=0;
            } else i++;
        }
    }
    /* 缓存保存接受/拒绝结果，重复请求返回原结果，不会再次执行运动。 */
    for(unsigned i=0;i<r->cache_used;i++) if(r->cache[i].seq==seq) {
        if(r->cache[i].op!=op || r->cache[i].ch!=ch || r->cache[i].arg!=arg) return R_SEQUENCE;
        return r->cache[i].result;
    }
    if(r->have_seq && (uint16_t)(seq-r->last_seq)>=0x8000U) return R_SEQUENCE;
    if(r->have_seq && seq==r->last_seq) return R_SEQUENCE;
    /* Persist the sequence along with the pre-move dirty record. Reboot cannot
     * re-execute an already admitted move even after the RAM cache is lost. */
    r->last_seq=seq; r->have_seq=1;
    uint16_t result=execute(r,op,ch,arg);
    unsigned i=r->cache_next; r->cache_next=(uint8_t)((i+1)%16);
    if(r->cache_used<16) r->cache_used++;
    r->cache[i].seq=seq; r->cache[i].op=op; r->cache[i].ch=ch;
    r->cache[i].arg=arg; r->cache[i].result=result;
    r->last_seq=seq; r->last_result=result; r->have_seq=1;
    return result;
}
/**
 * @brief 读取单个寄存器；通道基址 0x100+i*0x40，32位字段高16位在前，未定义地址返回 R_ADDRESS。
 *
 * 输入：a为16位寄存器地址，v须可写；先置0，再映射字段。
 * 布局：全局0x0000起，通道0x0100+i*0x40；配置、实时状态、自检与手调分区。
 * 编码：32位数高寄存器在前，负数保留补码；角度0.1度和FG计数不能混用。
 * 限制：无效地址返回R_ADDRESS；原始角度仍需结合valid，不自动归零。
 */
static int read_one(const robot_t *r,uint16_t a,uint16_t *v)
{
    *v=0;
    /* 全局区含实际协议、存储标志、UID及能力；0x20为配置草稿，0x36/37为最近命令结果。 */
    /*
     * 寄存器分区：0x0000 schema；1～4当前通信/PWM；5～7保存状态；8～D UID；E有序号；F协议版本；10能力。
     * 0x20～23全局配置暂存；0x30～34命令回读；0x35只读0；0x36/37最近序号/结果。
     * 通道0x100+0x40*i：+00～04生效配置，+08～0C草稿，+10～16状态/故障/valid/brake/DIR/duty/RPM。
     * +18/19角度0.1度，+1A/1B有符号FG，+1C/1D目标0.1度；+20～2F自检，+30手调状态。
     */
    if(a==0) *v=ROBOT_SCHEMA;
    else if(a==1) *v=r->protocol;
    else if(a==2) *v=r->address;
    else if(a==3) *v=r->node;
    else if(a==4) *v=(uint16_t)r->config.pwm_hz;
    else if(a==5) *v=r->dirty;
    else if(a==6) *v=r->storage_error;
    else if(a==7) *v=r->pending_save;
    else if(a>=8 && a<14) *v=(uint16_t)(r->uid[(a-8)/2]>>((a&1) ? 0:16));
    else if(a==14) *v=r->have_seq;
    else if(a==15) *v=ROBOT_COMMAND_PROTOCOL;
    else if(a==0x10) *v=ROBOT_CAPABILITIES;
    else if(a>=0x20 && a<0x24) {
        const uint32_t b[4]={r->staged.protocol,r->staged.address,r->staged.node,r->staged.pwm_hz}; *v=(uint16_t)b[a-0x20];
    } else if(a>=0x30 && a<0x35) *v=r->command[a-0x30];
    else if(a==0x35) *v=0;
    else if(a==0x36) *v=r->last_seq;
    else if(a==0x37) *v=r->last_result;
    else if(a>=0x100 && a<0x200) {
        unsigned i=(a-0x100)/0x40, off=(a-0x100)%0x40;
        const robot_motor_t *m=&r->motor[i];
        const robot_config_t *c=(off>=8 && off<=12) ? &r->staged.channel[i]:&r->config.channel[i];
        uint32_t cfg[5]={c->enabled,c->role,c->gear,c->inverted,c->rpm};
        /* 每路+0～4为生效配置，+8～12为暂存，+0x10起为状态；角度是0.1度，位置是FG数。 */
        if(off<5) *v=(uint16_t)cfg[off];
        else if(off>=8 && off<=12) *v=(uint16_t)cfg[off-8];
        else if(off==0x30) *v=m->manual;
        else if(off==16) *v=m->state;
        else if(off==17) *v=m->fault;
        else if(off==18) *v=m->valid;
        else if(off==19) *v=m->brake;
        else if(off==20) *v=m->direction;
        else if(off==21) *v=m->duty;
        else if(off==22) *v=(uint16_t)m->rpm;
        else if(off>=24 && off<=29) {
            uint32_t n=(uint32_t)(off<26 ? robot_angle(r,i,m->position):(off<28 ? m->position:robot_angle(r,i,m->target)));
            *v=(uint16_t)(n>>((off&1) ? 0:16));
        } else if(off>=0x20 && off<=0x2F) {
            const robot_selftest_result_t *t=&r->selftest[i];
            switch(off) {
            case 0x20:*v=t->state;break;
            case 0x21:*v=t->reason;break;
            case 0x22:*v=(uint16_t)(t->pulses>>16);break;
            case 0x23:*v=(uint16_t)t->pulses;break;
            case 0x24:*v=t->powered_ms;break;
            case 0x25:*v=(uint16_t)(t->completed_ms>>16);break;
            case 0x26:*v=(uint16_t)t->completed_ms;break;
            case 0x27:*v=t->sequence;break;
            case 0x28:*v=t->phase;break;
            case 0x29:*v=t->return_ms;break;
            case 0x2A:*v=(uint16_t)(t->outbound_pulses>>16);break;
            case 0x2B:*v=(uint16_t)t->outbound_pulses;break;
            case 0x2C:*v=(uint16_t)(t->return_pulses>>16);break;
            case 0x2D:*v=(uint16_t)t->return_pulses;break;
            case 0x2E:*v=(uint16_t)((uint32_t)t->residual>>16);break;
            case 0x2F:*v=(uint16_t)t->residual;break;
            }
        } else return R_ADDRESS;
    } else return R_ADDRESS;
    return R_OK;
}
/**
 * @brief 连续读取1～125个寄存器到调用方缓冲；遇未定义地址返回错误，缓冲可能已填部分数据。
 *
 * 输入：a起始地址，n=1～125，v至少n个uint16元素。
 * 过程：先防地址溢出，再逐项read_one；任一项不合法立即返回错误。
 * 限制：失败可能已填部分缓冲，但协议层不会把它当正常完整响应发送。
 */
int robot_read(const robot_t *r,uint16_t a,uint16_t n,uint16_t *v)
{
    if(!n || n>125 || (uint32_t)a+n>65536) return R_ADDRESS;
    for(unsigned i=0;i<n;i++) { int e=read_one(r,(uint16_t)(a+i),&v[i]); if(e) return e; }
    return R_OK;
}
/**
 * @brief 只允许全局暂存区及各路配置暂存区；完整5字命令由 robot_write 单独识别。
 *
 * 输入：待写地址a；返回是否属于配置暂存字段。
 * 范围：全局0x20～23，通道基址+8～12；其余不可由普通配置写路径修改。
 * 例外：完整5字命令0x30由robot_write更早分支处理，不在此白名单。
 */
static int writable(uint16_t a)
{
    if(a>=0x20 && a<0x24) return 1;
    return a>=0x100 && a<0x200 && (a-0x100)%0x40>=8 && (a-0x100)%0x40<=12;
}
/**
 * @brief 0x30起连续5字原子提交即执行；其余合法写入仅改 staged，旧0x35提交帧不再可写。
 *
 * 输入：v为n个寄存器，完整命令必须a=0x30且n=5。
 * 命令：字段为seq/op/ch/arg高16/arg低16，验证交robot_command，收到即执行，无A55A提交。
 * 配置：先检查整个区间都可写，再只改staged；需要CMD_CONFIG才保存并应用。
 * 限制：部分命令写及旧0x35写拒绝；普通暂存成功不表示值已通过完整配置校验。
 */
int robot_write(robot_t *r,uint16_t a,uint16_t n,const uint16_t *v)
{
    /* Atomic command only: partial writes and the old commit register are
     * read-only. No action occurs until the complete request is validated. */
    if(a==0x30 && n==5) {
        memcpy(r->command,v,sizeof(r->command));
        uint32_t arg=((uint32_t)v[3]<<16)|v[4];
        return robot_command(r,v[0],v[1],v[2],(int32_t)arg);
    }
    if(!n || n>123 || (uint32_t)a+n>65536) return R_ADDRESS;
    /* 先验证整个写区间再修改，避免跨只读地址时只写入半份配置。 */
    for(unsigned i=0;i<n;i++) if(!writable((uint16_t)(a+i))) return R_ADDRESS;
    for(unsigned i=0;i<n;i++,a++) {
        if(a==0x20) r->staged.protocol=v[i];
        else if(a==0x21) r->staged.address=v[i];
        else if(a==0x22) r->staged.node=v[i];
        else if(a==0x23) r->staged.pwm_hz=v[i];
        else {
            robot_config_t *c=&r->staged.channel[(a-0x100)/0x40];
            switch((a-0x100)%0x40) {
            case 8:c->enabled=v[i];break; case 9:c->role=v[i];break;
            case 10:c->gear=v[i];break; case 11:c->inverted=v[i];break; case 12:c->rpm=v[i];break;
            }
        }
    }
    return R_OK;
}
