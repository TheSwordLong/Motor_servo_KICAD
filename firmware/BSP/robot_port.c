/*
 * robot_port.c — GD32 硬件适配层：连接状态机、FG、GPIO、Flash、USART0 Modbus 与 CAN。
 * 主循环推进业务；SysTick 自检保护可先切断输出并锁存反馈，随后主循环消费该快照。
 * 普通协议处理不在 ISR 内执行，Flash 仅在禁止运动且反馈稳定时写入。
 */
#include "robot_port.h"
#include "robot.h"
#include "robot_store.h"
#include "robot_protocol.h"
#include "robot_rtu.h"
#include "gd32f30x.h"
#include "systick.h"
#include "capture.h"
#include "pwm.h"
#include "motor_io.h"
#include "motor_pins.h"
#include "usart0.h"
#include "can_bus.h"
#include "board_config.h"
#include <string.h>

robot_t g_robot;
static robot_store_t store;
static robot_rtu_t rtu;
static uint8_t initialized,can_ready;
static uint8_t output_dir[4],output_brake[4];
static uint8_t can_reply[8],can_pending,can_mailbox=CAN_NOMAILBOX;
static uint32_t can_started,rx_overflow;
/* Owned by main with IRQ masking; ISR may only latch a stop. */
/* 主循环关中断修改此共享状态；ISR只紧急停止和锁存，不直接推进完整状态机。 */
static volatile uint8_t probe_armed,probe_latched,probe_channel;
static volatile uint32_t probe_started,probe_stopped,probe_target;
static volatile robot_feedback_t probe_origin[4],probe_cutoff[4];
/* Fixed at arming: ISR does not race main's cumulative counter updates. */
static volatile uint32_t probe_unexpected_allowance[4];

/**
 * @brief SysTick 自检保护：达到脉冲目标/通电时限或捕获异常先切断输出，锁存快照供主循环处理。
 *
 * 上下文：SysTick；只在probe_armed时工作，避免主循环堵塞使自检无限通电。
 * 检查：2秒限时、当前路目标脉冲、任一路捕获错误、其他路剩余非预期额度。
 * 动作：PWM0/BRAKE1，保存cutoff反馈及停止时刻，清armed并置latched。
 * 限制：不解析命令、不写Flash；失败原因和回程由主循环决定。
 */
void robot_selftest_safety_tick(void)
{
    if(!probe_armed) return;
    unsigned ch=probe_channel;
    int halt=(uint32_t)(sys_tick_ms-probe_started)>=ROBOT_SELFTEST_POWER_MS;
    robot_feedback_t f[4];
    for(unsigned i=0;i<4;i++) {
        f[i].count=g_fg[i].pulse_count; f[i].errors=g_fg[i].errors;
        f[i].rpm=capture_get_speed_rpm((uint8_t)i);
        if(f[i].errors!=probe_origin[i].errors) halt=1;
        if(i==ch) {
            if((uint32_t)(f[i].count-probe_origin[i].count)>=probe_target) halt=1;
        } else if((uint32_t)(f[i].count-probe_origin[i].count)>
                  probe_unexpected_allowance[i]) halt=1;
    }
    if(!halt) return;
    pwm_set_permille((uint8_t)ch,0); motor_set_brake((uint8_t)ch,1);
    output_brake[ch]=1;
    for(unsigned i=0;i<4;i++) probe_cutoff[i]=f[i];
    probe_stopped=sys_tick_ms; probe_armed=0; probe_latched=1;
}

/**
 * @brief 读取 SysTick 生成可回绕的32位 us 时间戳；短暂屏蔽中断并补偿挂起节拍。
 *
 * 过程：保存PRIMASK、关中断读取ms和SysTick计数；节拍挂起时补1ms。
 * 返回：32位微秒，约71.6分钟回绕，调用者使用无符号时间差。
 * 限制：依赖SysTick和SystemCoreClock已初始化，不用于Reset_Handler早期。
 */
uint32_t robot_micros(void)
{
    uint32_t key=__get_PRIMASK(); __disable_irq();
    uint32_t ms=sys_tick_ms, val=SysTick->VAL;
    if(SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) { ms++; val=SysTick->VAL; }
    __set_PRIMASK(key);
    return ms*1000U+(SysTick->LOAD-val)/(SystemCoreClock/1000000U);
}
/**
 * @brief 复位早期/异常路径强制 PWM高、BRAKE低，不依赖全局初始化或改变DIR；复位保持期间无法控制输出。
 *
 * 调用：Reset_Handler在SystemInit前，以及异常停机路径。
 * 过程：启用所需GPIO，PWM四路改推挽高，BRAKE低制动，DIR保持原值。
 * 限制：不访问未初始化业务全局、不依赖浮点/计时；重置为GPIO后正常PWM需重新初始化。
 */
void robot_emergency_stop(void)
{
    /* Also called by Reset_Handler before SystemInit: keep this integer-only,
     * independent of clocks/timers/global initialization, and leave DIR alone.
     * It cannot control the pins before the CPU leaves hardware reset. */
    rcu_periph_clock_enable(RCU_GPIOD);
    gpio_bit_set(GPIOD,PWM_GPIO_PINS);
    gpio_init(GPIOD,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,PWM_GPIO_PINS);
    rcu_periph_clock_enable(RCU_GPIOA); rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOE);
    /* Do not reverse a moving motor while stopping after a CPU fault. */
    gpio_bit_reset(M1_BRAKE_PORT,M1_BRAKE_PIN);
    gpio_bit_reset(M2_BRAKE_PORT,M2_BRAKE_PIN);
    gpio_bit_reset(M3_BRAKE_PORT,M3_BRAKE_PIN);
    gpio_bit_reset(M4_BRAKE_PORT,M4_BRAKE_PIN);
    gpio_init(M1_BRAKE_PORT,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,M1_BRAKE_PIN);
    gpio_init(M2_BRAKE_PORT,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,M2_BRAKE_PIN);
    gpio_init(M3_BRAKE_PORT,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,M3_BRAKE_PIN);
    gpio_init(M4_BRAKE_PORT,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,M4_BRAKE_PIN);
}
/**
 * @brief 原子应用单路输出并维护自检保护；改 DIR/BRAKE 前先 PWM停止，没有额外方向建立延时。
 *
 * 输入：i有效通道、duty千分比、dir物理电平选择、brake逻辑制动标志。
 * 过程：关中断协调自检armed/latched，先PWM0再写DIR/BRAKE，最后brake?0:duty。
 * 自检：实际释放时重设计时基准；已被ISR截停时不得重新放行输出。
 * 限制：没有方向建立延时，也不测端子电压；output_dir/brake只是输出缓存。
 */
static void drive(uint8_t i,uint16_t duty,uint8_t dir,uint8_t brake)
{
    if(!initialized) return;
    uint32_t key=__get_PRIMASK(); __disable_irq();
    if(g_robot.probe.active && g_robot.probe.current==i) {
        if(brake) {
            if(probe_armed) {
                uint32_t elapsed=sys_tick_ms-probe_started;
                g_robot.probe.stopped_ms=sys_tick_ms;
                uint16_t duration=(uint16_t)(elapsed>UINT16_MAX ? UINT16_MAX:elapsed);
                if(g_robot.selftest[i].phase==SP_RETURN_SETTLING)
                    g_robot.selftest[i].return_ms=duration;
                else g_robot.selftest[i].powered_ms=duration;
            }
            probe_armed=0;
        }
        else if(probe_latched) { duty=0; brake=1; }
        else if(!probe_armed && g_robot.motor[i].state==S_SELFTEST) {
            /* Dirty Flash commit may have advanced time since the last loop. */
            g_robot.now_ms=sys_tick_ms;
            g_robot.probe.started_ms=g_robot.probe.ramp_ms=sys_tick_ms;
            probe_started=sys_tick_ms; probe_channel=i;
            probe_target=g_robot.probe.leg_target;
            for(unsigned j=0;j<4;j++) {
                probe_origin[j].count=g_robot.motor[j].count;
                probe_origin[j].errors=g_robot.motor[j].errors;
                uint32_t used=g_robot.motor[j].unexpected_fg;
                uint32_t limit=robot_unexpected_fg_limit(g_robot.config.channel[j].gear);
                probe_unexpected_allowance[j]=limit==UINT32_MAX ? UINT32_MAX :
                    (used<limit ? limit-used:0);
            }
            probe_armed=1;
        }
    }
    /* 这里先清PWM再写DIR/BRAKE，然后恢复需求值；并未验证电气电平或等待DIR稳定。 */
    if(brake || dir!=output_dir[i] || brake!=output_brake[i]) pwm_set_permille(i,0);
    motor_set_dir(i,dir); motor_set_brake(i,brake);
    output_dir[i]=dir; output_brake[i]=brake;
    pwm_set_permille(i,brake ? 0:duty);
    __set_PRIMASK(key);
}
/**
 * @brief 从存储区相对字节偏移读32位字；偏移范围由存储层保证。
 *
 * 输入：offset为保留Flash区内4字节对齐的相对偏移，由存储层保证。
 * 返回：直接读取volatile32位字，没有地址校验；不应对外开放任意读地址。
 */
static uint32_t flash_read(uint32_t offset) { return *(volatile uint32_t *)(STORE_BASE+offset); }
/**
 * @brief 清除上次 Flash 完成、编程和写保护状态，避免旧标志影响本次结果。
 *
 * 过程：清BANK0结束、编程错误、写保护错误标志。
 * 作用：每次编程/擦除前调用，避免把前次状态误判为本次结果。
 */
static void flash_clear_status(void)
{
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
}
/**
 * @brief 写存储区内4字节对齐字；调用者负责解锁，成功返回1，越界/失败返回0。
 *
 * 输入：offset<STORE_SIZE且4字节对齐，word为写入32位数据。
 * 过程：验证范围、清状态、调用fmc_word_program；FMC_READY返回1，其余0。
 * 前提：调用者已解锁Flash且选择空白可编程位置；本函数不擦页。
 */
static int flash_program(uint32_t offset,uint32_t word)
{
    if(offset>=STORE_SIZE || (offset&3)) return 0;
    flash_clear_status();
    return FMC_READY==fmc_word_program(STORE_BASE+offset,word);
}
/**
 * @brief 只允许擦除保留区的两页起点；调用者负责解锁，返回硬件擦除是否成功。
 *
 * 输入：offset只能是0或STORE_PAGE_SIZE，即两页的页首。
 * 过程：清状态后擦除对应页，成功1失败0。
 * 前提：调用者已解锁；不能用于擦任意固件地址。
 */
static int flash_erase(uint32_t offset)
{
    if(offset!=0 && offset!=STORE_PAGE_SIZE) return 0;
    flash_clear_status();
    return FMC_READY==fmc_page_erase(STORE_BASE+offset);
}
/**
 * @brief 保存适配：整板不忙且 PWM为0，写前后核对 FG；扰动时拒绝起动并尽力补写 dirty。
 *
 * 输入：待持久镜像p及dirty；由状态机在静止流程调用。
 * 写前：拒绝busy/PWM非0/FG计数或错误未消费；解锁、追加存储、重新锁Flash。
 * 写后：若期间有FG变化，返回0；若刚写了干净记录则尽力再写dirty保守记录。
 * 限制：返回0可能是运动扰动而非Flash损坏；不会主动清故障或停止正在运行通道。
 */
static int save(const robot_image_t *p,uint32_t dirty)
{
    if(robot_is_busy(&g_robot)) return 0;
    for(unsigned i=0;i<4;i++) if(g_robot.motor[i].duty ||
       g_fg[i].pulse_count!=g_robot.motor[i].count || g_fg[i].errors!=g_robot.motor[i].errors) return 0;
    /* 单Bank Flash操作可能阻塞取指，因此仅静止窗口允许执行；保存后还要检查期间的FG。 */
    fmc_unlock(); int ok=robot_store_save(&store,p,dirty); fmc_lock();
    for(unsigned i=0;i<4;i++) if(g_fg[i].pulse_count!=g_robot.motor[i].count ||
       g_fg[i].errors!=g_robot.motor[i].errors) {
        /* An external disturbance during the quiet write window forbids
         * starting. Try to persist the conservative dirty state as well. */
        if(!dirty && ok) {
            fmc_unlock(); (void)robot_store_save(&store,p,1); fmc_lock();
        }
        return 0;
    }
    return ok;
}
/**
 * @brief RTU完整帧回调；发送忙或接收溢出时不执行请求，避免执行成功却无法排入应答。
 *
 * 输入：RTU已定界的数据p/n，unused为回调上下文占位。
 * 前置：串口发送忙或发现接收溢出时直接丢弃，避免无应答执行。
 * 过程：robot_modbus生成最多256字节应答，再复制到异步发送缓冲。
 * 限制：本函数运行于主循环，不由UART ISR执行业务命令。
 */
static void frame(const uint8_t *p,size_t n,void *unused)
{
    (void)unused;
    uint8_t reply[256];
    /* Half-duplex request/response discipline: never execute requests whose
     * replies cannot be queued. Host must wait for a response or timeout. */
    if(usart0_tx_busy() || rx_overflow!=g_usart0_rx_overrun) return;
    size_t count=robot_modbus(&g_robot,p,n,reply);
    if(count) (void)usart0_send(reply,(uint16_t)count);
}
/**
 * @brief 限量消费带时间戳字节，溢出时丢帧并等待静默重同步；空队列时检查RTU结束。
 *
 * 过程：发现溢出就关中断丢队列并标坏帧；每轮最多消费512个带时间戳事件。
 * 定界：先取当前us，再观察空队列并调用idle，避免新字节导致无符号差误判帧结束。
 * 限制：不回显文本；上位机必须按请求/应答节奏避免排队挤满。
 */
static void poll_serial(void)
{
    if(rx_overflow!=g_usart0_rx_overrun) {
        uint32_t key=__get_PRIMASK(); __disable_irq();
        rx_overflow=g_usart0_rx_overrun; usart0_discard_rx();
        rtu.active=1; rtu.bad=1; rtu.last_us=robot_micros();
        __set_PRIMASK(key);
    }
    uint8_t byte,error; uint32_t stamp;
    for(unsigned k=0;k<512 && usart0_rx_event(&byte,&stamp,&error);k++)
        robot_rtu_byte(&rtu,byte,stamp,error,frame,0);
    /* Take time BEFORE observing empty queue, so a newly arriving byte cannot
     * cause a premature boundary through unsigned timestamp subtraction. */
    uint32_t now=robot_micros();
    if(!usart0_rx_available()) robot_rtu_idle(&rtu,now,frame,0);
}
/**
 * @brief 非阻塞发送应答并最多消费3帧；只接收0x600+node标准8字节请求，应答ID为0x580+node。
 *
 * 发送：已有邮箱成功/失败或超过50ms即释放；待应答先占邮箱，未发完不执行新请求。
 * 接收：每轮最多检查硬件FIFO深度3帧，仅标准数据帧、8字节、0x600+node。
 * 限制：邮箱耗尽时保留待发送应答，不重执行命令；不会用阻塞测试发送函数。
 */
static void poll_can(void)
{
    if(!can_ready) return;
    if(can_mailbox!=CAN_NOMAILBOX) {
        can_transmit_state_enum state=can_transmit_states(CAN0,can_mailbox);
        if(state==CAN_TRANSMIT_OK || state==CAN_TRANSMIT_FAILED ||
           (uint32_t)(sys_tick_ms-can_started)>50) {
            if(state!=CAN_TRANSMIT_OK) (void)can_transmission_stop(CAN0,can_mailbox);
            can_mailbox=CAN_NOMAILBOX;
        }
    }
    if(can_pending && can_mailbox==CAN_NOMAILBOX) {
        can_transmit_message_struct m; can_struct_para_init(CAN_TX_MESSAGE_STRUCT,&m);
        m.tx_sfid=0x580U+g_robot.node; m.tx_ff=CAN_FF_STANDARD; m.tx_ft=CAN_FT_DATA; m.tx_dlen=8;
        memcpy(m.tx_data,can_reply,8); can_mailbox=can_message_transmit(CAN0,&m);
        if(can_mailbox!=CAN_NOMAILBOX) { can_pending=0; can_started=sys_tick_ms; }
    }
    if(can_pending || can_mailbox!=CAN_NOMAILBOX) return;
    /* At most the hardware FIFO depth per loop; no bus load can starve motion. */
    for(unsigned k=0;k<3 && can_receive_message_length_get(CAN0,CAN_FIFO0);k++) {
        can_receive_message_struct m; can_struct_para_init(CAN_RX_MESSAGE_STRUCT,&m);
        can_message_receive(CAN0,CAN_FIFO0,&m);
        if(m.rx_ff!=CAN_FF_STANDARD || m.rx_ft!=CAN_FT_DATA || m.rx_dlen!=8 ||
           m.rx_sfid!=0x600U+g_robot.node) continue;
        robot_can(&g_robot,m.rx_data,can_reply); can_pending=1; break;
    }
}
/**
 * @brief 初始化安全输出、加载Flash/UID、启动FG及选定通信；协议0为USART0，1为CAN。
 *
 * 顺序：GPIO/PWM安全初态，释放JTAG，按宏配置停机输入，加载最新有效存储。
 * 身份：从芯片UID地址读3字，注入输出/保存/频率回调，初始化robot。
 * 完成：启动捕获，按保存的protocol选择USART0或CAN，二者不是同时运行的两条命令通道。
 */
void robot_port_init(void)
{
    motor_io_init(); pwm_init(); initialized=1;
    /* PB4/JTAG released while retaining SWD. No PA15 emergency input by default. */
    rcu_periph_clock_enable(RCU_AF); gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP,ENABLE);
#if BUTTON_ESTOP_ENABLE
    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_init(GPIOA,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,GPIO_PIN_15);
#endif
    robot_image_t p; robot_defaults(&p); uint32_t dirty=1;
    const store_ops_t storage={flash_read,flash_program,flash_erase};
    if(!robot_store_load(&store,&storage,&p,&dirty)) robot_defaults(&p);
    /* 芯片96位UID只读，不是Modbus地址；上位机可用其绑定板卡。 */
    uint32_t uid[3]={*(volatile uint32_t *)0x1FFFF7E8U,
                     *(volatile uint32_t *)0x1FFFF7ECU,*(volatile uint32_t *)0x1FFFF7F0U};
    const robot_ops_t ops={drive,save,pwm_set_frequency};
    robot_init(&g_robot,&ops,&p,(int)dirty,uid,sys_tick_ms);
    capture_init();
    if(g_robot.protocol==0) usart0_init(); else can_ready=can_bus_init();
}
/**
 * @brief 读取一致FG快照，先推进控制再轮询通信；自检ISR锁存停机快照优先于更新的余转数据。
 *
 * 过程：短临界区选择ISR截停快照或当前FG快照，启用按键时先处理停机。
 * 顺序：robot_update消费快照后清latched，再收发协议，按键保持时再次停机。
 * 目的：余转不能先于截停快照消费，否则计数相减会回绕；新余转留下一轮处理。
 */
void robot_port_update(void)
{
    robot_feedback_t f[4];
    uint32_t key=__get_PRIMASK(); __disable_irq();
    uint8_t latched=probe_latched;
    uint32_t sample_ms=latched ? probe_stopped:sys_tick_ms;
    if(latched) for(unsigned i=0;i<4;i++) f[i]=probe_cutoff[i];
    else for(unsigned i=0;i<4;i++) {
        f[i].count=g_fg[i].pulse_count; f[i].errors=g_fg[i].errors;
        f[i].rpm=capture_get_speed_rpm((uint8_t)i);
    }
    __set_PRIMASK(key);
#if BUTTON_ESTOP_ENABLE
    /* Cancel before advancing; keep status reads available while held. */
    uint8_t button_down=RESET==gpio_input_bit_get(GPIOA,GPIO_PIN_15);
    g_robot.stop_input=button_down;
    if(button_down) { g_robot.now_ms=sample_ms; robot_stop_all(&g_robot); }
#endif
    /* Select stop/current snapshot atomically. Never consume newer coast
     * counts before a pending cutoff snapshot (unsigned deltas would wrap).
     * Coast feedback is consumed on the next loop after a latched stop. */
    robot_update(&g_robot,sample_ms,f);
    if(latched) {
        key=__get_PRIMASK(); __disable_irq(); probe_latched=0; __set_PRIMASK(key);
    }
    if(g_robot.protocol==0) poll_serial(); else poll_can();
#if BUTTON_ESTOP_ENABLE
    if(RESET==gpio_input_bit_get(GPIOA,GPIO_PIN_15)) {
        /* Immediate stop without a blocking debounce or serial print. */
        g_robot.stop_input=1;
        robot_stop_all(&g_robot);
    }
#endif
}
