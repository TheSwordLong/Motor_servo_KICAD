/*
 * capture.c — 四路单线 FG 上升沿捕获。定时器按实际总线时钟配置为 1 MHz，溢出扩展时间戳。
 * 每减速前电机转一圈按 6 脉冲测速；FG 本身没有方向信息。ISR 更新 g_fg，主循环读取快照。
 */
#include "capture.h"

#include <stddef.h>

#include "gd32f30x.h"
#include "motor_pins.h"
#include "systick.h"

/* Input filter: 0 = no filter, 1..15 increases noise immunity. */
#define CAPTURE_FILTER          8U

typedef struct {
    uint32_t           timer;      /* TIMER0 / TIMER1 / TIMER2 */
    uint16_t           channel;    /* TIMER_CH_x */
    uint32_t           int_flag;   /* TIMER_INT_FLAG_CHx */
    uint32_t           port;       /* GPIOx */
    uint32_t           pin;        /* GPIO_PIN_x */
    uint32_t           rcu_gpio;   /* RCU_GPIOx */
    uint32_t           remap;      /* value for gpio_pin_remap_config() */
} capture_hw_struct;

/*
 * Channel order matches the motor index: 0 = FG1, 1 = FG2, 2 = FG3, 3 = FG4.
 * See BSP/motor_pins.h for the full pinout and remap rationale.
 */
static const capture_hw_struct capture_hw[CAPTURE_CHANNEL_COUNT] = {
    /* FG1: PB10 / TIMER1_CH2 */
    { TIMER1, TIMER_CH_2, TIMER_INT_FLAG_CH2, M1_FG_PORT, M1_FG_PIN,
      RCU_GPIOB, GPIO_TIMER1_PARTIAL_REMAP1 },
    /* FG2: PE11 / TIMER0_CH1 */
    { TIMER0, TIMER_CH_1, TIMER_INT_FLAG_CH1, M2_FG_PORT, M2_FG_PIN,
      RCU_GPIOE, GPIO_TIMER0_FULL_REMAP },
    /* FG3: PB4 / TIMER2_CH0 */
    { TIMER2, TIMER_CH_0, TIMER_INT_FLAG_CH0, M3_FG_PORT, M3_FG_PIN,
      RCU_GPIOB, GPIO_TIMER2_PARTIAL_REMAP },
    /* FG4: PA0 / TIMER1_CH0 */
    { TIMER1, TIMER_CH_0, TIMER_INT_FLAG_CH0, M4_FG_PORT, M4_FG_PIN,
      RCU_GPIOA, GPIO_TIMER1_PARTIAL_REMAP1 }
};

/* Monitoring data, one entry per FG channel. */
volatile fg_monitor_struct g_fg[CAPTURE_CHANNEL_COUNT];
volatile fg_debug_struct g_fg_debug[CAPTURE_CHANNEL_COUNT];

/**
 * @brief 主循环最多每 10 ms 发布一次调试快照；短暂关中断读取一致数据，速度超过 250 ms 无边沿显示 0。
 *
 * 输入：读取sys_tick_ms、g_fg和物理FG输入电平；无外部参数。
 * 过程：间隔不足10ms直接返回；关中断复制，恢复中断后计算age及pulse_delta。
 * 输出：g_fg_debug[0..3]用于Live Watch；首次pulse_delta=0，无边沿超过250ms时显示RPM=0。
 * 限制：不修改原始捕获计数或运动状态；调试器同时读取整个数组仍可能跨两次发布。
 */
void capture_debug_update(void)
{
    static uint32_t last_sample_ms;
    static uint32_t previous_count[CAPTURE_CHANNEL_COUNT];
    static uint8_t sampled;
    uint32_t now=sys_tick_ms;
    if(sampled && (uint32_t)(now-last_sample_ms)<10U) return;
    fg_debug_struct snapshot[CAPTURE_CHANNEL_COUNT];
    /* Coherent ISR data, with only a short read-only critical section.
     * Publish the debug copy after restoring IRQs; control never reads it. */
    uint32_t key=__get_PRIMASK(); __disable_irq();
    now=sys_tick_ms;
    for(unsigned i=0;i<CAPTURE_CHANNEL_COUNT;i++) {
        snapshot[i].pulse_count=g_fg[i].pulse_count;
        snapshot[i].errors=g_fg[i].errors;
        snapshot[i].period_us=g_fg[i].period_us;
        snapshot[i].last_edge_ms=g_fg[i].last_edge_ms;
        snapshot[i].speed_rpm=g_fg[i].speed_rpm;
        snapshot[i].level=(uint32_t)(gpio_input_bit_get(capture_hw[i].port,capture_hw[i].pin)!=RESET);
    }
    __set_PRIMASK(key);
    for(unsigned i=0;i<CAPTURE_CHANNEL_COUNT;i++) {
        snapshot[i].age_ms=now-snapshot[i].last_edge_ms;
        if(snapshot[i].age_ms>250U) snapshot[i].speed_rpm=0;
        snapshot[i].pulse_delta=sampled ? snapshot[i].pulse_count-previous_count[i]:0U;
        previous_count[i]=snapshot[i].pulse_count;
        snapshot[i].sample_ms=now;
        g_fg_debug[i]=snapshot[i];
    }
    last_sample_ms=now; sampled=1;
}

/* Previous capture value per channel; used to compute the period. */
static volatile uint32_t capture_last[CAPTURE_CHANNEL_COUNT];
static uint32_t capture_high[3];
static volatile uint8_t capture_have_previous[CAPTURE_CHANNEL_COUNT];

/* Extend each 16-bit timer with its own update interrupt. If update and
 * capture are pending together, low captures belong to the new epoch.
 * Valid while IRQ latency < half a timer wrap (32.768ms). */
/**
 * @brief 共用捕获 ISR：处理计数器溢出、过捕获错误、周期及边沿计数；ti 对应定时器扩展高位槽。
 *
 * 输入：timer为TIMER0/1/2，ti为对应高位累计槽0/1/2。
 * 过程：遍历该定时器所属电机，读取CCR，补偿溢出，检查过捕获，再更新周期和边沿数。
 * 测速：已有上一边沿且间隔不超过1000ms才计算；period_us=1MHz计数差，RPM=10000000/period_us。
 * 异常：过捕获只递增errors，故障决策交主循环；禁止在此阻塞发送或写Flash。
 */
static void capture_irq(uint32_t timer, unsigned ti)
{
    uint32_t flags=TIMER_INTF(timer);
    uint32_t epoch=capture_high[ti];
    for(unsigned i=0;i<CAPTURE_CHANNEL_COUNT;i++) {
        const capture_hw_struct *hw=&capture_hw[i];
        if(hw->timer!=timer) continue;
        uint32_t over=hw->int_flag<<8;
        if(!(flags & hw->int_flag)) {
            if(flags&over) { g_fg[i].errors++; timer_flag_clear(timer,over); }
            continue;
        }
        uint32_t low=timer_channel_capture_value_register_read(timer,hw->channel)&0xFFFFU;
        uint32_t after=TIMER_INTF(timer);
        uint32_t stamp=epoch+low;
        /* 捕获与溢出接近时用低半周期判断先后，避免跨65536us回绕得到错误周期。 */
        if(((flags|after)&TIMER_INTF_UPIF) && low<0x8000U) stamp+=0x10000U;
        if((flags|after)&over) { g_fg[i].errors++; timer_flag_clear(timer,over); }
        uint32_t elapsed=sys_tick_ms-g_fg[i].last_edge_ms;
        /* 首个边沿或间隔超过1秒只建立基准；有效连续边沿周期用于测速。 */
        if(capture_have_previous[i] && elapsed<=1000U) {
            uint32_t period=stamp-capture_last[i];
            g_fg[i].period_ticks=(uint16_t)period;
            g_fg[i].period_us=period;
            g_fg[i].freq_hz=period ? 1000000U/period:0;
            /* 60秒×1000000us÷每圈6脉冲，因此电机RPM=10000000/周期us。 */
            g_fg[i].speed_rpm=period ? 10000000U/period:0;
        } else { g_fg[i].period_us=0; g_fg[i].freq_hz=0; g_fg[i].speed_rpm=0; }
        capture_last[i]=stamp; capture_have_previous[i]=1;
        g_fg[i].last_edge_ms=sys_tick_ms; g_fg[i].pulse_count++;
        /* Reading CHxCV clears CHxIF. Do not clear it again: a new edge may
         * have arrived after the register read. */
    }
    if(flags&TIMER_INTF_UPIF) {
        capture_high[ti]+=0x10000U;
        timer_interrupt_flag_clear(timer,TIMER_INT_FLAG_UP);
    }
}
/**
 * @brief 服务共享 TIMER1 的 M1/CH2 和 M4/CH0 捕获及溢出中断。
 *
 * 上下文：硬件ISR，共用capture_irq(TIMER1,1)。
 * 归属：M1和M4共用定时器；统一处理捕获/溢出，不能为其中一路单独复位整个TIMER1。
 */
void TIMER1_IRQHandler(void) { capture_irq(TIMER1,1); }
/**
 * @brief 服务 M3 的 TIMER2_CH0 捕获及溢出中断。
 *
 * 上下文：硬件ISR，共用capture_irq(TIMER2,2)。
 * 归属：M3 FG输入为PB4，初始化须先释放JTAG占用。
 */
void TIMER2_IRQHandler(void) { capture_irq(TIMER2,2); }
/**
 * @brief 处理 TIMER0 溢出并通过共用逻辑处理同次捕获标志。
 *
 * 上下文：TIMER0更新ISR，共用高位时间戳槽0。
 * 说明：与通道ISR调用同一处理器，由实际标志决定工作，不对每个入口盲目增加脉冲。
 */
void TIMER0_UP_IRQHandler(void) { capture_irq(TIMER0,0); }
/**
 * @brief 处理 M2 的 TIMER0_CH1 捕获，与更新中断共享时间戳逻辑。
 *
 * 上下文：TIMER0通道ISR，负责M2/PE11捕获。
 * 说明：共享处理器协调UPIF及捕获先后；周期不是仅16位CCR直接相减。
 */
void TIMER0_Channel_IRQHandler(void) { capture_irq(TIMER0,0); }

/**
 * @brief 按物理通道表配置 FG 内部上拉和定时器重映射；index 必须小于 4。
 *
 * 输入：index=0～3，本函数不做越界检查，由capture_init保证。
 * 过程：按capture_hw表启用GPIO时钟、AF重映射及内部上拉；不配置输出驱动。
 */
static void capture_gpio_init(uint8_t index)
{
    const capture_hw_struct *hw = &capture_hw[index];

    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(hw->rcu_gpio);

    /* Each channel writes a different AFIO_PCF0 field, so they do not clobber. */
    gpio_pin_remap_config(hw->remap, ENABLE);

    /*
     * Pull-up input: motor FG outputs are usually open-drain and need a pull-up
     * to produce a valid high level. A push-pull source overrides it harmlessly.
     */
    gpio_init(hw->port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, hw->pin);
}

/**
 * @brief 启动时初始化四路捕获和三个定时器；共享的 TIMER1 只初始化一次。
 *
 * 过程：清各路测量基准；逐路配置输入，上升沿、无输入分频、数字滤波值8。
 * 时基：APB分频时补定时器倍频，计数器1MHz/16位；同一定时器仅初始化一次。
 * 中断：启用捕获及更新，TIMER0两个入口和TIMER1/2优先级均为1。
 * 限制：面向上电初始化；运行中重复调用会破坏计数基准，不是运动清零接口。
 */
void capture_init(void)
{
    uint8_t index;
    uint32_t inited_timer[CAPTURE_CHANNEL_COUNT];
    uint8_t inited_count = 0U;
    uint8_t timer_done;
    uint8_t i;

    for(index = 0U; index < CAPTURE_CHANNEL_COUNT; index++) 
    {
        timer_parameter_struct timerpara;
        timer_ic_parameter_struct icpara;
        const capture_hw_struct *hw = &capture_hw[index];

        g_fg[index].period_ticks = 0U;
        g_fg[index].period_us = 0U;
        g_fg[index].freq_hz = 0U;
        g_fg[index].speed_rpm = 0U;
        g_fg[index].pulse_count = 0U;
        capture_last[index] = 0U;
        capture_have_previous[index] = 0U;

        capture_gpio_init(index);

        /*
         * FG1 and FG4 share TIMER1, so configure the timer base only once.
         * Re-running timer_init()/timer_deinit() for the second channel would
         * reset the counter that the first channel is already using.
         */
        timer_done = 0U;
        for(i = 0U; i < inited_count; i++) {
            if(inited_timer[i] == hw->timer) {
                timer_done = 1U;
                break;
            }
        }

        if(0U == timer_done) {
            rcu_periph_clock_enable((hw->timer == TIMER0) ? RCU_TIMER0 :
                                    (hw->timer == TIMER1) ? RCU_TIMER1 : RCU_TIMER2);
            timer_deinit(hw->timer);

            timer_struct_para_init(&timerpara);
            uint32_t clock=rcu_clock_freq_get(hw->timer==TIMER0 ? CK_APB2:CK_APB1);
            uint32_t mask=hw->timer==TIMER0 ? RCU_CFG0_APB2PSC:RCU_CFG0_APB1PSC;
            /* 分别读取APB1/APB2并应用定时器倍频规则，统一配置1us计数步长。 */
            if((RCU_CFG0 & mask)!=0) clock*=2;
            timerpara.prescaler         = clock/1000000U-1U;
            timerpara.alignedmode       = TIMER_COUNTER_EDGE;
            timerpara.counterdirection  = TIMER_COUNTER_UP;
            timerpara.clockdivision     = TIMER_CKDIV_DIV1;
            timerpara.period            = 0xFFFFU;
            timerpara.repetitioncounter = 0U;
            timer_init(hw->timer, &timerpara);

            inited_timer[inited_count] = hw->timer;
            inited_count++;
        }

        timer_channel_input_struct_para_init(&icpara);
        icpara.icpolarity  = TIMER_IC_POLARITY_RISING;
        icpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
        icpara.icprescaler = TIMER_IC_PSC_DIV1;
        icpara.icfilter    = CAPTURE_FILTER;
        timer_input_capture_config(hw->timer, hw->channel, &icpara);

        timer_interrupt_flag_clear(hw->timer, hw->int_flag);

        /* Enable the matching channel interrupt (CH0/CH1/CH2 share one macro each). */
        if(TIMER_INT_FLAG_CH0 == hw->int_flag) {
            timer_interrupt_enable(hw->timer, TIMER_INT_CH0);
        } else if(TIMER_INT_FLAG_CH1 == hw->int_flag) {
            timer_interrupt_enable(hw->timer, TIMER_INT_CH1);
        } else {
            timer_interrupt_enable(hw->timer, TIMER_INT_CH2);
        }

        timer_interrupt_flag_clear(hw->timer,TIMER_INT_FLAG_UP);
        timer_interrupt_enable(hw->timer,TIMER_INT_UP);
        timer_enable(hw->timer);
    }

    /* Same preemption, different sub-priority, so the channels do not block each other. */
    nvic_irq_enable(TIMER0_UP_IRQn, 1U, 0U);
    nvic_irq_enable(TIMER0_Channel_IRQn, 1U, 0U);
    nvic_irq_enable(TIMER1_IRQn, 1U, 0U);
    nvic_irq_enable(TIMER2_IRQn, 1U, 0U);
}

/**
 * @brief 读取累计无符号 FG 边沿数；index 为 0～3，越界返回 0，计数不含方向。
 *
 * 输入：index=0～3；返回该通道自采集以来的uint32累计上升沿数。
 * 限制：计数可能回绕；多字段一致性由调用者关中断快照保证，不会自动换算为角度。
 */
uint32_t capture_get_count(uint8_t index)
{
    if(index >= CAPTURE_CHANNEL_COUNT) {
        return 0U;
    }

    return g_fg[index].pulse_count;
}

/**
 * @brief 返回上次计算的周期 us；越界返回 0，静止时可能保留历史周期。
 *
 * 输入：index=0～3；输出上一次有效测量的周期微秒数。
 * 限制：首边沿可能为0；停转后历史周期不会在本函数清除，不能只凭非零判断仍在转动。
 */
uint32_t capture_get_period_us(uint8_t index)
{
    if(index >= CAPTURE_CHANNEL_COUNT) {
        return 0U;
    }

    return g_fg[index].period_us;
}

/**
 * @brief 返回上次计算的整数频率 Hz；越界返回 0，此接口不做静止超时清零。
 *
 * 输入：index=0～3；输出整数除法得到的Hz，分数部分丢弃。
 * 限制：没有年龄检查；需要当前速度应使用带250ms超时的capture_get_speed_rpm。
 */
uint32_t capture_get_freq_hz(uint8_t index)
{
    if(index >= CAPTURE_CHANNEL_COUNT) {
        return 0U;
    }

    return g_fg[index].freq_hz;
}

/**
 * @brief 返回减速前 RPM；越界或超过 250 ms 没有边沿返回 0。
 *
 * 输入：index=0～3；依据当前sys_tick_ms与last_edge_ms判断反馈年龄。
 * 返回：有效期内返回减速前RPM，超过250ms或越界返回0；不是输出轴RPM。
 * 限制：零速可能表示还未取得两次有效边沿，不足以单独判断断线。
 */
uint32_t capture_get_speed_rpm(uint8_t index)
{
    if(index >= CAPTURE_CHANNEL_COUNT) {
        return 0U;
    }

    return (uint32_t)(sys_tick_ms-g_fg[index].last_edge_ms)>250U ? 0U:g_fg[index].speed_rpm;
}
