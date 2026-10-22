/*
 * pwm.c — 四路 TIMER3 PWM 底层，PD12～PD15 全重映射对应 CH0～CH3。
 * duty 为低电平千分比：0=持续高电平停止，1000=持续低电平运行；不是 RPM。
 * 正常运动上限由 robot.h 另行限制，本模块只限制输入不超过 1000。
 */
#include "pwm.h"
#include "gd32f30x.h"
#include "motor_pins.h"
static uint32_t period=PWM_PERIOD+1U;
static const uint16_t channels[4]={TIMER_CH_0,TIMER_CH_1,TIMER_CH_2,TIMER_CH_3};
/**
 * @brief 写比较寄存器，compare 单位为定时器计数；超过当前周期截断，无效通道忽略。
 *
 * 输入：channel=0～3，compare为CCR计数，允许0到当前period；越界通道直接返回。
 * 过程：compare超过period时截断，再写TIMER3对应通道CCR，无等待。
 * 含义：低极性PWM下0为持续高，period为持续低；不是百分比接口。
 */
void pwm_set_duty(uint8_t channel,uint16_t compare)
{
    if(channel>=4) return;
    if(compare>period) compare=(uint16_t)period;
    timer_channel_output_pulse_value_config(TIMER3,channels[channel],compare);
}

/**
 * @brief 设置低电平千分比 0～1000，四舍五入换算比较值；0 为停止高电平。
 *
 * 输入：duty为低电平千分比，超过1000截为1000，channel透传给底层。
 * 换算：compare=(period*duty+500)/1000，+500用于最接近整数计数的舍入。
 * 示例：100为10%低，950为95%低；这里只做输出换算，不读取FG或限制到正常运动950。
 */
void pwm_set_permille(uint8_t channel,uint16_t duty)
{
    if(duty>1000) duty=1000;
    /* 低电平计数=周期计数×千分比/1000，+500实现整数舍入；0会产生持续高电平。 */
    pwm_set_duty(channel,(uint16_t)((period*duty+500)/1000));
}


/**
 * @brief 整数百分比转千分比交给底层，超过 100% 会被截断；channel 为 0～3。
 *
 * 输入：percent为uint8整数百分比；乘10后转到千分比接口。
 * 限制：不支持小数百分比；大于100最终在底层截为1000，无RPM闭环功能。
 */
void pwm_set_duty_percent(uint8_t channel,uint8_t percent)
{ pwm_set_permille(channel,(uint16_t)percent*10U); }


/**
 * @brief 四路比较值清零，PWM 变为停止高电平；本函数不设置 BRAKE。
 *
 * 过程：依次把四路CCR写0；输出为持续高停止指令。
 * 限制：不会设置刹车、不修改robot.motor的duty缓存；正常业务应经状态机保持软硬件一致。
 */
void pwm_stop_all(void)
{ for(unsigned i=0;i<4;i++) pwm_set_duty((uint8_t)i,0); }


/**
 * @brief 仅接受 20000～30000 Hz，先停四路再改周期；调用者须保证电机已停止。
 *
 * 输入：hz在20000～30000内才执行，越界直接返回且保留原输出。
 * 过程：四路先清零，按实际APB1及倍频求计数时钟；period四舍五入，ARR=period-1并软件更新。
 * 影响：四路共享频率，原占空比不会恢复；必须在整板静止时调用。
 */
void pwm_set_frequency(uint32_t hz)
{
    if(hz<20000 || hz>30000) return;
    pwm_stop_all();
    uint32_t clock=rcu_clock_freq_get(CK_APB1);
    /* APB1分频不为1时定时器时钟倍频，不能把APB1时钟直接当PWM计数时钟。 */
    if((RCU_CFG0 & RCU_CFG0_APB1PSC)!=0) clock*=2;
    period=(clock+hz/2)/hz;
    timer_autoreload_value_config(TIMER3,period-1);
    timer_event_software_generate(TIMER3,TIMER_EVENT_SRC_UPG);
}


/**
 * @brief 先把 PWM GPIO 拉高，再配置低有效 TIMER3 PWM 和重映射；初始 25 kHz、四路占空比 0。
 *
 * 顺序：GPIO预置高并设推挽，复位TIMER3，配置PWM0/低极性/高空闲态，CCR全部0。
 * 完成：设25kHz、使能定时器、启用全重映射，PD12～15切为复用推挽。
 * 限制：不能覆盖CPU仍在硬件复位期间的引脚电平，也不代替外部电平设计。
 */
void pwm_init(void)
{
    rcu_periph_clock_enable(RCU_AF); rcu_periph_clock_enable(RCU_GPIOD);
    gpio_bit_set(GPIOD,PWM_GPIO_PINS);
    gpio_init(GPIOD,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,PWM_GPIO_PINS);
    rcu_periph_clock_enable(RCU_TIMER3); timer_deinit(TIMER3);
    timer_parameter_struct t; timer_struct_para_init(&t);
    t.prescaler=0; t.period=PWM_PERIOD; timer_init(TIMER3,&t);
    timer_oc_parameter_struct oc; timer_channel_output_struct_para_init(&oc);
    /* PWM0配合低极性：比较窗口为低电平，比较值0为停止高电平。 */
    oc.outputstate=TIMER_CCX_ENABLE; oc.ocpolarity=TIMER_OC_POLARITY_LOW;
    oc.ocidlestate=TIMER_OC_IDLE_STATE_HIGH;
    for(unsigned i=0;i<4;i++) {
        timer_channel_output_config(TIMER3,channels[i],&oc);
        timer_channel_output_mode_config(TIMER3,channels[i],TIMER_OC_MODE_PWM0);
        timer_channel_output_shadow_config(TIMER3,channels[i],TIMER_OC_SHADOW_DISABLE);
        pwm_set_duty((uint8_t)i,0);
    }
    pwm_set_frequency(25000); timer_enable(TIMER3);
    gpio_pin_remap_config(GPIO_TIMER3_REMAP,ENABLE);
    gpio_init(GPIOD,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,PWM_GPIO_PINS);
}
