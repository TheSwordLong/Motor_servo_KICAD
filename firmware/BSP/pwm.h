/*
 * pwm.h — 四路 TIMER3 PWM 底层，PD12～PD15 全重映射对应 CH0～CH3。
 * duty 为低电平千分比：0=持续高电平停止，1000=持续低电平运行；不是 RPM。
 * 正常运动上限由 robot.h 另行限制，本模块只限制输入不超过 1000。
 */
#ifndef BSP_PWM_H
#define BSP_PWM_H
#include <stdint.h>
/* Low-time duty: 0=HIGH stop, 100%=LOW run. Fixed TIMER3 CH0..3 PD12..15. */
#define PWM_PERIOD 4799U
/**
 * @brief 先把 PWM GPIO 拉高，再配置低有效 TIMER3 PWM 和重映射；初始 25 kHz、四路占空比 0。
 *
 * 顺序：GPIO预置高并设推挽，复位TIMER3，配置PWM0/低极性/高空闲态，CCR全部0。
 * 完成：设25kHz、使能定时器、启用全重映射，PD12～15切为复用推挽。
 * 限制：不能覆盖CPU仍在硬件复位期间的引脚电平，也不代替外部电平设计。
 */
void pwm_init(void);
/**
 * @brief 写比较寄存器，compare 单位为定时器计数；超过当前周期截断，无效通道忽略。
 *
 * 输入：channel=0～3，compare为CCR计数，允许0到当前period；越界通道直接返回。
 * 过程：compare超过period时截断，再写TIMER3对应通道CCR，无等待。
 * 含义：低极性PWM下0为持续高，period为持续低；不是百分比接口。
 */
void pwm_set_duty(uint8_t channel,uint16_t compare);
/**
 * @brief 整数百分比转千分比交给底层，超过 100% 会被截断；channel 为 0～3。
 *
 * 输入：percent为uint8整数百分比；乘10后转到千分比接口。
 * 限制：不支持小数百分比；大于100最终在底层截为1000，无RPM闭环功能。
 */
void pwm_set_duty_percent(uint8_t channel,uint8_t percent);
/**
 * @brief 设置低电平千分比 0～1000，四舍五入换算比较值；0 为停止高电平。
 *
 * 输入：duty为低电平千分比，超过1000截为1000，channel透传给底层。
 * 换算：compare=(period*duty+500)/1000，+500用于最接近整数计数的舍入。
 * 示例：100为10%低，950为95%低；这里只做输出换算，不读取FG或限制到正常运动950。
 */
void pwm_set_permille(uint8_t channel,uint16_t duty);
/* All motors must be stopped. Resets all duties to zero. */
/**
 * @brief 仅接受 20000～30000 Hz，先停四路再改周期；调用者须保证电机已停止。
 *
 * 输入：hz在20000～30000内才执行，越界直接返回且保留原输出。
 * 过程：四路先清零，按实际APB1及倍频求计数时钟；period四舍五入，ARR=period-1并软件更新。
 * 影响：四路共享频率，原占空比不会恢复；必须在整板静止时调用。
 */
void pwm_set_frequency(uint32_t hz);
/**
 * @brief 四路比较值清零，PWM 变为停止高电平；本函数不设置 BRAKE。
 *
 * 过程：依次把四路CCR写0；输出为持续高停止指令。
 * 限制：不会设置刹车、不修改robot.motor的duty缓存；正常业务应经状态机保持软硬件一致。
 */
void pwm_stop_all(void);
#endif
