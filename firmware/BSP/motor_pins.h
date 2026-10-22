/*
 * motor_pins.h — 电机接口固定引脚表；逻辑关节标签和减速比配置不会改变这里的物理接线。
 * M3 BRAKE 使用 PD2。FG 为开集电极反馈的采集输入，按减速前每圈 6 脉冲处理。
 */
#ifndef BSP_MOTOR_PINS_H
#define BSP_MOTOR_PINS_H

#include "gd32f30x.h"

/*
 * Four-motor pin assignment for GD32F303VET6 (LQFP100).
 *
 * Motor | PWM (timer)          | DIR (GPIO) | BRAKE (GPIO) | FG (timer)
 * ------+----------------------+------------+--------------+--------------------
 *   1   | PD12 / TIMER3_CH0    | PE15       | PE13         | PB10 / TIMER1_CH2
 *   2   | PD13 / TIMER3_CH1    | PC5        | PA7          | PE11 / TIMER0_CH1
 *   3   | PD14 / TIMER3_CH2    | PD3        | PD2          | PB4  / TIMER2_CH0
 *   4   | PD15 / TIMER3_CH3    | PB8        | PB5          | PA0  / TIMER1_CH0
 *
 * DIR and BRAKE are plain GPIO outputs; they never use a timer.
 *
 * Signal levels:
 *   DIR   low  = forward, high = reverse      -> initialized: low (forward)
 *   BRAKE low  = brake,   high = released     -> initialized: low (engaged)
 *   PWM   high = stopped, low = running.
 * These are software-initialized levels, NOT a guarantee during hardware reset.
 *
 * ---------------------------------------------------------------------------
 * Timer / remap allocation (AFIO_PCF0 fields, all independent):
 *
 *   TIMER3  bit12   = 1     PWM1..PWM4  -> PD12..PD15  (full remap)
 *   TIMER1  bit9:8  = 0b10  FG1 = PB10(CH2), FG4 = PA0(CH0)
 *   TIMER2  bit11:10= 0b10  FG3 = PB4(CH0)
 *   TIMER0  bit7:6  = 0b11  FG2 = PE11(CH1)
 *
 * Per the GD32F30x user manual:
 *   TIMER3_REMAP 1 bit   : 0 = PB6..PB9, 1 = PD12..PD15
 *   TIMER1_REMAP [1:0]   : 10 = CH0/PA0, CH1/PA1, CH2/PB10, CH3/PB11
 *   TIMER2_REMAP [1:0]   : 10 = CH0/PB4, CH1/PB5, CH2/PB0, CH3/PB1
 *   TIMER0_REMAP [1:0]   : 11 = CH0/PE9, CH1/PE11, CH2/PE13, CH3/PE14, BKIN/PE15
 *
 * IMPORTANT - remap does not steal a pin from GPIO:
 *   Enabling a timer remap only changes which pins the timer channels are
 *   routed to. A pin is driven by the timer ONLY when its GPIO mode is set to
 *   alternate function (GPIO_MODE_AF_PP). Because PE13, PE15, PA7, PB5 and PB8
 *   are configured as GPIO_MODE_OUT_PP, they remain plain GPIO outputs even
 *   though TIMER0 full remap nominally covers PE13/PE15 and TIMER3 remap
 *   nominally covers PB8.
 *
 * ---------------------------------------------------------------------------
 * Hardware notes:
 *   - PB10 is USART2_TX by default (see BSP/usart2.c). Do not enable USART2
 *     while FG1 occupies PB10; use the PD8/PD9 remap instead if needed.
 *   - PB4 is JTAG NJTRST; BSP/button.c already releases it from JTAG via
 *     GPIO_SWJ_SWDPENABLE_REMAP.
 *   - PE15 is TIMER0_BKIN under TIMER0 full remap. It is used here as a plain
 *     GPIO (DIR1), so TIMER0 break input is not available while DIR1 uses it.
 */

/* --- Motor 1 --------------------------------------------------------- */
/* M1：PWM PD12，DIR PE15，BRAKE PE13，FG PB10/TIMER1_CH2。 */
#define M1_PWM_PORT         GPIOD
#define M1_PWM_PIN          GPIO_PIN_12
#define M1_DIR_PORT         GPIOE
#define M1_DIR_PIN          GPIO_PIN_15
#define M1_BRAKE_PORT       GPIOE
#define M1_BRAKE_PIN        GPIO_PIN_13
#define M1_FG_PORT          GPIOB
#define M1_FG_PIN           GPIO_PIN_10

/* --- Motor 2 --------------------------------------------------------- */
/* M2：PWM PD13，DIR PC5，BRAKE PA7，FG PE11/TIMER0_CH1。 */
#define M2_PWM_PORT         GPIOD
#define M2_PWM_PIN          GPIO_PIN_13
#define M2_DIR_PORT         GPIOC
#define M2_DIR_PIN          GPIO_PIN_5
#define M2_BRAKE_PORT       GPIOA
#define M2_BRAKE_PIN        GPIO_PIN_7
#define M2_FG_PORT          GPIOE
#define M2_FG_PIN           GPIO_PIN_11

/* --- Motor 3 --------------------------------------------------------- */
/* M3：PWM PD14，DIR PD3，BRAKE PD2，FG PB4/TIMER2_CH0；PB4须释放JTAG。 */
#define M3_PWM_PORT         GPIOD
#define M3_PWM_PIN          GPIO_PIN_14
#define M3_DIR_PORT         GPIOD
#define M3_DIR_PIN          GPIO_PIN_3
#define M3_BRAKE_PORT       GPIOD
#define M3_BRAKE_PIN        GPIO_PIN_2
#define M3_FG_PORT          GPIOB
#define M3_FG_PIN           GPIO_PIN_4

/* --- Motor 4 --------------------------------------------------------- */
/* M4：PWM PD15，DIR PB8，BRAKE PB5，FG PA0/TIMER1_CH0。 */
#define M4_PWM_PORT         GPIOD
#define M4_PWM_PIN          GPIO_PIN_15
#define M4_DIR_PORT         GPIOB
#define M4_DIR_PIN          GPIO_PIN_8
#define M4_BRAKE_PORT       GPIOB
#define M4_BRAKE_PIN        GPIO_PIN_5
#define M4_FG_PORT          GPIOA
#define M4_FG_PIN           GPIO_PIN_0

/* --- PWM timer (all four channels share TIMER3) ---------------------- */
#define PWM_TIMER           TIMER3
#define PWM_RCU_TIMER       RCU_TIMER3
#define PWM_GPIO_PORT       GPIOD
#define PWM_GPIO_CLOCK      RCU_GPIOD
#define PWM_GPIO_PINS       (GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)

/* --- Direction / brake output ports and clocks ----------------------- */
/* 仅为跨端口引脚位号集合，不能把该掩码应用到同一个GPIO端口。 */
#define MOTOR_DIR_GPIO_PINS (GPIO_PIN_15 | GPIO_PIN_5 | GPIO_PIN_3 | GPIO_PIN_8)

/* --- Signal levels --------------------------------------------------- */
#define DIR_FORWARD_LEVEL   0U      /* low  = forward */
#define DIR_REVERSE_LEVEL   1U      /* high = reverse */
#define BRAKE_ON_LEVEL      0U      /* low  = brake   */
#define BRAKE_OFF_LEVEL     1U      /* high = released (power-on default) */

/* --- Speed feedback: pulses per mechanical revolution ---------------- */
#define FG_PULSES_PER_ROUND 6U

#endif /* BSP_MOTOR_PINS_H */
