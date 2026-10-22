/*
 * led.c — PC13 低有效指示灯接口；pulse 为阻塞延时，当前机器人主循环未调用这些灯效。
 */
#include "led.h"

#include "gd32f30x.h"
#include "systick.h"

#define LED0_GPIO_PORT       GPIOC
#define LED0_GPIO_PIN        GPIO_PIN_13
#define LED0_GPIO_CLOCK      RCU_GPIOC

/**
 * @brief 配置 PC13 推挽输出并熄灭指示灯。
 *
 * 过程：开启GPIOC时钟，将PC13设为2MHz推挽输出，调用led0_off置高。
 * 作用：只初始化灯；与电机运行状态没有自动绑定。
 */
void led0_init(void)
{
    rcu_periph_clock_enable(LED0_GPIO_CLOCK);
    gpio_init(LED0_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, LED0_GPIO_PIN);
    led0_off();
}

/**
 * @brief PC13 输出低电平点亮指示灯。
 *
 * 过程：复位PC13输出位；按本板低有效接法表示亮。
 * 限制：无延时和状态缓存，调用前应已初始化GPIO。
 */
void led0_on(void)
{
    /* LED0 is active-low. */
    gpio_bit_reset(LED0_GPIO_PORT, LED0_GPIO_PIN);
}

/**
 * @brief PC13 输出高电平熄灭指示灯。
 *
 * 过程：置位PC13输出位；按本板低有效接法表示灭。
 * 限制：不会改变其他GPIO位或电机输出。
 */
void led0_off(void)
{
    gpio_bit_set(LED0_GPIO_PORT, LED0_GPIO_PIN);
}

/**
 * @brief 亮灯 duration_ms 毫秒后熄灭；阻塞调用，不适合运动控制实时路径。
 *
 * 输入：duration_ms为毫秒；顺序为亮灯、delay_1ms、灭灯。
 * 限制：整段阻塞调用线程；依赖系统节拍，勿在ISR或运动更新路径使用。
 */
void led0_pulse(uint32_t duration_ms)
{
    led0_on();
    delay_1ms(duration_ms);
    led0_off();
}
