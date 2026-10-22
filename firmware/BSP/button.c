/*
 * button.c — PA15 低有效按键及阻塞式 20 ms 消抖。当前 robot_port 的可选停机输入采用直接采样，
 * 不调用本模块的按键事件接口；BUTTON_ESTOP_ENABLE 默认关闭。
 */
#include "button.h"

#include "gd32f30x.h"
#include "systick.h"

#define SW1_GPIO_PORT          GPIOA
#define SW1_GPIO_PIN           GPIO_PIN_15
#define SW1_GPIO_CLOCK         RCU_GPIOA
#define SW1_PRESSED_LEVEL      RESET
#define SW1_DEBOUNCE_MS        20U

/**
 * @brief 配置 PA15 内部上拉并释放 JTAG 引脚，保留 SWD。
 *
 * 调用：启动阶段一次；需要GD32库时钟接口可用。
 * 过程：启用AF/GPIOA，关闭JTAG保留SWD，再把PA15设为内部上拉输入；不启动中断。
 * 限制：只初始化引脚，不判断是否装有实体按键，不直接停止电机。
 */
void button_init(void)
{
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(SW1_GPIO_CLOCK);

    /* Release PA15 (JTDI) from JTAG while keeping PA13/PA14 SWD enabled. */
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);

    /* SW1 is active-low: PA15 is pulled up internally and pressed to GND. */
    gpio_init(SW1_GPIO_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, SW1_GPIO_PIN);
}

/**
 * @brief 每次消抖后的按下只返回一次 1，否则返回 0；按下/松开确认可能阻塞 20 ms。
 *
 * 输出：确认新按下返回1；保持按住、松开或抖动未确认返回0。
 * 过程：静态press_accepted记住本次按压；松开也需延时确认后才允许下一次事件。
 * 限制：主循环调用，不适合ISR；每次消抖等待20ms，当前机器人停机路径未调用它。
 */
uint8_t button_pressed_event(void)
{
    static uint8_t press_accepted = 0U;
    FlagStatus level = gpio_input_bit_get(SW1_GPIO_PORT, SW1_GPIO_PIN);

    if((0U == press_accepted) && (SW1_PRESSED_LEVEL == level)) {
        delay_1ms(SW1_DEBOUNCE_MS);

        if(SW1_PRESSED_LEVEL == gpio_input_bit_get(SW1_GPIO_PORT, SW1_GPIO_PIN)) {
            press_accepted = 1U;
            return 1U;
        }
    } else if((0U != press_accepted) && (SW1_PRESSED_LEVEL != level)) {
        delay_1ms(SW1_DEBOUNCE_MS);

        if(SW1_PRESSED_LEVEL != gpio_input_bit_get(SW1_GPIO_PORT, SW1_GPIO_PIN)) {
            press_accepted = 0U;
        }
    }

    return 0U;
}
