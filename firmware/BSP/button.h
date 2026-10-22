/*
 * button.h — PA15 低有效按键及阻塞式 20 ms 消抖。当前 robot_port 的可选停机输入采用直接采样，
 * 不调用本模块的按键事件接口；BUTTON_ESTOP_ENABLE 默认关闭。
 */
#ifndef BSP_BUTTON_H
#define BSP_BUTTON_H

#include <stdint.h>

/**
 * @brief 配置 PA15 内部上拉并释放 JTAG 引脚，保留 SWD。
 *
 * 调用：启动阶段一次；需要GD32库时钟接口可用。
 * 过程：启用AF/GPIOA，关闭JTAG保留SWD，再把PA15设为内部上拉输入；不启动中断。
 * 限制：只初始化引脚，不判断是否装有实体按键，不直接停止电机。
 */
void button_init(void);

/* Returns 1 once for each debounced SW1 press, otherwise returns 0. */
/**
 * @brief 每次消抖后的按下只返回一次 1，否则返回 0；按下/松开确认可能阻塞 20 ms。
 *
 * 输出：确认新按下返回1；保持按住、松开或抖动未确认返回0。
 * 过程：静态press_accepted记住本次按压；松开也需延时确认后才允许下一次事件。
 * 限制：主循环调用，不适合ISR；每次消抖等待20ms，当前机器人停机路径未调用它。
 */
uint8_t button_pressed_event(void);

#endif /* BSP_BUTTON_H */
