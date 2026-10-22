/*
 * motor_test.h — 历史 USART0 文本调试台，当前 CMake 未纳入 motor_test.c，main 不调用。
 * 它直接操作占空比/方向/刹车，不具备 robot 状态机互锁；不能与现役 Modbus 同时消费串口。
 * 旧频率表按 60 MHz 写死，与现役 pwm.c 的动态时钟/周期管理不一致，仅供历史参考。
 */
#ifndef BSP_MOTOR_TEST_H
#define BSP_MOTOR_TEST_H

#include <stdint.h>

/*
 * Four-motor debug and test shell.
 *
 * Purpose: bring up and exercise the motor power stage (PWM / DIR / BRAKE) from
 * a serial terminal on USART0 (PA9/PA10, 115200 8N1) instead of running any
 * communication-interface test. FG speed feedback is read from the input
 * capture driver (BSP/capture.c).
 *
 * Pinout: see BSP/motor_pins.h
 *   PWM   PD12..PD15  TIMER3_CH0..CH3
 *   DIR   PE15 / PC5 / PD3 / PB8
 *   BRAKE PE13 / PA7 / PD2 / PB5
 *   FG    PB10 / PE11 / PB4 / PA0
 *
 * Levels:
 *   DIR   low  = forward, high = reverse
 *   BRAKE low  = brake,   high = released
 *
 * Safety: every motor starts stopped (0% duty) and brake released. Nothing moves
 * until a command asks for it.
 */

/* Number of motors handled by the shell (index 0..3 = motor 1..4). */
#define MOTOR_TEST_COUNT        4U

/* PWM period of the initial 30 kHz configuration (TIMER3, APB1 = 60 MHz). */
#define MOTOR_TEST_PWM_PERIOD   1999U

/* Duty set by the startup self-test when MOTOR_TEST_SELFTEST_PWM is enabled. */
#define MOTOR_TEST_SELFTEST_DUTY_PCT 50U

/*
 * Per-motor state, refreshed on every command and kept volatile so it can be
 * watched live in the debugger:
 *
 *   print g_motor_state[0].duty_tenth_pct   0..1000 = 0.0% .. 100.0%
 *   print g_motor_state[0].reverse          0 = forward, 1 = reverse
 *   print g_motor_state[0].brake            0 = released, 1 = braked
 *   print g_motor_state[0].rpm              measured FG speed
 */
typedef struct {
    volatile uint16_t duty_tenth_pct;   /* commanded duty, tenths of a percent */
    volatile uint8_t  reverse;          /* 0 = forward, 1 = reverse */
    volatile uint8_t  brake;            /* 0 = released, 1 = braked */
    volatile uint32_t rpm;              /* last measured speed from FG capture */
    volatile uint32_t commands;         /* commands that touched this motor */
} motor_debug_state_t;

extern volatile motor_debug_state_t g_motor_state[MOTOR_TEST_COUNT];

/* Currently selected motor for the m<n> commands: 0..3. */
extern volatile uint8_t g_motor_active;

/* Duty applied to (re)initialisation, in tenths of a percent. */
extern volatile uint16_t g_motor_duty_tenth_pct;

/* PWM frequency in Hz as configured by the f command. */
extern volatile uint32_t g_motor_pwm_hz;

/*
 * Configure PWM / DIR / BRAKE and the debug state, then (when
 * MOTOR_TEST_SELFTEST_PWM is 1) apply MOTOR_TEST_SELFTEST_DUTY_PCT to all four
 * channels so the output can be checked with a meter or scope.
 */
/**
 * @brief 历史调试台初始化并可选输出测试PWM；当前main不调用，本函数中的旧频率显示仅供参考。
 *
 * 过程：清历史状态，初始化GPIO/PWM，历史stop_all释放刹车，再打印帮助状态。
 * 可选：MOTOR_TEST_SELFTEST_PWM=1时会直接给四路测试占空比，默认关闭。
 * 限制：当前构建不运行此函数，不能用这里的启动语义解释robot_port。
 */
void motor_test_init(void);

/*
 * Drain USART0 and consume any complete command lines.
 * Call once per main loop iteration.
 */
/**
 * @brief 消费USART0到换行后执行历史命令；长行清缓存继续接收，不应与RTU组帧共用队列。
 *
 * 过程：消费USART0字节，CR/LF结束时执行缓存命令；不带换行不执行。
 * 长行：满时清cmd_len并增加dropped，后续字符仍可能形成尾段命令，并非一直丢到换行。
 * 限制：当前未启用；不要与现役RTU同时作为接收队列消费者。
 */
void motor_test_update(void);

/* Stop every motor: 0% duty on all four channels. */
/**
 * @brief 历史停止语义：PWM清零、方向正向、刹车释放；与现役robot_stop_all制动语义不同。
 *
 * 过程：先pwm_stop_all，再四路方向设0、brake设0，清显示占空比。
 * 重要：历史brake=0是释放，不是锁定；不等待静止、不保存、不清robot状态。
 */
void motor_test_stop_all(void);

/* Print the one-line status table over USART0. */
/**
 * @brief 输出历史缓存和计数；rpm字段此文件没有周期刷新，不能当作现役实时转速。
 *
 * 过程：格式化频率、选中路、命令数及四路缓存字段并发送。
 * 限制：本文件没有持续刷新rpm字段；实际测速应看capture模块。
 */
void motor_test_print_status(void);

/* Print the command help over USART0. */
/**
 * @brief 输出历史文本命令帮助，不代表现役固件支持这些串口文本命令。
 *
 * 过程：USART0输出文本命令、示例和历史提示。
 * 限制：帮助中的默认频率/编译描述是旧文本，不能作为当前机器人构建事实。
 */
void motor_test_print_help(void);

#endif /* BSP_MOTOR_TEST_H */
