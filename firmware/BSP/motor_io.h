/*
 * motor_io.h — 四路 DIR/BRAKE GPIO 输出，通道 0～3 对应物理 M1～M4，实际引脚见 motor_pins.h。
 * 当前均为推挽输出；DIR 1=高，BRAKE 1=低电平制动。输出命令不能证明端子实际电压。
 */
#ifndef BSP_MOTOR_IO_H
#define BSP_MOTOR_IO_H

#include <stdint.h>

/*
 * DIR / BRAKE control for the four motors. Plain GPIO outputs only.
 *
 * Motor index: 0..3 maps to motor 1..4 (see BSP/motor_pins.h for the pinout).
 *
 * Levels:
 *   DIR   low  = forward, high = reverse
 *   BRAKE low  = brake,   high = released
 */

/* Configure all DIR and BRAKE pins. DIR -> forward (low), BRAKE -> engaged (low). */
/**
 * @brief 先预置 DIR/BRAKE 为低，再配置为推挽输出，避免切换输出模式时先出现释放电平。
 *
 * 过程：启用A～E端口时钟，先写安全输出锁存值，再配置八个DIR/BRAKE推挽引脚。
 * 初态：DIR低、BRAKE低；PWM由独立模块初始化。
 * 限制：没有开漏输出或端子回读；实际电压须测量确认。
 */
void motor_io_init(void);

/* 0 = forward (low), non-zero = reverse (high). */
/**
 * @brief motor=0～3；reverse 非零输出高、零输出低；无效通道不操作，不负责停止或换向等待。
 *
 * 输入：motor=0～3；reverse=0输出低，任意非零输出高；其他motor值无动作。
 * 过程：只写相应GPIO置位/复位寄存器，不检查速度、刹车或故障。
 * 限制：直接调用者必须保证允许换向；现役路径由drive先停PWM再调用。
 */
void motor_set_dir(uint8_t motor, uint8_t reverse);

/* 0 = release (high), non-zero = brake (low). */
/**
 * @brief motor=0～3；brake 非零输出低电平制动，零输出高电平释放；无效通道不操作。
 *
 * 输入：motor=0～3；brake=1表示制动（低），0表示释放（高）。
 * 过程：只改BRAKE输出位，不清PWM也不等待机械停止。
 * 限制：必须与PWM和状态机协调；软件值不等于实物制动已生效。
 */
void motor_set_brake(uint8_t motor, uint8_t brake);

/* Put every motor back into the safe power-on state (forward, brake engaged). */
/**
 * @brief 预置四路方向低、刹车低；调用前须启用 GPIO 时钟，本函数不配置 PWM。
 *
 * 过程：将四路DIR和BRAKE输出锁存位都清低。
 * 前提：相关GPIO时钟已开启；本函数不初始化引脚模式。
 * 限制：会改方向，不用于正在运动的CPU异常停车；异常路径另用robot_emergency_stop保持DIR。
 */
void motor_io_safe_state(void);

#endif /* BSP_MOTOR_IO_H */
