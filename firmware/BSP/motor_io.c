/*
 * motor_io.c — 四路 DIR/BRAKE GPIO 输出，通道 0～3 对应物理 M1～M4，实际引脚见 motor_pins.h。
 * 当前均为推挽输出；DIR 1=高，BRAKE 1=低电平制动。输出命令不能证明端子实际电压。
 */
#include "motor_io.h"

#include "gd32f30x.h"
#include "motor_pins.h"

/*
 * Every DIR and BRAKE pin is distinct (no pin is used for both DIR and BRAKE),
 * so motor_io_safe_state() can drive them all in one call per port.
 */
/**
 * @brief 先预置 DIR/BRAKE 为低，再配置为推挽输出，避免切换输出模式时先出现释放电平。
 *
 * 过程：启用A～E端口时钟，先写安全输出锁存值，再配置八个DIR/BRAKE推挽引脚。
 * 初态：DIR低、BRAKE低；PWM由独立模块初始化。
 * 限制：没有开漏输出或端子回读；实际电压须测量确认。
 */
void motor_io_init(void)
{
    /* M2_DIR=PC5; M3_DIR=PD3; M1_DIR=PE15, M1_BRAKE=PE13 -> GPIOA/GPIOB/GPIOC/GPIOD/GPIOE */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);

    /*
     * Write the output data BEFORE switching the pins to output mode, so the
     * first driven level is already the safe one:
     *   DIR   -> low  (forward)
     *   BRAKE -> low (engaged)
     */
    motor_io_safe_state();

    gpio_init(M1_DIR_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, M1_DIR_PIN);
    gpio_init(M2_DIR_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, M2_DIR_PIN);
    gpio_init(M3_DIR_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, M3_DIR_PIN);
    gpio_init(M4_DIR_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, M4_DIR_PIN);

    gpio_init(M1_BRAKE_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, M1_BRAKE_PIN);
    gpio_init(M2_BRAKE_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, M2_BRAKE_PIN);
    gpio_init(M3_BRAKE_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, M3_BRAKE_PIN);
    gpio_init(M4_BRAKE_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, M4_BRAKE_PIN);
}

/**
 * @brief motor=0～3；reverse 非零输出高、零输出低；无效通道不操作，不负责停止或换向等待。
 *
 * 输入：motor=0～3；reverse=0输出低，任意非零输出高；其他motor值无动作。
 * 过程：只写相应GPIO置位/复位寄存器，不检查速度、刹车或故障。
 * 限制：直接调用者必须保证允许换向；现役路径由drive先停PWM再调用。
 */
void motor_set_dir(uint8_t motor, uint8_t reverse)
{
    switch(motor) {
    case 0U:
        reverse ? gpio_bit_set(M1_DIR_PORT, M1_DIR_PIN) : gpio_bit_reset(M1_DIR_PORT, M1_DIR_PIN);
        break;
    case 1U:
        reverse ? gpio_bit_set(M2_DIR_PORT, M2_DIR_PIN) : gpio_bit_reset(M2_DIR_PORT, M2_DIR_PIN);
        break;
    case 2U:
        reverse ? gpio_bit_set(M3_DIR_PORT, M3_DIR_PIN) : gpio_bit_reset(M3_DIR_PORT, M3_DIR_PIN);
        break;
    case 3U:
        reverse ? gpio_bit_set(M4_DIR_PORT, M4_DIR_PIN) : gpio_bit_reset(M4_DIR_PORT, M4_DIR_PIN);
        break;
    default:
        break;
    }
}

/**
 * @brief motor=0～3；brake 非零输出低电平制动，零输出高电平释放；无效通道不操作。
 *
 * 输入：motor=0～3；brake=1表示制动（低），0表示释放（高）。
 * 过程：只改BRAKE输出位，不清PWM也不等待机械停止。
 * 限制：必须与PWM和状态机协调；软件值不等于实物制动已生效。
 */
void motor_set_brake(uint8_t motor, uint8_t brake)
{
    /* brake ON = low, brake OFF/released = high */
    switch(motor) {
    case 0U:
        brake ? gpio_bit_reset(M1_BRAKE_PORT, M1_BRAKE_PIN) : gpio_bit_set(M1_BRAKE_PORT, M1_BRAKE_PIN);
        break;
    case 1U:
        brake ? gpio_bit_reset(M2_BRAKE_PORT, M2_BRAKE_PIN) : gpio_bit_set(M2_BRAKE_PORT, M2_BRAKE_PIN);
        break;
    case 2U:
        brake ? gpio_bit_reset(M3_BRAKE_PORT, M3_BRAKE_PIN) : gpio_bit_set(M3_BRAKE_PORT, M3_BRAKE_PIN);
        break;
    case 3U:
        brake ? gpio_bit_reset(M4_BRAKE_PORT, M4_BRAKE_PIN) : gpio_bit_set(M4_BRAKE_PORT, M4_BRAKE_PIN);
        break;
    default:
        break;
    }
}

/**
 * @brief 预置四路方向低、刹车低；调用前须启用 GPIO 时钟，本函数不配置 PWM。
 *
 * 过程：将四路DIR和BRAKE输出锁存位都清低。
 * 前提：相关GPIO时钟已开启；本函数不初始化引脚模式。
 * 限制：会改方向，不用于正在运动的CPU异常停车；异常路径另用robot_emergency_stop保持DIR。
 */
void motor_io_safe_state(void)
{
    /* DIR all forward (low). */
    gpio_bit_reset(GPIOE, M1_DIR_PIN);
    gpio_bit_reset(GPIOC, M2_DIR_PIN);
    gpio_bit_reset(GPIOD, M3_DIR_PIN);
    gpio_bit_reset(GPIOB, M4_DIR_PIN);

    /* BRAKE all engaged (low). */
    gpio_bit_reset(GPIOE, M1_BRAKE_PIN);
    gpio_bit_reset(GPIOA, M2_BRAKE_PIN);
    gpio_bit_reset(GPIOD, M3_BRAKE_PIN);
    gpio_bit_reset(GPIOB, M4_BRAKE_PIN);
}
