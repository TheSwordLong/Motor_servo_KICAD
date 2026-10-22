/*
 * motor_test.c — 历史 USART0 文本调试台，当前 CMake 未纳入 motor_test.c，main 不调用。
 * 它直接操作占空比/方向/刹车，不具备 robot 状态机互锁；不能与现役 Modbus 同时消费串口。
 * 旧频率表按 60 MHz 写死，与现役 pwm.c 的动态时钟/周期管理不一致，仅供历史参考。
 */
#include "motor_test.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "board_config.h"
#include "capture.h"
#include "gd32f30x.h"
#include "motor_io.h"
#include "motor_pins.h"
#include "pwm.h"
#include "systick.h"
#include "usart0.h"

/* Longest accepted command line, including the terminating NUL. */
#define MOTOR_CMD_LINE_SIZE  32U

/* ------------------------------------------------------------------ */
/* Debugger visible state                                              */
/* ------------------------------------------------------------------ */

volatile motor_debug_state_t g_motor_state[MOTOR_TEST_COUNT];
volatile uint8_t  g_motor_active = 0U;
volatile uint16_t g_motor_duty_tenth_pct = 0U;
volatile uint32_t g_motor_pwm_hz = 30000U;

/* ------------------------------------------------------------------ */
/* Private state                                                       */
/* ------------------------------------------------------------------ */

static char     cmd_line[MOTOR_CMD_LINE_SIZE];
static uint16_t cmd_len = 0U;
static uint32_t cmd_dropped = 0U;
static uint32_t cmd_processed = 0U;

static const uint16_t pwm_channel[MOTOR_TEST_COUNT] = {
    TIMER_CH_0, TIMER_CH_1, TIMER_CH_2, TIMER_CH_3
};

/*
 * Prescalers usable for TIMER3 (APB1 = 60 MHz) and the resulting counter
 * frequency after a 16-bit period. Keeping the prescaler small keeps the duty
 * resolution high, so 30 kHz stays on prescaler 0 with period 1999.
 */
static const struct {
    uint16_t prescaler;
    uint32_t clk_hz;
} pwm_freq_table[] = {
    {   0U, 60000000U },   /*  0.9 - 60.0 kHz, 0.018% duty steps  */
    {   1U, 30000000U },   /*  0.5 - 30.0 kHz, 0.037% duty steps  */
    {   4U, 12000000U },   /*  0.2 - 12.0 kHz, 0.091% duty steps  */
    {   9U,  6000000U },   /*  0.1 -  6.0 kHz, 0.18%  duty steps  */
    {  19U,  3000000U },   /* 45.8 - 3000 Hz,  0.37%  duty steps  */
    {  59U,  1000000U },   /* 15.3 - 1000 Hz,  1.1%   duty steps  */
    { 599U,   100000U },   /*  1.5 -  100 Hz, 11%     duty steps  */
};

#define PWM_FREQ_TABLE_COUNT (sizeof(pwm_freq_table) / sizeof(pwm_freq_table[0]))

/* ------------------------------------------------------------------ */
/* Console helpers                                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief 历史调试台文本输出，委托USART0阻塞发送。
 *
 * 输入：历史调试文本，交给usart0_write。
 * 限制：阻塞输出，文本不能插入现役Modbus通信。
 */
static void mt_write(const char *text)
{
    usart0_write(text);
}

/*
 * Newlib-nano printf cannot do floating point, so every duty is carried as
 * tenths of a percent (123 -> "12.3") and the decimal point is inserted by hand.
 */
/**
 * @brief 比较值/周期换算为0.1%显示单位，按整数四舍五入；周期0返回0。
 *
 * 输入：duty为比较值，period为完整周期计数。
 * 换算：(duty*1000+period/2)/period，输出0.1%单位；period0返回0。
 * 限制：用于历史显示，中间乘法未做通用溢出保护。
 */
static uint32_t duty_to_pct_x10(uint32_t duty, uint32_t period)
{
    if(0U == period) {
        return 0U;
    }

    return ((duty * 1000U) + (period / 2U)) / period;
}

/**
 * @brief 将0.1%单位格式化为带一位小数的百分比字符串，输出长度受out_size限制。
 *
 * 输入：out/out_size为输出缓冲，tenth_pct为0.1%单位。
 * 过程：snprintf格式化整数及小数位，缓冲不足时截断；不改变电机占空比。
 */
static void format_duty(char *out, size_t out_size, uint32_t tenth_pct)
{
    (void)snprintf(out, out_size, "%lu.%lu%%",
                   (unsigned long)(tenth_pct / 10U),
                   (unsigned long)(tenth_pct % 10U));
}

/**
 * @brief 返回软件方向状态的FWD/REV文本，不是实际转向测量。
 *
 * 输入：reverse为软件方向值，零返回FWD，非零返回REV静态字符串。
 * 限制：不分配内存、不读取GPIO或FG方向。
 */
static const char *dir_text(uint8_t reverse)
{
    return (0U != reverse) ? "REV" : "FWD";
}

/**
 * @brief 返回软件制动状态的ON/OFF文本，不是电平回读。
 *
 * 输入：brake为逻辑制动值，零返回OFF，非零返回ON静态字符串。
 * 限制：描述命令缓存，不是刹车动作确认。
 */
static const char *brake_text(uint8_t brake)
{
    return (0U != brake) ? "ON" : "OFF";
}

/* ------------------------------------------------------------------ */
/* PWM frequency                                                       */
/* ------------------------------------------------------------------ */

/*
 * Reconfigure TIMER3 for the requested frequency.
 *
 * Returns 1 and updates g_motor_pwm_hz on success, 0 when the request is outside
 * the achievable range.
 *
 * NOTE: this rewrites the prescaler, which changes the duty resolution. The
 * caller must re-apply every duty afterwards.
 */
/**
 * @brief 历史算法用固定60MHz频率表选择误差最小的分频/周期；不更新pwm.c内部period，勿混用。
 *
 * 输入：freq_hz非0；在固定60MHz分频表内选可达且误差最小的周期。
 * 过程：直接改PSC/ARR，软件更新并等待UP标志，更新历史频率缓存；不可达返回0。
 * 限制：pwm.c的period缓存未同步，重新调用历史占空比接口也不能保证正确比例。
 */
static uint8_t motor_test_set_pwm_freq(uint32_t freq_hz)
{
    uint32_t best = 0U;
    uint32_t best_err = 0xFFFFFFFFU;
    uint32_t i;
    uint32_t period;

    if(0U == freq_hz) {
        return 0U;
    }

    /* Pick the smallest prescaler (highest resolution) that can reach freq_hz. */
    for(i = 0U; i < PWM_FREQ_TABLE_COUNT; i++) {
        uint32_t p;
        uint32_t err;

        if(pwm_freq_table[i].clk_hz < freq_hz) {
            continue;
        }

        p = (pwm_freq_table[i].clk_hz + (freq_hz / 2U)) / freq_hz;
        if(0U == p) {
            p = 1U;
        }
        if(p > 65536U) {
            continue;
        }

        err = (pwm_freq_table[i].clk_hz / p > freq_hz)
              ? (pwm_freq_table[i].clk_hz / p) - freq_hz
              : freq_hz - (pwm_freq_table[i].clk_hz / p);

        if(err < best_err) {
            best_err = err;
            best = i;
        }
    }

    if(0xFFFFFFFFU == best_err) {
        return 0U;   /* too low for the largest prescaler */
    }

    period = (pwm_freq_table[best].clk_hz + (freq_hz / 2U)) / freq_hz;
    if(period > 65536U) {
        return 0U;
    }

    TIMER_PSC(PWM_TIMER) = pwm_freq_table[best].prescaler;
    TIMER_CAR(PWM_TIMER) = period - 1U;

    /* Reload the prescaler and the period now instead of at the next overflow. */
    timer_event_software_generate(PWM_TIMER, TIMER_EVENT_SRC_UPG);
    while(RESET == timer_flag_get(PWM_TIMER, TIMER_FLAG_UP)) {
    }
    timer_flag_clear(PWM_TIMER, TIMER_FLAG_UP);

    g_motor_pwm_hz = pwm_freq_table[best].clk_hz / period;
    return 1U;
}

/* Read back the actual compare value so the printed duty matches the hardware. */
/**
 * @brief 历史百分比输出并回读比较寄存器；motor须为0～3，返回compare计数。
 *
 * 输入：motor有效0～3，percent整数百分比。
 * 过程：调用pwm_set_duty_percent，再读CCR作为结果。
 * 限制：本函数不校验motor便索引通道表，只能由已检查调用者使用。
 */
static uint16_t motor_test_apply_duty(uint8_t motor, uint8_t percent)
{
    uint16_t compare;

    pwm_set_duty_percent(motor, percent);
    compare = (uint16_t)timer_channel_capture_value_register_read(PWM_TIMER, pwm_channel[motor]);

    return compare;
}

/* ------------------------------------------------------------------ */
/* Motor control                                                       */
/* ------------------------------------------------------------------ */

/**
 * @brief 更新输出及调试占空比缓存，不执行robot状态机保护。
 *
 * 过程：输出占空比，再按实际ARR读回计算显示值，写全局及单路历史缓存。
 * 限制：缓存不等于电机转速，不执行制动或故障保护。
 */
static void motor_test_apply_duty_only(uint8_t motor, uint8_t percent)
{
    uint16_t compare = motor_test_apply_duty(motor, percent);
    uint32_t period = (uint32_t)TIMER_CAR(PWM_TIMER) + 1U;

    g_motor_duty_tenth_pct = (uint16_t)duty_to_pct_x10(compare, period);
    g_motor_state[motor].duty_tenth_pct = g_motor_duty_tenth_pct;
}

/**
 * @brief 直接改DIR并更新历史缓存，不负责先停PWM或等待电机静止。
 *
 * 输入：motor及reverse；调用motor_set_dir后把布尔方向写历史缓存。
 * 限制：必须保证motor有效，且外部保证静止；函数不会停PWM。
 */
static void motor_test_apply_dir_only(uint8_t motor, uint8_t reverse)
{
    motor_set_dir(motor, reverse);
    g_motor_state[motor].reverse = (0U != reverse) ? 1U : 0U;
}

/**
 * @brief 直接改BRAKE并更新历史缓存，不执行运动互锁。
 *
 * 输入：motor及brake；调用motor_set_brake后更新历史缓存。
 * 限制：不检查占空比或运动状态，不能代替robot的BRAKE命令。
 */
static void motor_test_apply_brake_only(uint8_t motor, uint8_t brake)
{
    motor_set_brake(motor, brake);
    g_motor_state[motor].brake = (0U != brake) ? 1U : 0U;
}

/* Select the motor the m<n> commands act on. Returns 1 on success. */
/**
 * @brief 选择文本命令控制对象，0～3成功返回1，越界返回0。
 *
 * 输入：历史内部motor编号0～3，文本m1～m4先减1再调用。
 * 结果：成功更新g_motor_active返回1，越界保持原选择返回0。
 */
static uint8_t motor_test_select(uint8_t motor)
{
    if(motor >= MOTOR_TEST_COUNT) {
        return 0U;
    }

    g_motor_active = motor;
    return 1U;
}

/**
 * @brief 历史停止语义：PWM清零、方向正向、刹车释放；与现役robot_stop_all制动语义不同。
 *
 * 过程：先pwm_stop_all，再四路方向设0、brake设0，清显示占空比。
 * 重要：历史brake=0是释放，不是锁定；不等待静止、不保存、不清robot状态。
 */
void motor_test_stop_all(void)
{
    uint8_t motor;

    pwm_stop_all();

    for(motor = 0U; motor < MOTOR_TEST_COUNT; motor++) {
        motor_test_apply_dir_only(motor, 0U);
        motor_test_apply_brake_only(motor, 0U);
        g_motor_state[motor].duty_tenth_pct = 0U;
    }

    g_motor_duty_tenth_pct = 0U;
}

/* ------------------------------------------------------------------ */
/* Console output                                                      */
/* ------------------------------------------------------------------ */

/**
 * @brief 输出历史文本命令帮助，不代表现役固件支持这些串口文本命令。
 *
 * 过程：USART0输出文本命令、示例和历史提示。
 * 限制：帮助中的默认频率/编译描述是旧文本，不能作为当前机器人构建事实。
 */
void motor_test_print_help(void)
{
    mt_write("\r\nMotor debug console (USART0 115200 8N1)\r\n");
    mt_write("  ?              this help\r\n");
    mt_write("  s              status of all four motors\r\n");
    mt_write("  x              stop all motors (duty 0)\r\n");
    mt_write("  m<n>           select motor n = 1..4\r\n");
    mt_write("  d<0|1>         direction: 0 = forward, 1 = reverse\r\n");
    mt_write("  p<0..100>      duty in percent (p30 = 30.0%)\r\n");
    mt_write("  b<0|1>         brake: 0 = release, 1 = brake\r\n");
    mt_write("  a<0..100>      apply direction + duty to ALL motors\r\n");
    mt_write("  f<Hz>          PWM frequency, e.g. f20000 (default 30000)\r\n");
    mt_write("  z              re-apply direction + duty to every motor\r\n");
    mt_write("  v              version\r\n");
    mt_write("\r\nExamples\r\n");
    mt_write("  m1 d0 p30     motor 1 forward at 30%\r\n");
    mt_write("  m3 d1 p50     motor 3 reverse at 50%\r\n");
    mt_write("  x             stop everything\r\n");
    mt_write("\r\nWatch in the debugger: g_motor_state[0..3], g_motor_duty_tenth_pct,\r\n");
    mt_write("g_motor_active, g_motor_pwm_hz, g_fg[].speed_rpm\r\n");
    mt_write("\r\nSAFETY: DIR/BRAKE/PWM are live as soon as a command is accepted.\r\n");
    mt_write("Keep the motor supply off until the PWM levels are verified.\r\n\r\n");
}

/**
 * @brief 输出历史缓存和计数；rpm字段此文件没有周期刷新，不能当作现役实时转速。
 *
 * 过程：格式化频率、选中路、命令数及四路缓存字段并发送。
 * 限制：本文件没有持续刷新rpm字段；实际测速应看capture模块。
 */
void motor_test_print_status(void)
{
    char line[128];
    char duty_text[12];
    uint8_t motor;

    (void)snprintf(line, sizeof(line),
                   "PWM %lu Hz  sel=M%u  cmd=%lu drop=%lu  tick=%lu ms\r\n",
                   (unsigned long)g_motor_pwm_hz,
                   (unsigned)(g_motor_active + 1U),
                   (unsigned long)cmd_processed,
                   (unsigned long)cmd_dropped,
                   (unsigned long)sys_tick_ms);
    mt_write(line);

    for(motor = 0U; motor < MOTOR_TEST_COUNT; motor++) {
        format_duty(duty_text, sizeof(duty_text), g_motor_state[motor].duty_tenth_pct);

        (void)snprintf(line, sizeof(line),
                       "M%u %s %s duty=%-7s rpm=%-6lu cmds=%lu\r\n",
                       (unsigned)(motor + 1U),
                       dir_text(g_motor_state[motor].reverse),
                       brake_text(g_motor_state[motor].brake),
                       duty_text,
                       (unsigned long)g_motor_state[motor].rpm,
                       (unsigned long)g_motor_state[motor].commands);
        mt_write(line);
    }
}

/* ------------------------------------------------------------------ */
/* Command parsing                                                     */
/* ------------------------------------------------------------------ */

/*
 * Parse one unsigned decimal value. Returns 1 and writes the result on success,
 * 0 when the text is empty, out of range or not a number.
 */
/**
 * @brief 历史十进制解析，逐字符检查和max比较；未单独保护乘10的uint32溢出，仅限受控测试输入。
 *
 * 输入：NUL结尾text、max上限、out输出；空文本/非数字/超过max返回0。
 * 过程：逐字符十进制累加，成功写out返回1。
 * 限制：text不能NULL；乘法溢出未单独检测，不能直接当不可信网络整数解析器。
 */
static uint8_t parse_u32(const char *text, uint32_t max, uint32_t *out)
{
    uint32_t value = 0U;
    size_t index = 0U;

    if(('\0' == text[0]) || (NULL == out)) {
        return 0U;
    }

    while('\0' != text[index]) {
        if((text[index] < '0') || (text[index] > '9')) {
            return 0U;
        }

        value = (value * 10U) + (uint32_t)(text[index] - '0');
        if(value > max) {
            return 0U;
        }

        index++;
    }

    *out = value;
    return 1U;
}

/**
 * @brief 解析单条NUL结尾历史命令并直接操作输出；不提供现役协议的保存/故障/序号保护。
 *
 * 输入：一条NUL结尾文本命令，按首字符进入?、s、v、x、z、m、d、b、p、a、f分支。
 * 作用：选择电机、直接输出、查询、改历史频率；参数由parse_u32检查。
 * 限制：没有现役命令序号、dirty保存或运动互锁；所有引脚修改立即生效。
 */
static void motor_test_handle_command(const char *cmd)
{
    char reply[96];
    char duty_text[12];
    uint32_t value;
    uint8_t motor;

    switch(cmd[0]) {
    case '\0':
        return;

    case '?':
        motor_test_print_help();
        return;

    case 's':
        motor_test_print_status();
        return;

    case 'v':
        mt_write("motor_test: 4-channel PWM debug shell, 30 kHz default, build "
                 __DATE__ " " __TIME__ "\r\n");
        mt_write("test modes: RS-485/CAN/USART2 are NOT compiled in this build\r\n");
        return;

    case 'x':
        motor_test_stop_all();
        mt_write("all motors stopped\r\n");
        return;

    case 'z':
        /* Re-apply direction and duty to every motor, e.g. after f<Hz>. */
        for(value = 0U; value < MOTOR_TEST_COUNT; value++) {
            motor_test_apply_dir_only((uint8_t)value, g_motor_state[value].reverse);
            motor_test_apply_brake_only((uint8_t)value, g_motor_state[value].brake);
            motor_test_apply_duty_only((uint8_t)value, (uint8_t)(g_motor_state[value].duty_tenth_pct / 10U));
        }
        mt_write("direction and duty re-applied to all motors\r\n");
        return;

    case 'm':
        if(0U == parse_u32(&cmd[1], 4U, &value) || (0U == value)) {
            mt_write("usage: m1 | m2 | m3 | m4\r\n");
            return;
        }
        (void)motor_test_select((uint8_t)(value - 1U));
        (void)snprintf(reply, sizeof(reply), "selected motor %lu\r\n", (unsigned long)value);
        mt_write(reply);
        return;

    case 'd': {
        motor = g_motor_active;

        if(0U == parse_u32(&cmd[1], 1U, &value)) {
            mt_write("usage: d0 (forward) | d1 (reverse)\r\n");
            return;
        }
        motor_test_apply_dir_only(motor, (uint8_t)value);
        g_motor_state[motor].commands++;
        (void)snprintf(reply, sizeof(reply), "M%u dir=%s\r\n",
                       (unsigned)(motor + 1U), dir_text((uint8_t)value));
        mt_write(reply);
        return;
    }

    case 'b': {
        motor = g_motor_active;

        if(0U == parse_u32(&cmd[1], 1U, &value)) {
            mt_write("usage: b0 (release) | b1 (brake)\r\n");
            return;
        }
        motor_test_apply_brake_only(motor, (uint8_t)value);
        g_motor_state[motor].commands++;
        (void)snprintf(reply, sizeof(reply), "M%u brake=%s\r\n",
                       (unsigned)(motor + 1U), brake_text((uint8_t)value));
        mt_write(reply);
        return;
    }

    case 'p': {
        motor = g_motor_active;

        if(0U == parse_u32(&cmd[1], 100U, &value)) {
            mt_write("usage: p0 .. p100 (percent)\r\n");
            return;
        }
        motor_test_apply_duty_only(motor, (uint8_t)value);
        g_motor_state[motor].commands++;
        format_duty(duty_text, sizeof(duty_text), g_motor_state[motor].duty_tenth_pct);
        (void)snprintf(reply, sizeof(reply), "M%u duty=%s (compare=%lu/%lu)\r\n",
                       (unsigned)(motor + 1U), duty_text,
                       (unsigned long)timer_channel_capture_value_register_read(PWM_TIMER, pwm_channel[motor]),
                       (unsigned long)(TIMER_CAR(PWM_TIMER) + 1U));
        mt_write(reply);
        return;
    }

    case 'a': {
        if(0U == parse_u32(&cmd[1], 100U, &value)) {
            mt_write("usage: a0 .. a100 (percent, all motors)\r\n");
            return;
        }

        for(motor = 0U; motor < MOTOR_TEST_COUNT; motor++) {
            motor_test_apply_dir_only(motor, 0U);
            motor_test_apply_brake_only(motor, 0U);
            motor_test_apply_duty_only(motor, (uint8_t)value);
            g_motor_state[motor].commands++;
        }

        format_duty(duty_text, sizeof(duty_text), g_motor_duty_tenth_pct);
        (void)snprintf(reply, sizeof(reply), "all motors forward at %s\r\n", duty_text);
        mt_write(reply);
        return;
    }

    case 'f':
        if(0U == parse_u32(&cmd[1], 100000U, &value) || (0U == value)) {
            mt_write("usage: f1000 .. f60000 (Hz)\r\n");
            return;
        }

        if(0U == motor_test_set_pwm_freq(value)) {
            (void)snprintf(reply, sizeof(reply),
                           "cannot reach %lu Hz (range about 1.5 Hz .. 60 kHz)\r\n",
                           (unsigned long)value);
            mt_write(reply);
            return;
        }

        /*
         * The prescaler changed, so every compare register is stale. Re-apply
         * the stored duty of each motor and report the rounded result.
         */
        for(motor = 0U; motor < MOTOR_TEST_COUNT; motor++) {
            motor_test_apply_duty_only(motor, (uint8_t)(g_motor_state[motor].duty_tenth_pct / 10U));
        }

        (void)snprintf(reply, sizeof(reply), "PWM set to %lu Hz (period=%lu)\r\n",
                       (unsigned long)g_motor_pwm_hz,
                       (unsigned long)(TIMER_CAR(PWM_TIMER) + 1U));
        mt_write(reply);
        return;

    default:
        (void)snprintf(reply, sizeof(reply), "unknown command '%c' - type ? for help\r\n", cmd[0]);
        mt_write(reply);
        return;
    }
}

/* ------------------------------------------------------------------ */
/* Public entry points                                                 */
/* ------------------------------------------------------------------ */

/**
 * @brief 历史调试台初始化并可选输出测试PWM；当前main不调用，本函数中的旧频率显示仅供参考。
 *
 * 过程：清历史状态，初始化GPIO/PWM，历史stop_all释放刹车，再打印帮助状态。
 * 可选：MOTOR_TEST_SELFTEST_PWM=1时会直接给四路测试占空比，默认关闭。
 * 限制：当前构建不运行此函数，不能用这里的启动语义解释robot_port。
 */
void motor_test_init(void)
{
    uint8_t motor;

    for(motor = 0U; motor < MOTOR_TEST_COUNT; motor++) {
        g_motor_state[motor].duty_tenth_pct = 0U;
        g_motor_state[motor].reverse = 0U;
        g_motor_state[motor].brake = 0U;
        g_motor_state[motor].rpm = 0U;
        g_motor_state[motor].commands = 0U;
    }

    cmd_len = 0U;
    cmd_dropped = 0U;
    cmd_processed = 0U;

    /* 当前motor_io_init先制动；后面的历史stop_all会再次释放刹车。 */
    motor_io_init();
    pwm_init();

    /* 历史显示仍按60MHz推算；当前pwm_init实际初始化25kHz，此调试台未与其同步。 */
    g_motor_pwm_hz = 60000000U / ((uint32_t)TIMER_CAR(PWM_TIMER) + 1U);
    g_motor_active = 0U;

    /* Everything stopped: no motion until a command asks for it. */
    motor_test_stop_all();

    mt_write("\r\n=== Motor control debug test ===\r\n");

#if (1 == MOTOR_TEST_SELFTEST_PWM)
    {
        char line[96];

        for(motor = 0U; motor < MOTOR_TEST_COUNT; motor++) {
            motor_test_apply_duty_only(motor, MOTOR_TEST_SELFTEST_DUTY_PCT);
        }

        (void)snprintf(line, sizeof(line),
                       "SELF TEST: PD12..PD15 = %u%% at %lu Hz, verify with a meter/scope\r\n",
                       (unsigned)MOTOR_TEST_SELFTEST_DUTY_PCT,
                       (unsigned long)g_motor_pwm_hz);
        mt_write(line);
    }
#else
    mt_write("all four channels at 0% - nothing moves until you send a command\r\n");
#endif

    mt_write("PWM: PD12..PD15 (TIMER3_CH0..CH3), DIR/BRK: see BSP/motor_pins.h\r\n");
    motor_test_print_help();
    motor_test_print_status();
}

/**
 * @brief 消费USART0到换行后执行历史命令；长行清缓存继续接收，不应与RTU组帧共用队列。
 *
 * 过程：消费USART0字节，CR/LF结束时执行缓存命令；不带换行不执行。
 * 长行：满时清cmd_len并增加dropped，后续字符仍可能形成尾段命令，并非一直丢到换行。
 * 限制：当前未启用；不要与现役RTU同时作为接收队列消费者。
 */
void motor_test_update(void)
{
    uint8_t rx_byte;

    while(0U != usart0_rx_pop(&rx_byte)) {
        if(('\n' == rx_byte) || ('\r' == rx_byte)) {
            if(0U != cmd_len) {
                cmd_line[cmd_len] = '\0';
                cmd_processed++;
                motor_test_handle_command(cmd_line);
                cmd_len = 0U;
            }
        } else if(cmd_len < (MOTOR_CMD_LINE_SIZE - 1U)) {
            cmd_line[cmd_len] = (char)rx_byte;
            cmd_len++;
        } else {
            /* Overlong line: drop it and resynchronise on the next newline. */
            cmd_len = 0U;
            cmd_dropped++;
        }
    }
}
