/*
 * rs485.c — 保留的 USART1 半双工 RS485 驱动：PA2 TX、PA3 RX、PA1 DE。
 * 当前机器人 Modbus 使用 USART0/CH340，不调用本模块初始化。中断生产字节，主循环消费。
 * 发送为阻塞式，延时为软件近似值；不要把历史测试接口当作当前机器人通信入口。
 */
#include "rs485.h"

#include "board_config.h"
#include "gd32f30x.h"
#include "systick.h"

/* ------------------------------------------------------------------ */
/* Ring buffer: single producer (ISR), single consumer (main loop).     */
/* ------------------------------------------------------------------ */

static volatile uint8_t  rx_buf[BOARD_RS485_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0U;   /* written by the ISR only */
static volatile uint16_t rx_tail = 0U;   /* written by the consumer only */

static volatile rs485_stats_t stats;

/* ------------------------------------------------------------------ */
/* Receive-only monitoring globals (see rs485.h)                       */
/* ------------------------------------------------------------------ */

volatile char     g_rs485_line[BOARD_RS485_LINE_BUF_SIZE];
volatile uint32_t g_rs485_line_len = 0U;
volatile uint32_t g_rs485_rx_total = 0U;
volatile uint32_t g_rs485_rx_dropped = 0U;

/**
 * @brief 返回编译时 RS485_TEST_MODE_RX 选择，不代表硬件当前接收状态。
 *
 * 输入：无；使用预处理宏选择返回值。
 * 限制：只说明历史测试模式编译选择，不读取DE，也不禁止其他函数发送。
 */
uint8_t rs485_is_rx_only(void)
{
#if (1 == RS485_TEST_MODE_RX)
    return 1U;
#else
    return 0U;
#endif
}

/* ------------------------------------------------------------------ */
/* Microsecond busy-wait for the DE guard intervals                    */
/* ------------------------------------------------------------------ */

/*
 * Calibrated for 120 MHz. This is a coarse busy-wait used only for the 50 us
 * DE windows, not for anything timing-critical.
 */
/**
 * @brief 按循环次数粗略延时，非校准微秒定时器，实际时间依赖时钟和优化。
 *
 * 输入：microseconds转换成microseconds*10次易失循环。
 * 限制：没有硬件定时精度保证，值过大可溢出；仅适合当前小延时参数。
 */
static void rs485_delay_us(uint32_t microseconds)
{
    volatile uint32_t loops = microseconds * 10U;

    while(0U != loops) {
        loops--;
    }
}

/* ------------------------------------------------------------------ */
/* DE control                                                          */
/* ------------------------------------------------------------------ */

/**
 * @brief 按编译极性使能发送DE；不发送数据。
 *
 * 过程：按BOARD_RS485_DE_TX_LEVEL输出发送使能电平。
 * 限制：不等待隔离器建立时间，实际发送函数在本调用后另加延时。
 */
void rs485_de_assert(void)
{
#if (1U == BOARD_RS485_DE_TX_LEVEL)
    gpio_bit_set(BOARD_RS485_GPIO_PORT, BOARD_RS485_DE_PIN);
#else
    gpio_bit_reset(BOARD_RS485_GPIO_PORT, BOARD_RS485_DE_PIN);
#endif
}

/**
 * @brief 恢复接收DE；强制空闲高测试宏启用时例外，不代表已验证收发器电平。
 *
 * 过程：正常按DE反极性回接收；FORCE_DE_IDLE_HIGH=1时固定高。
 * 限制：测试强制高可能使接收关闭；它不是通信状态回读。
 */
void rs485_de_release(void)
{
#if (1U == BOARD_RS485_FORCE_DE_IDLE_HIGH)
    /* Diagnostic override: keep the DE line high instead of returning to receive. */
    gpio_bit_set(BOARD_RS485_GPIO_PORT, BOARD_RS485_DE_PIN);
#else
#if (1U == BOARD_RS485_DE_TX_LEVEL)
    gpio_bit_reset(BOARD_RS485_GPIO_PORT, BOARD_RS485_DE_PIN);
#else
    gpio_bit_set(BOARD_RS485_GPIO_PORT, BOARD_RS485_DE_PIN);
#endif
#endif
}

/* ------------------------------------------------------------------ */
/* Initialisation                                                      */
/* ------------------------------------------------------------------ */

/**
 * @brief baudrate=0使用9600默认值；配置USART1和RX中断，清接收队列及统计。
 *
 * 输入：baudrate为波特率，0选择BOARD_RS485_BAUDRATE。
 * 过程：预置DE接收，再配TX/RX和USART1 8N1，禁用RTS/CTS，启用RBNE中断。
 * 完成：清环形队列和统计，记录实际APB1时钟；历史文本/握手状态不在这里全部复位。
 */
void rs485_init(uint32_t baudrate)
{
    uint32_t apb1_hz;

    if(0U == baudrate) {
        baudrate = BOARD_RS485_BAUDRATE;
    }

    rcu_periph_clock_enable(BOARD_RS485_GPIO_CLOCK);

    /*
     * Latch DE low BEFORE switching the pin to output, so enabling the driver
     * cannot produce a glitch on the bus.
     */
    rs485_de_release();
    gpio_init(BOARD_RS485_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, BOARD_RS485_DE_PIN);

    /*
     * PA2 = USART1_TX (idles high), PA3 = USART1_RX. No remap.
     *
     * PA3 is a floating input: the transceiver's RO output always drives it, so
     * no internal pull is needed. Note that an internal pull would NOT replace
     * the A/B differential idle bias on the bus anyway.
     */
    gpio_init(BOARD_RS485_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, BOARD_RS485_TX_PIN);
    gpio_init(BOARD_RS485_GPIO_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, BOARD_RS485_RX_PIN);

    rcu_periph_clock_enable(RCU_USART1);

    usart_deinit(USART1);
    usart_baudrate_set(USART1, baudrate);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    /* Hardware RTS/CTS stays off: PA1 is software-controlled DE, not USART1_RTS. */
    usart_hardware_flow_rts_config(USART1, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART1, USART_CTS_DISABLE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_enable(USART1);

    usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);
    usart_interrupt_enable(USART1, USART_INT_RBNE);
    nvic_irq_enable(USART1_IRQn, 2U, 0U);

    rx_head = 0U;
    rx_tail = 0U;

    /* Derive the divider from the real APB1 frequency, not a hard-coded clock. */
    apb1_hz = rcu_clock_freq_get(CK_APB1);

    stats.rx_bytes = 0U;
    stats.tx_bytes = 0U;
    stats.rx_overflow = 0U;
    stats.uart_errors = 0U;
    stats.rx_orerr = 0U;
    stats.rx_ferr = 0U;
    stats.rx_nerr = 0U;
    stats.rx_perr = 0U;
    stats.tx_timeouts = 0U;
    stats.rx_dropped = 0U;
    stats.apb1_hz = apb1_hz;
    stats.baudrate = baudrate;
}

/* ------------------------------------------------------------------ */
/* Interrupt service                                                   */
/* ------------------------------------------------------------------ */

/**
 * @brief USART1接收中断处理，累计错误并把字节入队；满队列丢字节，不打印/延时。
 *
 * 调用：仅USART1_IRQHandler转入。
 * 过程：读状态计ORERR/FERR/NERR/PERR，有RBNE时读数据并推进单生产者head。
 * 满队列：丢弃本字节并增加overflow；不阻塞等待消费者。
 */
void rs485_isr(void)
{
    uint32_t stat = USART_STAT0(USART1);

    if(0U != (stat & USART_STAT0_ORERR)) {
        stats.rx_orerr++;
        stats.uart_errors++;
    }
    if(0U != (stat & USART_STAT0_FERR)) {
        stats.rx_ferr++;
        stats.uart_errors++;
    }
    if(0U != (stat & USART_STAT0_NERR)) {
        stats.rx_nerr++;
        stats.uart_errors++;
    }
    if(0U != (stat & USART_STAT0_PERR)) {
        stats.rx_perr++;
        stats.uart_errors++;
    }

    /*
     * Reading STAT0 (above) and then DATA clears RBNE together with the error
     * flags, as required by the reference manual. Do not invent a generic
     * "clear everything" write.
     */
    if(0U != (stat & USART_STAT0_RBNE)) {
        uint8_t data = (uint8_t)USART_DATA(USART1);
        uint16_t next = (uint16_t)((rx_head + 1U) % BOARD_RS485_RX_BUF_SIZE);

        if(next == rx_tail) {
            /* Buffer full: drop the new byte and count it. Never overwrite. */
            stats.rx_overflow++;
        } else {
            rx_buf[rx_head] = data;
            rx_head = next;
            stats.rx_bytes++;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Receive API                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief 返回环形队列当前待取字节数；ISR仍可能继续入队。
 *
 * 返回：(head+容量-tail)%容量，最多容量减1。
 * 并发：ISR可继续入队，结果为瞬时值；不清队列或统计。
 */
size_t rs485_rx_available(void)
{
    return (size_t)((rx_head + BOARD_RS485_RX_BUF_SIZE - rx_tail) % BOARD_RS485_RX_BUF_SIZE);
}

/**
 * @brief 最多取capacity字节，返回实际数量；NULL缓冲返回0，不阻塞等待。
 *
 * 输入：data至少capacity字节空间；NULL直接返回0。
 * 过程：从tail拷贝到capacity或当前空为止，返回已拷贝数量。
 * 并发：主循环应为唯一消费者，不要与其他读/flush调用并发。
 */
size_t rs485_read(uint8_t *data, size_t capacity)
{
    size_t count = 0U;

    if(NULL == data) {
        return 0U;
    }

    while((count < capacity) && (rx_tail != rx_head)) {
        data[count] = rx_buf[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % BOARD_RS485_RX_BUF_SIZE);
        count++;
    }

    return count;
}

/**
 * @brief 丢弃当前接收队列；不清硬件FIFO或历史统计。
 *
 * 过程：tail=head，放弃当前排队字节。
 * 限制：新到达字节仍可继续入队，不清历史文本或统计，也不重置USART硬件。
 */
void rs485_rx_flush(void)
{
    rx_tail = rx_head;
}

/* ------------------------------------------------------------------ */
/* Transmit API                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief 阻塞发送并等待TC后释放DE；返回0成功、-1空指针、-2超时，本实现没有-3返回路径。
 *
 * 输入：data非NULL，length字节；timeout_ms=0使用默认500ms。
 * 顺序：DE发送→建立延时→逐字节等待TBE→最后等待TC→保持延时→释放DE。
 * 超时：等待循环累计近似us，包含等待而非精确墙钟时间；DE退出仍会执行，统计加1。
 * 返回：0完成，-1空指针，-2超时；不是现役USART0非阻塞发送接口。
 */
int rs485_send(const uint8_t *data, size_t length, uint32_t timeout_ms)
{
    size_t index;
    uint32_t elapsed_us = 0U;
    uint32_t timeout_us;
    int result = 0;

    if(NULL == data) {
        return -1;
    }

    /* A zero-length frame succeeds immediately and must not touch DE. */
    if(0U == length) {
        return 0;
    }

    if(0U == timeout_ms) {
        timeout_ms = BOARD_RS485_TX_TIMEOUT_MS;
    }

    /*
     * Total timeout covers the whole frame at line rate plus DE switching
     * margin. 8N1 needs about length * 10 bits / baud seconds.
     */
    timeout_us = timeout_ms * 1000U;

    rs485_de_assert();
    rs485_delay_us(BOARD_RS485_DE_ASSERT_US);

    for(index = 0U; index < length; index++) {
        /* Wait for the transmit data register to be empty. */
        while(RESET == usart_flag_get(USART1, USART_FLAG_TBE)) {
            rs485_delay_us(1U);
            elapsed_us += 1U;
            if(elapsed_us >= timeout_us) {
                result = -2;
                break;
            }
        }

        if(0 != result) {
            break;
        }

        usart_data_transmit(USART1, data[index]);
        stats.tx_bytes++;
    }

    if(0 == result) {
        /* Wait for the real transmission-complete flag, not TBE. */
        while(RESET == usart_flag_get(USART1, USART_FLAG_TC)) {
            rs485_delay_us(1U);
            elapsed_us += 1U;
            if(elapsed_us >= timeout_us) {
                result = -2;
                break;
            }
        }
    }

    if(0 == result) {
        rs485_delay_us(BOARD_RS485_DE_HOLD_US);
    }

    /*
     * Always restore the receiver, on every path. A timeout must never leave
     * the driver enabled and the bus occupied.
     */
    rs485_de_release();

    if(-2 == result) {
        stats.tx_timeouts++;
    }

    return result;
}

/* ------------------------------------------------------------------ */
/* Receive-only monitoring                                             */
/* ------------------------------------------------------------------ */

/**
 * @brief 短暂关中断清空调试文本累积，不清接收环形队列或历史总计数。
 *
 * 过程：保存PRIMASK、关中断、清调试数组和长度，再恢复原中断状态。
 * 限制：不清g_rs485_rx_total、队列及ready_stopped，不等同于重启握手。
 */
void rs485_line_clear(void)
{
    uint32_t index;

    /* Mask interrupts so the ISR cannot append while the buffer is cleared. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    for(index = 0U; index < BOARD_RS485_LINE_BUF_SIZE; index++) {
        g_rs485_line[index] = '\0';
    }
    g_rs485_line_len = 0U;

    if(0U == primask) {
        __enable_irq();
    }
}

/**
 * @brief 主循环把接收队列追加到调试字符串，满后计丢弃；会消费数据，不能与另一协议解析器并用。
 *
 * 过程：每次取最多32字节，追加到NUL结尾调试字符串。
 * 容量：字符串满后继续消费队列并增加dropped；总接收计数仍增加。
 * 限制：监视功能会消耗协议数据，只能用于独占接收测试。
 */
void rs485_monitor_update(void)
{
    uint8_t chunk[32];
    size_t got;

    /*
     * Drain the ring buffer into the global capture buffer. Bytes beyond the
     * buffer capacity are counted in g_rs485_rx_dropped so a full buffer is
     * distinguishable from "nothing arrived".
     */
    while((got = rs485_read(chunk, sizeof(chunk))) > 0U) {
        size_t index;

        for(index = 0U; index < got; index++) {
            g_rs485_rx_total++;

            if(g_rs485_line_len < (BOARD_RS485_LINE_BUF_SIZE - 1U)) {
                g_rs485_line[g_rs485_line_len] = (char)chunk[index];
                g_rs485_line_len++;
                g_rs485_line[g_rs485_line_len] = '\0';
            } else {
                g_rs485_rx_dropped++;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Ready handshake (RS485_TEST_MODE_RX)                                */
/* ------------------------------------------------------------------ */

volatile uint32_t g_rs485_ready_sent = 0U;
volatile uint32_t g_rs485_ready_stopped = 0U;

static uint32_t ready_last_ms = 0U;

/**
 * @brief 首次收到任何数据前周期发送ready；停止标志锁存，不会自动重新握手。
 *
 * 过程：已经stopped直接返回；收到过任意字节则锁存停止；否则到周期发送ready。
 * 限制：send失败也会增加ready_sent调用计数；不能把该值当成功送达帧数。
 */
void rs485_ready_handshake_step(void)
{
    if(0U != g_rs485_ready_stopped) {
        return;   /* the host already answered: stay quiet */
    }

    /* Any received byte ends the handshake. */
    if(0U != g_rs485_rx_total) {
        g_rs485_ready_stopped = 1U;
        return;
    }

    if((uint32_t)(sys_tick_ms - ready_last_ms) >= BOARD_RS485_READY_PERIOD_MS) {
        ready_last_ms = sys_tick_ms;

        /*
         * rs485_send() asserts DE, sends the frame, waits for TC, then releases
         * DE, so PA1 is low again by the time this returns.
         */
        (void)rs485_send((const uint8_t *)"ready\r\n", 7U, BOARD_RS485_TX_TIMEOUT_MS);
        g_rs485_ready_sent++;
    }
}

/* ------------------------------------------------------------------ */
/* Statistics                                                          */
/* ------------------------------------------------------------------ */

/**
 * @brief 关中断复制一致统计快照，out为NULL时忽略，恢复原中断状态。
 *
 * 输入：out为结构体输出指针，NULL忽略。
 * 过程：关中断复制所有统计后恢复先前状态；避免ISR更新造成多字段不一致。
 * 限制：拷贝不清统计，发送字节计数也不证明远端收到。
 */
void rs485_get_stats(rs485_stats_t *out)
{
    uint32_t primask;

    if(NULL == out) {
        return;
    }

    /*
     * The counters are written by the ISR, so mask interrupts while copying to
     * avoid a torn snapshot.
     */
    primask = __get_PRIMASK();
    __disable_irq();

    out->rx_bytes = stats.rx_bytes;
    out->tx_bytes = stats.tx_bytes;
    out->rx_overflow = stats.rx_overflow;
    out->uart_errors = stats.uart_errors;
    out->rx_orerr = stats.rx_orerr;
    out->rx_ferr = stats.rx_ferr;
    out->rx_nerr = stats.rx_nerr;
    out->rx_perr = stats.rx_perr;
    out->tx_timeouts = stats.tx_timeouts;
    out->rx_dropped = stats.rx_dropped;
    out->apb1_hz = stats.apb1_hz;
    out->baudrate = stats.baudrate;

    if(0U == primask) {
        __enable_irq();
    }
}
