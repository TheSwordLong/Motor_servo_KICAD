/*
 * rs485.h — 保留的 USART1 半双工 RS485 驱动：PA2 TX、PA3 RX、PA1 DE。
 * 当前机器人 Modbus 使用 USART0/CH340，不调用本模块初始化。中断生产字节，主循环消费。
 * 发送为阻塞式，延时为软件近似值；不要把历史测试接口当作当前机器人通信入口。
 */
#ifndef BSP_RS485_H
#define BSP_RS485_H

#include <stddef.h>
#include <stdint.h>

/*
 * RS-485 half-duplex driver on USART1.
 *
 *   PA2 = RS485_TX = USART1_TX (default mapping, no remap)
 *   PA3 = RS485_RX = USART1_RX
 *   PA1 = RS485_DE = plain GPIO, high = driver enabled
 *
 * See BSP/board_config.h for the verified pinout and hardware notes.
 *
 * Reception uses the USART1 RBNE interrupt and a 512-byte ring buffer.
 * The ISR only stores bytes and counts errors; it never prints, delays or
 * transmits. USART1_IRQHandler lives in Core/Src/gd32f30x_it.c.
 *
 * Concurrency model: the ISR is the single producer, the main loop is the
 * single consumer. The ring buffer needs no critical section because head is
 * written only by the ISR and tail only by the consumer. The statistics
 * snapshot does need one: rs485_get_stats() briefly masks interrupts.
 */

/* ------------------------------------------------------------------ */
/* Statistics                                                          */
/* ------------------------------------------------------------------ */

typedef struct {
    uint32_t rx_bytes;       /* bytes accepted into the ring buffer      */
    uint32_t tx_bytes;       /* bytes handed to the transmit register    */
    uint32_t rx_overflow;    /* bytes dropped because the buffer was full */
    uint32_t uart_errors;    /* ORERR + FERR + NERR + PERR occurrences   */
    uint32_t rx_orerr;       /* overrun error count                      */
    uint32_t rx_ferr;        /* framing error count                      */
    uint32_t rx_nerr;        /* noise error count                        */
    uint32_t rx_perr;        /* parity error count                       */
    uint32_t tx_timeouts;    /* transmit attempts that hit the timeout   */
    uint32_t rx_dropped;     /* reserved: bytes dropped by policy        */
    uint32_t apb1_hz;        /* APB1 frequency actually used for the baud */
    uint32_t baudrate;       /* configured baud rate                     */
} rs485_stats_t;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/*
 * Configure PA1/PA2/PA3 and enable USART1 at the given baud rate.
 * baudrate == 0 selects BOARD_RS485_BAUDRATE.
 * DE is driven low (receive) before the pins become outputs, so enabling the
 * driver cannot glitch onto the bus.
 */
/**
 * @brief baudrate=0使用9600默认值；配置USART1和RX中断，清接收队列及统计。
 *
 * 输入：baudrate为波特率，0选择BOARD_RS485_BAUDRATE。
 * 过程：预置DE接收，再配TX/RX和USART1 8N1，禁用RTS/CTS，启用RBNE中断。
 * 完成：清环形队列和统计，记录实际APB1时钟；历史文本/握手状态不在这里全部复位。
 */
void rs485_init(uint32_t baudrate);

/*
 * Send one frame.
 *
 * Returns 0 on success, -1 on NULL data, -2 on timeout.
 * No -3 return path is implemented.
 *
 * With non-NULL data, length 0 succeeds immediately without touching DE.
 * The whole frame must fit in timeout_ms; the required line time at 8N1 is
 * about length * 10 / baud seconds. DE is always released before returning,
 * including on every error path, so the bus is never left driven.
 */
/**
 * @brief 阻塞发送并等待TC后释放DE；返回0成功、-1空指针、-2超时，本实现没有-3返回路径。
 *
 * 输入：data非NULL，length字节；timeout_ms=0使用默认500ms。
 * 顺序：DE发送→建立延时→逐字节等待TBE→最后等待TC→保持延时→释放DE。
 * 超时：等待循环累计近似us，包含等待而非精确墙钟时间；DE退出仍会执行，统计加1。
 * 返回：0完成，-1空指针，-2超时；不是现役USART0非阻塞发送接口。
 */
int rs485_send(const uint8_t *data, size_t length, uint32_t timeout_ms);

/*
 * Copy up to capacity bytes out of the ring buffer.
 * Returns the number of bytes copied (0 when nothing is available).
 */
/**
 * @brief 最多取capacity字节，返回实际数量；NULL缓冲返回0，不阻塞等待。
 *
 * 输入：data至少capacity字节空间；NULL直接返回0。
 * 过程：从tail拷贝到capacity或当前空为止，返回已拷贝数量。
 * 并发：主循环应为唯一消费者，不要与其他读/flush调用并发。
 */
size_t rs485_read(uint8_t *data, size_t capacity);

/* Copy a consistent snapshot of the statistics counters. */
/**
 * @brief 关中断复制一致统计快照，out为NULL时忽略，恢复原中断状态。
 *
 * 输入：out为结构体输出指针，NULL忽略。
 * 过程：关中断复制所有统计后恢复先前状态；避免ISR更新造成多字段不一致。
 * 限制：拷贝不清统计，发送字节计数也不证明远端收到。
 */
void rs485_get_stats(rs485_stats_t *out);

/* Number of bytes currently waiting in the ring buffer. */
/**
 * @brief 返回环形队列当前待取字节数；ISR仍可能继续入队。
 *
 * 返回：(head+容量-tail)%容量，最多容量减1。
 * 并发：ISR可继续入队，结果为瞬时值；不清队列或统计。
 */
size_t rs485_rx_available(void);

/* Discard everything in the ring buffer. */
/**
 * @brief 丢弃当前接收队列；不清硬件FIFO或历史统计。
 *
 * 过程：tail=head，放弃当前排队字节。
 * 限制：新到达字节仍可继续入队，不清历史文本或统计，也不重置USART硬件。
 */
void rs485_rx_flush(void);

/* Drive DE high (transmit). Used internally; exposed for bring-up tests. */
/**
 * @brief 按编译极性使能发送DE；不发送数据。
 *
 * 过程：按BOARD_RS485_DE_TX_LEVEL输出发送使能电平。
 * 限制：不等待隔离器建立时间，实际发送函数在本调用后另加延时。
 */
void rs485_de_assert(void);

/* Drive DE low (receive). */
/**
 * @brief 恢复接收DE；强制空闲高测试宏启用时例外，不代表已验证收发器电平。
 *
 * 过程：正常按DE反极性回接收；FORCE_DE_IDLE_HIGH=1时固定高。
 * 限制：测试强制高可能使接收关闭；它不是通信状态回读。
 */
void rs485_de_release(void);

/*
 * Called by USART1_IRQHandler in gd32f30x_it.c.
 * Stores one received byte and accounts for the USART error flags.
 * Must not be called from anywhere else.
 */
/**
 * @brief USART1接收中断处理，累计错误并把字节入队；满队列丢字节，不打印/延时。
 *
 * 调用：仅USART1_IRQHandler转入。
 * 过程：读状态计ORERR/FERR/NERR/PERR，有RBNE时读数据并推进单生产者head。
 * 满队列：丢弃本字节并增加overflow；不阻塞等待消费者。
 */
void rs485_isr(void);

/* ------------------------------------------------------------------ */
/* Receive-only monitoring (RS485_TEST_MODE_RX)                        */
/* ------------------------------------------------------------------ */

/*
 * Everything received is appended to these globals, which the debugger can
 * watch directly:
 *
 *   print g_rs485_line       <- accumulated received text
 *   print g_rs485_line_len   <- number of valid bytes
 *   print g_rs485_rx_total   <- total bytes received since reset
 *
 * g_rs485_line always holds BOARD_RS485_LINE_BUF_SIZE bytes and is kept
 * NUL-terminated, so it can also be printed as a C string. It stops appending
 * once full; call rs485_line_clear() to start a new capture. The counters keep
 * running regardless.
 */
extern volatile char     g_rs485_line[];
extern volatile uint32_t g_rs485_line_len;
extern volatile uint32_t g_rs485_rx_total;
extern volatile uint32_t g_rs485_rx_dropped;

/*
 * Append every byte currently in the ring buffer to g_rs485_line.
 * Call this from the main loop.
 */
/**
 * @brief 主循环把接收队列追加到调试字符串，满后计丢弃；会消费数据，不能与另一协议解析器并用。
 *
 * 过程：每次取最多32字节，追加到NUL结尾调试字符串。
 * 容量：字符串满后继续消费队列并增加dropped；总接收计数仍增加。
 * 限制：监视功能会消耗协议数据，只能用于独占接收测试。
 */
void rs485_monitor_update(void);

/* Reset g_rs485_line_len to 0 and blank the buffer. */
/**
 * @brief 短暂关中断清空调试文本累积，不清接收环形队列或历史总计数。
 *
 * 过程：保存PRIMASK、关中断、清调试数组和长度，再恢复原中断状态。
 * 限制：不清g_rs485_rx_total、队列及ready_stopped，不等同于重启握手。
 */
void rs485_line_clear(void);

/* True while the driver is in the receive-only configuration. */
/**
 * @brief 返回编译时 RS485_TEST_MODE_RX 选择，不代表硬件当前接收状态。
 *
 * 输入：无；使用预处理宏选择返回值。
 * 限制：只说明历史测试模式编译选择，不读取DE，也不禁止其他函数发送。
 */
uint8_t rs485_is_rx_only(void);

/*
 * Handshake helper for RS485_TEST_MODE_RX.
 *
 * Sends "ready" repeatedly at BOARD_RS485_READY_PERIOD_MS until the host sends
 * anything at all. Once at least one byte has arrived it stops transmitting and
 * never sends again, leaving the received data in g_rs485_line.
 *
 * Call this from the main loop after rs485_monitor_update().
 */
/**
 * @brief 首次收到任何数据前周期发送ready；停止标志锁存，不会自动重新握手。
 *
 * 过程：已经stopped直接返回；收到过任意字节则锁存停止；否则到周期发送ready。
 * 限制：send失败也会增加ready_sent调用计数；不能把该值当成功送达帧数。
 */
void rs485_ready_handshake_step(void);

/* Number of "ready" frames transmitted by the handshake helper. */
extern volatile uint32_t g_rs485_ready_sent;

/* 1 once host data has arrived and the handshake stopped transmitting. */
extern volatile uint32_t g_rs485_ready_stopped;

#endif /* BSP_RS485_H */
