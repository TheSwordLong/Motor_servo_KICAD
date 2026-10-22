/*
 * usart0.h — 现役 CH340/Modbus 串口：PA9 TX、PA10 RX，115200 8N1。
 * ISR 将字节、微秒时间戳和错误标志写入环形队列；主循环组帧，TBE 中断发送二进制应答。
 * 文本累积/回显/OK 检查为保留调试接口，不能与 RTU 同时消费接收队列。
 */
#ifndef BSP_USART0_H
#define BSP_USART0_H

#include <stdint.h>

/*
 * USART0 serial port.
 *
 *   PA9  = USART0_TX   (LQFP100 pin 68)
 *   PA10 = USART0_RX   (LQFP100 pin 69)
 *
 * Both pins use the default (non-remapped) USART0 mapping, so no AFIO remap is
 * needed. Neither pin is used by the four-motor assignment.
 *
 * USART0 is on APB2 (120 MHz); the divider is derived by usart_baudrate_set().
 *
 * Reception:
 *   RBNE interrupt -> ring buffer, so no received byte is lost between polls.
 *   usart0_receive_update() drains the ring buffer and accumulates bytes into
 *   the global g_usart0_line for debugger inspection.
 */

#define USART0_PORT_BAUDRATE 115200U

/* ---------- transmit ---------- */

/* Configure PA9/PA10 and enable USART0 at USART0_PORT_BAUDRATE. */
/**
 * @brief 启动USART0 115200 8N1及接收中断；应在系统初始化阶段调用。
 *
 * 过程：GPIOA/USART0时钟、PA9复用推挽/PA10上拉、115200 8N1，然后使能RX和中断优先级2。
 * 限制：队列依赖启动时全局清零；运行中再次调用不等同于完整重置软件状态。
 */
void usart0_init(void);

/* Blocking send of one byte (waits for TBE). */
/**
 * @brief 等待TBE后写一个字节，无超时；不等最后停止位，不应与异步帧发送混用。
 *
 * 输入：一个待发字节。
 * 过程：忙等TBE后写DATA，函数返回时最后停止位可能尚未发出。
 * 限制：无超时，串口未初始化可阻塞；不能和TBE中断发送同时使用。
 */
void usart0_write_byte(uint8_t data);

/* Blocking send of a NUL-terminated string. Ignores NULL. */
/**
 * @brief 阻塞发送NUL结尾文本，NULL忽略；仅用于保留调试接口。
 *
 * 输入：NUL结尾text，NULL忽略。
 * 过程：逐字节调用阻塞write_byte；不发送结尾NUL。
 * 限制：只用于历史调试，现役Modbus不能混入这些文本。
 */
void usart0_write(const char *text);

/* ---------- receive ---------- */

/* Number of received bytes still waiting in the ring buffer. */
/**
 * @brief 返回当前接收字节数，队列由ISR生产、主循环消费。
 *
 * 返回：512槽环形队列中的字节数量，读取后新字节仍可能到达。
 * 用途：配合先采样时间的RTU idle逻辑，而不是忙等收满帧。
 */
uint32_t usart0_rx_available(void);

/* Pop one byte from the ring buffer: 1 = got a byte, 0 = buffer empty. */
/**
 * @brief 保留的单字节接收接口：成功1，空或NULL输出0；不返回RTU时间戳。
 *
 * 输入：data非NULL才允许取字节；队列空或NULL返回0。
 * 限制：取出会同步消费该槽时间戳/错误信息但不返回，现役RTU应使用rx_event。
 */
uint8_t usart0_rx_pop(uint8_t *data);

/*
 * Drain the ring buffer into the global monitor variables.
 * Call this from the main loop.
 */
/**
 * @brief 把接收队列消耗到调试字符串，超出字符串容量的字节不保留。
 *
 * 过程：逐字节rx_pop，最多留63字节并维持结尾NUL。
 * 限制：调试数组满后其余取出的字节不保存；不能作为可靠二进制协议接收器。
 */
void usart0_receive_update(void);

/*
 * Echo any newly received bytes back out of the transmitter.
 * Reads from g_usart0_line, so the monitor buffer stays intact.
 * Call this from the main loop after usart0_receive_update().
 */
/**
 * @brief 回显调试字符串新增部分；不会自动运行，也不能插入二进制Modbus应答流。
 *
 * 过程：用静态echoed游标只输出新增文本，缓冲清空或缩短时将游标重置。
 * 限制：调用可阻塞；不会清调试文本，也不是UART硬件本地回显。
 */
void usart0_echo_new_bytes(void);

/* Clear g_usart0_line and g_usart0_line_len. */
/**
 * @brief 清除历史调试字符串及长度，不清二进制接收队列。
 *
 * 过程：清64字节历史调试数组和长度。
 * 限制：不清环形队列、总计数或硬件状态；回显游标在echo下一次调用时自行校正。
 */
void usart0_line_clear(void);

/*
 * Return 1 when g_usart0_line contains the literal "OK" (case-insensitive).
 * The caller is expected to send its reply and then call usart0_line_clear(),
 * which yields exactly one reply per received "OK".
 */
/**
 * @brief 调试字符串含不区分大小写OK时返回1，不消费或清除字符串。
 *
 * 过程：遍历相邻字符，局部把小写转大写后寻找OK。
 * 返回：存在就返回1，不改变内容；调用者不清行则下次仍返回1。
 */
uint8_t usart0_ok_ready(void);

/* ---------- global monitor variables ---------- */

/*
 * Watch these in the debugger:
 *   print g_usart0_line        <- accumulated received text
 *   print g_usart0_rx_count    <- total bytes received
 *   print g_usart0_status      <- last USART_STAT0 read
 */
extern volatile uint8_t  g_usart0_rx_byte;    /* most recent received byte */
extern volatile uint32_t g_usart0_rx_count;   /* total received byte count  */
extern volatile uint32_t g_usart0_rx_overrun; /* ring buffer overrun count  */
extern volatile uint32_t g_usart0_status;     /* last USART0_STAT0 value    */
extern volatile char     g_usart0_line[];     /* accumulated received bytes */
extern volatile uint32_t g_usart0_line_len;   /* valid bytes in g_usart0_line */

/* Binary transport APIs. send copies data and transmits using TBE IRQ. */
/**
 * @brief 复制最多256字节到发送缓冲；忙或过长返回0，排入成功返回1，不表示线上已发送完成。
 *
 * 输入：p可读n字节，n<=256；调用者保证指针有效。
 * 互锁：关中断检查缓冲空和TC已完成，再复制/发布发送长度。
 * 返回：1为成功接管缓冲，0为忙/过长；n=0不启动TBE，发送完成需另查tx_busy。
 */
int usart0_send(const uint8_t *,uint16_t);
/**
 * @brief 发送缓冲未耗尽或硬件TC未置位都视为忙。
 *
 * 返回：tx_pos<tx_length或硬件TC未完成时为真。
 * 作用：防止覆盖尚在发送的数据；队列耗尽不代表线路已空闲。
 */
int usart0_tx_busy(void);
/**
 * @brief 取一个字节及其us时间戳/错误标志；成功1、空0，三个输出指针须有效。
 *
 * 输入：data/stamp/error都须可写，分别接收字节、us时间和UART错误位汇总。
 * 过程：队列空返回0；否则按相同tail读取三个字段后推进tail，返回1。
 * 限制：仅主循环单消费者使用，不能和rx_pop或文本监视同时取同一队列。
 */
int usart0_rx_event(uint8_t *,uint32_t *,uint8_t *);
/**
 * @brief 将tail移到head丢弃当前积压字节；溢出恢复调用者在临界区内执行。
 *
 * 过程：tail直接追上head丢积压。
 * 限制：不重置硬件或时间戳；溢出处理还要把RTU标坏，防止后半帧误执行。
 */
void usart0_discard_rx(void);
#endif /* BSP_USART0_H */
