/*
 * usart0.c — 现役 CH340/Modbus 串口：PA9 TX、PA10 RX，115200 8N1。
 * ISR 将字节、微秒时间戳和错误标志写入环形队列；主循环组帧，TBE 中断发送二进制应答。
 * 文本累积/回显/OK 检查为保留调试接口，不能与 RTU 同时消费接收队列。
 */
#include "usart0.h"

#include <stddef.h>

#include "gd32f30x.h"
#include "robot_port.h"
#include <string.h>

/* PA9 = USART0_TX, PA10 = USART0_RX (default mapping, no remap). */
#define USART0_GPIO_PORT    GPIOA
#define USART0_TX_PIN       GPIO_PIN_9
#define USART0_RX_PIN       GPIO_PIN_10
#define USART0_GPIO_CLOCK   RCU_GPIOA

/* Ring buffer filled by the RBNE interrupt, drained by the main loop. */
#define USART0_RX_BUF_SIZE  512U

/* Accumulated received text kept for debugger inspection. */
#define USART0_LINE_SIZE    64U

/* 环形队列保留一个空槽区分满/空；字节、时间戳、错误信息使用同一索引。 */
static volatile uint8_t usart0_rx_buf[USART0_RX_BUF_SIZE];
static volatile uint32_t rx_stamp[USART0_RX_BUF_SIZE];
static volatile uint8_t rx_error[USART0_RX_BUF_SIZE];
static uint8_t tx_buffer[256];
static volatile uint16_t tx_pos,tx_length;
static volatile uint16_t usart0_rx_head = 0U;
static volatile uint16_t usart0_rx_tail = 0U;

/* Global monitor variables (see usart0.h). */
volatile uint8_t  g_usart0_rx_byte = 0U;
volatile uint32_t g_usart0_rx_count = 0U;
volatile uint32_t g_usart0_rx_overrun = 0U;
volatile uint32_t g_usart0_status = 0U;
volatile char     g_usart0_line[USART0_LINE_SIZE];
volatile uint32_t g_usart0_line_len = 0U;

/**
 * @brief 启动USART0 115200 8N1及接收中断；应在系统初始化阶段调用。
 *
 * 过程：GPIOA/USART0时钟、PA9复用推挽/PA10上拉、115200 8N1，然后使能RX和中断优先级2。
 * 限制：队列依赖启动时全局清零；运行中再次调用不等同于完整重置软件状态。
 */
void usart0_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART0);

    /* The USART0 default pins are PA9 (TX) and PA10 (RX); no remap needed. */
    gpio_init(USART0_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, USART0_TX_PIN);
    gpio_init(USART0_GPIO_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, USART0_RX_PIN);

    usart_deinit(USART0);
    usart_baudrate_set(USART0, USART0_PORT_BAUDRATE);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);

    usart_interrupt_flag_clear(USART0, USART_INT_FLAG_RBNE);
    usart_interrupt_enable(USART0, USART_INT_RBNE);
    nvic_irq_enable(USART0_IRQn, 2U, 0U);
}

/**
 * @brief 等待TBE后写一个字节，无超时；不等最后停止位，不应与异步帧发送混用。
 *
 * 输入：一个待发字节。
 * 过程：忙等TBE后写DATA，函数返回时最后停止位可能尚未发出。
 * 限制：无超时，串口未初始化可阻塞；不能和TBE中断发送同时使用。
 */
void usart0_write_byte(uint8_t data)
{
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE)) {
    }
    usart_data_transmit(USART0, data);
}

/**
 * @brief 阻塞发送NUL结尾文本，NULL忽略；仅用于保留调试接口。
 *
 * 输入：NUL结尾text，NULL忽略。
 * 过程：逐字节调用阻塞write_byte；不发送结尾NUL。
 * 限制：只用于历史调试，现役Modbus不能混入这些文本。
 */
void usart0_write(const char *text)
{
    if(NULL == text) {
        return;
    }

    while('\0' != *text) {
        usart0_write_byte((uint8_t)*text);
        text++;
    }
}

/* Push one byte into the ring buffer. Drops the byte and counts an overrun if full. */
/**
 * @brief ISR单生产者入队，同时记录us时间戳和错误标志；满队列计溢出并丢弃。
 *
 * 调用：ISR，data已从UART读出；g_usart0_status为相应状态。
 * 过程：先计算next，满则只计overrun；有空间则写字节、robot_micros时间和错误标志，最后发布head。
 * 容量：512槽最多保存511字节，单生产者/单消费者模型。
 */
static void usart0_rx_push(uint8_t data)
{
    uint16_t next = (uint16_t)((usart0_rx_head + 1U) % USART0_RX_BUF_SIZE);

    if(next == usart0_rx_tail) {
        g_usart0_rx_overrun++;
        return;
    }

    usart0_rx_buf[usart0_rx_head] = data;
    rx_stamp[usart0_rx_head]=robot_micros();
    rx_error[usart0_rx_head]=(g_usart0_status & 15U)!=0;
    usart0_rx_head = next;
    g_usart0_rx_byte = data;
    g_usart0_rx_count++;
}

/**
 * @brief 处理接收和TBE发送；接收保留错误标志，发送最后一字节后关闭TBE中断。
 *
 * 接收：RBNE或错误位存在时读取DATA并入队，记录状态低4位错误。
 * 发送：若TBE且缓冲未发完则送下一字节；最后一字节写出关闭TBE中断。
 * 限制：这里只搬运字节，不解码Modbus，不等待整帧TC。
 */
void USART0_IRQHandler(void)
{
    uint32_t flags=USART_STAT0(USART0);
    if(flags & (USART_STAT0_RBNE|15U)) {
        g_usart0_status=flags;
        usart0_rx_push((uint8_t)usart_data_receive(USART0));
    }
    if(tx_pos<tx_length && (flags&USART_STAT0_TBE)) {
        usart_data_transmit(USART0,tx_buffer[tx_pos++]);
        if(tx_pos==tx_length) usart_interrupt_disable(USART0,USART_INT_TBE);
    }
}
/**
 * @brief 复制最多256字节到发送缓冲；忙或过长返回0，排入成功返回1，不表示线上已发送完成。
 *
 * 输入：p可读n字节，n<=256；调用者保证指针有效。
 * 互锁：关中断检查缓冲空和TC已完成，再复制/发布发送长度。
 * 返回：1为成功接管缓冲，0为忙/过长；n=0不启动TBE，发送完成需另查tx_busy。
 */
int usart0_send(const uint8_t *p,uint16_t n)
{
    uint32_t key=__get_PRIMASK(); __disable_irq();
    if(n>sizeof(tx_buffer) || tx_pos<tx_length || RESET==usart_flag_get(USART0,USART_FLAG_TC)) {
        __set_PRIMASK(key); return 0;
    }
    /* Publish buffer and indices atomically even if RX interrupts arrive
     * while the previous TX length is still nonzero. */
    /* 先在临界区复制并发布长度，再启用TBE；调用者须保证p在n字节范围内可读。 */
    memcpy(tx_buffer,p,n); tx_pos=0; tx_length=n;
    if(n) usart_interrupt_enable(USART0,USART_INT_TBE);
    __set_PRIMASK(key);
    return 1;
}
/**
 * @brief 发送缓冲未耗尽或硬件TC未置位都视为忙。
 *
 * 返回：tx_pos<tx_length或硬件TC未完成时为真。
 * 作用：防止覆盖尚在发送的数据；队列耗尽不代表线路已空闲。
 */
int usart0_tx_busy(void)
{ return tx_pos<tx_length || RESET==usart_flag_get(USART0,USART_FLAG_TC); }
/**
 * @brief 取一个字节及其us时间戳/错误标志；成功1、空0，三个输出指针须有效。
 *
 * 输入：data/stamp/error都须可写，分别接收字节、us时间和UART错误位汇总。
 * 过程：队列空返回0；否则按相同tail读取三个字段后推进tail，返回1。
 * 限制：仅主循环单消费者使用，不能和rx_pop或文本监视同时取同一队列。
 */
int usart0_rx_event(uint8_t *data,uint32_t *stamp,uint8_t *error)
{
    uint16_t tail=usart0_rx_tail;
    if(tail==usart0_rx_head) return 0;
    *data=usart0_rx_buf[tail]; *stamp=rx_stamp[tail]; *error=rx_error[tail];
    usart0_rx_tail=(uint16_t)((tail+1)%USART0_RX_BUF_SIZE); return 1;
}
/**
 * @brief 将tail移到head丢弃当前积压字节；溢出恢复调用者在临界区内执行。
 *
 * 过程：tail直接追上head丢积压。
 * 限制：不重置硬件或时间戳；溢出处理还要把RTU标坏，防止后半帧误执行。
 */
void usart0_discard_rx(void) { usart0_rx_tail=usart0_rx_head; }

/**
 * @brief 返回当前接收字节数，队列由ISR生产、主循环消费。
 *
 * 返回：512槽环形队列中的字节数量，读取后新字节仍可能到达。
 * 用途：配合先采样时间的RTU idle逻辑，而不是忙等收满帧。
 */
uint32_t usart0_rx_available(void)
{
    return (uint32_t)((usart0_rx_head + USART0_RX_BUF_SIZE - usart0_rx_tail) % USART0_RX_BUF_SIZE);
}

/**
 * @brief 保留的单字节接收接口：成功1，空或NULL输出0；不返回RTU时间戳。
 *
 * 输入：data非NULL才允许取字节；队列空或NULL返回0。
 * 限制：取出会同步消费该槽时间戳/错误信息但不返回，现役RTU应使用rx_event。
 */
uint8_t usart0_rx_pop(uint8_t *data)
{
    if((NULL == data) || (usart0_rx_tail == usart0_rx_head)) {
        return 0U;
    }

    *data = usart0_rx_buf[usart0_rx_tail];
    usart0_rx_tail = (uint16_t)((usart0_rx_tail + 1U) % USART0_RX_BUF_SIZE);
    return 1U;
}

/**
 * @brief 清除历史调试字符串及长度，不清二进制接收队列。
 *
 * 过程：清64字节历史调试数组和长度。
 * 限制：不清环形队列、总计数或硬件状态；回显游标在echo下一次调用时自行校正。
 */
void usart0_line_clear(void)
{
    uint32_t index;

    for(index = 0U; index < USART0_LINE_SIZE; index++) {
        g_usart0_line[index] = '\0';
    }

    g_usart0_line_len = 0U;
}

/**
 * @brief 把接收队列消耗到调试字符串，超出字符串容量的字节不保留。
 *
 * 过程：逐字节rx_pop，最多留63字节并维持结尾NUL。
 * 限制：调试数组满后其余取出的字节不保存；不能作为可靠二进制协议接收器。
 */
void usart0_receive_update(void)
{
    uint8_t data;

    while(0U != usart0_rx_pop(&data)) {
        if(g_usart0_line_len < (USART0_LINE_SIZE - 1U)) {
            g_usart0_line[g_usart0_line_len] = (char)data;
            g_usart0_line_len++;
            g_usart0_line[g_usart0_line_len] = '\0';
        }
    }
}

/**
 * @brief 回显调试字符串新增部分；不会自动运行，也不能插入二进制Modbus应答流。
 *
 * 过程：用静态echoed游标只输出新增文本，缓冲清空或缩短时将游标重置。
 * 限制：调用可阻塞；不会清调试文本，也不是UART硬件本地回显。
 */
void usart0_echo_new_bytes(void)
{
    static uint32_t echoed = 0U;

    /*
     * Echo straight out of the accumulated line buffer, so the buffer itself
     * stays intact for debugger inspection. "echoed" tracks how much has
     * already been sent.
     */
    while(echoed < g_usart0_line_len) {
        usart0_write_byte((uint8_t)g_usart0_line[echoed]);
        echoed++;
    }

    /* A cleared line buffer restarts the echo from zero. */
    if((0U == g_usart0_line_len) || (echoed > g_usart0_line_len)) {
        echoed = 0U;
    }
}

/**
 * @brief 调试字符串含不区分大小写OK时返回1，不消费或清除字符串。
 *
 * 过程：遍历相邻字符，局部把小写转大写后寻找OK。
 * 返回：存在就返回1，不改变内容；调用者不清行则下次仍返回1。
 */
uint8_t usart0_ok_ready(void)
{
    uint32_t index;

    /*
     * Look for the literal "OK" anywhere in the accumulated line.
     * ASCII case folding keeps "ok" and "Ok" working too.
     */
    for(index = 0U; (index + 1U) < g_usart0_line_len; index++) {
        char c0 = g_usart0_line[index];
        char c1 = g_usart0_line[index + 1U];

        if((c0 >= 'a') && (c0 <= 'z')) {
            c0 = (char)(c0 - 'a' + 'A');
        }
        if((c1 >= 'a') && (c1 <= 'z')) {
            c1 = (char)(c1 - 'a' + 'A');
        }

        if(('O' == c0) && ('K' == c1)) {
            return 1U;
        }
    }

    return 0U;
}
