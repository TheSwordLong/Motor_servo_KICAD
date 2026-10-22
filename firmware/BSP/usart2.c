/*
 * usart2.c — 保留的 USART2 轮询串口：PB10 TX、PB11 RX，115200 8N1。
 * PB10 与现役 M1 FG 冲突，当前 robot_port 不初始化本模块，不可直接与四路 FG 同时启用。
 */
#include "usart2.h"

#include <stddef.h>

#include "gd32f30x.h"

/**
 * @brief 初始化PB10/PB11为USART2 115200 8N1；与M1 FG冲突，当前机器人不调用。
 *
 * 过程：禁用USART2部分/全重映射，配置PB10 TX/PB11 RX，USART2为115200 8N1。
 * 限制：轮询接口不配中断队列；会占用M1 FG的PB10，当前机器人不调用。
 */
void usart2_init(void)
{
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_USART2);

    /* USART2 default pins: PB10 = TX, PB11 = RX. Do not enable remapping. */
    gpio_pin_remap_config(GPIO_USART2_PARTIAL_REMAP, DISABLE);
    gpio_pin_remap_config(GPIO_USART2_FULL_REMAP, DISABLE);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_11);

    usart_deinit(USART2);
    usart_baudrate_set(USART2, 115200U);
    usart_word_length_set(USART2, USART_WL_8BIT);
    usart_stop_bit_set(USART2, USART_STB_1BIT);
    usart_parity_config(USART2, USART_PM_NONE);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    usart_enable(USART2);
}

/**
 * @brief 轮询等待TBE后写一字节，无超时，不等待最后停止位。
 *
 * 输入：待发字节；忙等TBE后写DATA。
 * 限制：无超时、非整帧发送完成确认，依赖先初始化USART2。
 */
void usart2_write_byte(uint8_t data)
{
    while(RESET == usart_flag_get(USART2, USART_FLAG_TBE)) {
    }
    usart_data_transmit(USART2, data);
}

/**
 * @brief 阻塞发送NUL结尾字符串，NULL忽略。
 *
 * 输入：NUL结尾字符串，NULL不操作。
 * 过程：逐字符阻塞发送，不附加换行或NUL。
 */
void usart2_write(const char *text)
{
    if(NULL == text) {
        return;
    }

    while('\0' != *text) {
        usart2_write_byte((uint8_t)*text);
        text++;
    }
}

/**
 * @brief 非阻塞读取单字节，收到返回1，空或NULL返回0。
 *
 * 输入：可写data指针。
 * 返回：NULL或RBNE未置位返回0；否则读DATA并返回1，不等待新数据。
 */
uint8_t usart2_read_byte(uint8_t *data)
{
    if((NULL == data) || (RESET == usart_flag_get(USART2, USART_FLAG_RBNE))) {
        return 0U;
    }

    *data = (uint8_t)usart_data_receive(USART2);
    return 1U;
}
