/*
 * usart2.h — 保留的 USART2 轮询串口：PB10 TX、PB11 RX，115200 8N1。
 * PB10 与现役 M1 FG 冲突，当前 robot_port 不初始化本模块，不可直接与四路 FG 同时启用。
 */
#ifndef BSP_USART2_H
#define BSP_USART2_H

#include <stdint.h>

/**
 * @brief 初始化PB10/PB11为USART2 115200 8N1；与M1 FG冲突，当前机器人不调用。
 *
 * 过程：禁用USART2部分/全重映射，配置PB10 TX/PB11 RX，USART2为115200 8N1。
 * 限制：轮询接口不配中断队列；会占用M1 FG的PB10，当前机器人不调用。
 */
void usart2_init(void);
/**
 * @brief 轮询等待TBE后写一字节，无超时，不等待最后停止位。
 *
 * 输入：待发字节；忙等TBE后写DATA。
 * 限制：无超时、非整帧发送完成确认，依赖先初始化USART2。
 */
void usart2_write_byte(uint8_t data);
/**
 * @brief 阻塞发送NUL结尾字符串，NULL忽略。
 *
 * 输入：NUL结尾字符串，NULL不操作。
 * 过程：逐字符阻塞发送，不附加换行或NUL。
 */
void usart2_write(const char *text);

/* Non-blocking receive: returns 1 when one byte was read, otherwise 0. */
/**
 * @brief 非阻塞读取单字节，收到返回1，空或NULL返回0。
 *
 * 输入：可写data指针。
 * 返回：NULL或RBNE未置位返回0；否则读DATA并返回1，不等待新数据。
 */
uint8_t usart2_read_byte(uint8_t *data);

#endif /* BSP_USART2_H */
