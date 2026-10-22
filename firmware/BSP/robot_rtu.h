/*
 * robot_rtu.h — USART0 高波特率 RTU 组帧：字节时间戳单位 us，750 us 帧内超间隔标坏，
 * 1750 us 静默完成上一帧。UART 错误或缓冲溢出的帧丢弃，不交给命令执行器。
 */
#ifndef ROBOT_RTU_H
#define ROBOT_RTU_H
#include <stdint.h>
#include <stddef.h>
/* 组帧缓冲最大256字节；last_us为可回绕时间戳，active表示已起帧，bad表示整帧应丢弃。 */
typedef struct { uint8_t data[256]; size_t length; uint32_t last_us; int active,bad; } robot_rtu_t;
typedef void (*rtu_frame_fn)(const uint8_t *,size_t,void *);
/**
 * @brief 喂入一个字节及us时间戳/错误标志；使用无符号差处理时钟回绕，超长帧标坏。
 *
 * 输入：byte、到达时刻us、error非零代表UART错误；需按到达顺序逐字节调用。
 * 间隔：>=1750us先结束旧帧再以本字节起新帧；750～1750us之间把当前帧标坏。
 * 容量：超过256标坏且不越界写；坏帧仍等静默结束，不能把尾部当新命令。
 */
void robot_rtu_byte(robot_rtu_t *,uint8_t,uint32_t,int,rtu_frame_fn,void *);
/**
 * @brief 无新字节时调用，静默至少1750us则结束当前帧。
 *
 * 调用：主循环确认当前RX队列为空后，传入当前us。
 * 过程：active且距末字节至少1750us时finish，否则不操作；不阻塞等待。
 */
void robot_rtu_idle(robot_rtu_t *,uint32_t,rtu_frame_fn,void *);
#endif
