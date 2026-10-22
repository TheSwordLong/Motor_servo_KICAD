/*
 * robot_rtu.c — USART0 高波特率 RTU 组帧：字节时间戳单位 us，750 us 帧内超间隔标坏，
 * 1750 us 静默完成上一帧。UART 错误或缓冲溢出的帧丢弃，不交给命令执行器。
 */
#include "robot_rtu.h"
/**
 * @brief 交付非坏帧后清空组帧状态；无效帧静默丢弃，回调在调用线程同步执行。
 *
 * 输入：组帧状态r、同步回调fn和原样透传ctx；fn须有效。
 * 过程：只有active且非bad且length非0才回调；之后重置length/active/bad。
 * 限制：不做CRC检查，CRC由协议层处理；回调不能保留data指针假定内容长期不变。
 */
static void finish(robot_rtu_t *r,rtu_frame_fn fn,void *ctx)
{
    if(r->active && !r->bad && r->length) fn(r->data,r->length,ctx);
    r->length=0; r->active=0; r->bad=0;
}
/**
 * @brief 喂入一个字节及us时间戳/错误标志；使用无符号差处理时钟回绕，超长帧标坏。
 *
 * 输入：byte、到达时刻us、error非零代表UART错误；需按到达顺序逐字节调用。
 * 间隔：>=1750us先结束旧帧再以本字节起新帧；750～1750us之间把当前帧标坏。
 * 容量：超过256标坏且不越界写；坏帧仍等静默结束，不能把尾部当新命令。
 */
void robot_rtu_byte(robot_rtu_t *r,uint8_t byte,uint32_t us,int error,rtu_frame_fn fn,void *ctx)
{
    uint32_t gap=us-r->last_us;
    if(r->active && gap>=1750) finish(r,fn,ctx);
    else if(r->active && gap>750) r->bad=1;
    r->active=1; r->last_us=us;
    if(error) r->bad=1;
    if(r->length<sizeof(r->data)) r->data[r->length++]=byte;
    else r->bad=1;
}
/**
 * @brief 无新字节时调用，静默至少1750us则结束当前帧。
 *
 * 调用：主循环确认当前RX队列为空后，传入当前us。
 * 过程：active且距末字节至少1750us时finish，否则不操作；不阻塞等待。
 */
void robot_rtu_idle(robot_rtu_t *r,uint32_t us,rtu_frame_fn fn,void *ctx)
{ if(r->active && (uint32_t)(us-r->last_us)>=1750) finish(r,fn,ctx); }
