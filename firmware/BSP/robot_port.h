/*
 * robot_port.h — GD32 硬件适配层：连接状态机、FG、GPIO、Flash、USART0 Modbus 与 CAN。
 * 主循环推进业务；SysTick 自检保护可先切断输出并锁存反馈，随后主循环消费该快照。
 * 普通协议处理不在 ISR 内执行，Flash 仅在禁止运动且反馈稳定时写入。
 */
#ifndef ROBOT_PORT_H
#define ROBOT_PORT_H
#include <stdint.h>
/**
 * @brief 初始化安全输出、加载Flash/UID、启动FG及选定通信；协议0为USART0，1为CAN。
 *
 * 顺序：GPIO/PWM安全初态，释放JTAG，按宏配置停机输入，加载最新有效存储。
 * 身份：从芯片UID地址读3字，注入输出/保存/频率回调，初始化robot。
 * 完成：启动捕获，按保存的protocol选择USART0或CAN，二者不是同时运行的两条命令通道。
 */
void robot_port_init(void);
/**
 * @brief 读取一致FG快照，先推进控制再轮询通信；自检ISR锁存停机快照优先于更新的余转数据。
 *
 * 过程：短临界区选择ISR截停快照或当前FG快照，启用按键时先处理停机。
 * 顺序：robot_update消费快照后清latched，再收发协议，按键保持时再次停机。
 * 目的：余转不能先于截停快照消费，否则计数相减会回绕；新余转留下一轮处理。
 */
void robot_port_update(void);
/* 1 ms interrupt: output cutoff only, never Flash or protocol work. */
/**
 * @brief SysTick 自检保护：达到脉冲目标/通电时限或捕获异常先切断输出，锁存快照供主循环处理。
 *
 * 上下文：SysTick；只在probe_armed时工作，避免主循环堵塞使自检无限通电。
 * 检查：2秒限时、当前路目标脉冲、任一路捕获错误、其他路剩余非预期额度。
 * 动作：PWM0/BRAKE1，保存cutoff反馈及停止时刻，清armed并置latched。
 * 限制：不解析命令、不写Flash；失败原因和回程由主循环决定。
 */
void robot_selftest_safety_tick(void);
/**
 * @brief 复位早期/异常路径强制 PWM高、BRAKE低，不依赖全局初始化或改变DIR；复位保持期间无法控制输出。
 *
 * 调用：Reset_Handler在SystemInit前，以及异常停机路径。
 * 过程：启用所需GPIO，PWM四路改推挽高，BRAKE低制动，DIR保持原值。
 * 限制：不访问未初始化业务全局、不依赖浮点/计时；重置为GPIO后正常PWM需重新初始化。
 */
void robot_emergency_stop(void);
/**
 * @brief 读取 SysTick 生成可回绕的32位 us 时间戳；短暂屏蔽中断并补偿挂起节拍。
 *
 * 过程：保存PRIMASK、关中断读取ms和SysTick计数；节拍挂起时补1ms。
 * 返回：32位微秒，约71.6分钟回绕，调用者使用无符号时间差。
 * 限制：依赖SysTick和SystemCoreClock已初始化，不用于Reset_Handler早期。
 */
uint32_t robot_micros(void);
#endif
