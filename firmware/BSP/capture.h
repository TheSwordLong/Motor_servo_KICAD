/*
 * capture.h — 四路单线 FG 上升沿捕获。定时器按实际总线时钟配置为 1 MHz，溢出扩展时间戳。
 * 每减速前电机转一圈按 6 脉冲测速；FG 本身没有方向信息。ISR 更新 g_fg，主循环读取快照。
 */
#ifndef BSP_CAPTURE_H
#define BSP_CAPTURE_H

#include <stdint.h>

/* Fixed FG order: PB10/T1CH2, PE11/T0CH1, PB4/T2CH0, PA0/T1CH0.
 * 6 rising edges per PRE-GEAR shaft revolution. 1MHz timer ticks derived
 * from each APB timer clock. 32-bit software extension; overcapture counted.
 * TIMER0_Channel handles FG2; TIMER0_UP handles its epoch extension.
 * No signed encoder direction is available from these single FG wires. */
#define CAPTURE_US_PER_TICK 1U

/* Encoder pulses per mechanical revolution (same for all four channels). */
#define CAPTURE_PULSES_PER_ROUND 6U

/* Number of FG channels. */
#define CAPTURE_CHANNEL_COUNT 4U

/*
 * Per-channel monitoring data, refreshed on every captured rising edge.
 * Declared volatile so it can be watched live in the debugger:
 *
 *   print g_fg[0].speed_rpm     <- FG1
 *   print g_fg[1].speed_rpm     <- FG2
 */
/* 原始ISR监测量：period_ticks仅16位低位；period_us为扩展周期，count持续累计且无方向。 */
typedef struct {
    volatile uint32_t last_edge_ms;
    volatile uint32_t errors;
    volatile uint16_t period_ticks;  /* edge-to-edge period, counter ticks */
    volatile uint32_t period_us;     /* edge-to-edge period, microseconds */
    volatile uint32_t freq_hz;       /* pulse frequency, Hz              */
    volatile uint32_t speed_rpm;     /* mechanical speed, rpm            */
    volatile uint32_t pulse_count;   /* captured edge counter            */
} fg_monitor_struct;

extern volatile fg_monitor_struct g_fg[CAPTURE_CHANNEL_COUNT];

/* Read-only debug view: [0..3] = M1..M4, sampled by main about every 10 ms.
 * pulse_delta is since the previous snapshot, not a fixed 10 ms rate.
 * level is a sampled GPIO value, not a pulse trace; RPM is pre-gear.
 * The first sample has delta=0; age_ms before any edge is uptime.
 */
typedef struct {
    uint32_t pulse_count;
    uint32_t pulse_delta;
    uint32_t speed_rpm;       /* 0 after >250 ms without FG */
    uint32_t errors;          /* cumulative overcapture events */
    uint32_t level;           /* current GPIO input: 0=low, 1=high */
    uint32_t period_us;       /* last measured period, retained after stop */
    uint32_t last_edge_ms;
    uint32_t age_ms;
    uint32_t sample_ms;       /* snapshot uptime; 0 until first sample */
} fg_debug_struct;
extern volatile fg_debug_struct g_fg_debug[CAPTURE_CHANNEL_COUNT];
/* Call only after capture_init(), from main (no output or Flash effects). */
/**
 * @brief 主循环最多每 10 ms 发布一次调试快照；短暂关中断读取一致数据，速度超过 250 ms 无边沿显示 0。
 *
 * 输入：读取sys_tick_ms、g_fg和物理FG输入电平；无外部参数。
 * 过程：间隔不足10ms直接返回；关中断复制，恢复中断后计算age及pulse_delta。
 * 输出：g_fg_debug[0..3]用于Live Watch；首次pulse_delta=0，无边沿超过250ms时显示RPM=0。
 * 限制：不修改原始捕获计数或运动状态；调试器同时读取整个数组仍可能跨两次发布。
 */
void capture_debug_update(void);

/* Configure all four FG pins and timers for rising-edge input capture. */
/**
 * @brief 启动时初始化四路捕获和三个定时器；共享的 TIMER1 只初始化一次。
 *
 * 过程：清各路测量基准；逐路配置输入，上升沿、无输入分频、数字滤波值8。
 * 时基：APB分频时补定时器倍频，计数器1MHz/16位；同一定时器仅初始化一次。
 * 中断：启用捕获及更新，TIMER0两个入口和TIMER1/2优先级均为1。
 * 限制：面向上电初始化；运行中重复调用会破坏计数基准，不是运动清零接口。
 */
void capture_init(void);

/* Total captured edges on one channel (index 0..3). Returns 0 if out of range. */
/**
 * @brief 读取累计无符号 FG 边沿数；index 为 0～3，越界返回 0，计数不含方向。
 *
 * 输入：index=0～3；返回该通道自采集以来的uint32累计上升沿数。
 * 限制：计数可能回绕；多字段一致性由调用者关中断快照保证，不会自动换算为角度。
 */
uint32_t capture_get_count(uint8_t index);

/* Edge-to-edge period in microseconds. Returns 0 if out of range or not ready. */
/**
 * @brief 返回上次计算的周期 us；越界返回 0，静止时可能保留历史周期。
 *
 * 输入：index=0～3；输出上一次有效测量的周期微秒数。
 * 限制：首边沿可能为0；停转后历史周期不会在本函数清除，不能只凭非零判断仍在转动。
 */
uint32_t capture_get_period_us(uint8_t index);

/* Pulse frequency in Hz. Returns 0 if out of range or not ready. */
/**
 * @brief 返回上次计算的整数频率 Hz；越界返回 0，此接口不做静止超时清零。
 *
 * 输入：index=0～3；输出整数除法得到的Hz，分数部分丢弃。
 * 限制：没有年龄检查；需要当前速度应使用带250ms超时的capture_get_speed_rpm。
 */
uint32_t capture_get_freq_hz(uint8_t index);

/* Mechanical speed in rpm. Returns 0 if out of range or not ready. */
/**
 * @brief 返回减速前 RPM；越界或超过 250 ms 没有边沿返回 0。
 *
 * 输入：index=0～3；依据当前sys_tick_ms与last_edge_ms判断反馈年龄。
 * 返回：有效期内返回减速前RPM，超过250ms或越界返回0；不是输出轴RPM。
 * 限制：零速可能表示还未取得两次有效边沿，不足以单独判断断线。
 */
uint32_t capture_get_speed_rpm(uint8_t index);

#endif /* BSP_CAPTURE_H */
