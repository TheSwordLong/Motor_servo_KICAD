/*
 * can_bus.h — CAN0 底层初始化和保留的测试接口：PD0 RX、PD1 TX，按 APB1=60 MHz 配置 500 kbit/s。
 * 当前机器人业务由 robot_port 的非阻塞轮询收发，测试帧/回环步骤不会自动执行。
 */
#ifndef BSP_CAN_BUS_H
#define BSP_CAN_BUS_H

#include <stdint.h>

#define CAN_BUS_TEST_ID 0x321U

/* CAN0 on PD0 (RX) / PD1 (TX) via full remap, normal mode, 500 kbit/s. */
/**
 * @brief 初始化 CAN0；成功返回 1，硬件初始化失败返回 0，模式由编译宏决定。
 *
 * 过程：配置GPIO、复位CAN；启用bus-off自动恢复，禁止硬件自动重发。
 * 时序：prescaler=8，BS1=12，BS2=2；仅当CAN输入60MHz时为500kbit/s。
 * 输出：can_init失败返回0，否则安装过滤器返回1；它不证明总线上有其他节点。
 */
uint8_t can_bus_init(void);

/* Sends one standard 8-byte test frame. Returns 1 only after a bus ACK. */
/**
 * @brief 发送固定测试 ID 的 GD32CAN 加一字节序号载荷；返回底层发送结果。
 *
 * 输入：sequence为8位测试序号，拼在7字节GD32CAN文本后。
 * 输出：固定ID为CAN_BUS_TEST_ID；返回底层发送结果，不执行电机业务。
 */
uint8_t can_bus_send_test(uint8_t sequence);

/*
 * Send one standard data frame with an arbitrary ID and payload.
 * Returns 1 when the frame was acknowledged on the bus, 0 on no mailbox,
 * transmit failure or timeout. Used by the receive-to-transmit forwarder.
 */
/**
 * @brief 发送 1～8 字节标准数据帧；返回 1 表示发送成功，0 表示参数/邮箱/发送失败；轮询等待有上限。
 *
 * 输入：id应由调用者限制为11位标准ID；data须可读length字节，length=1～8。
 * 过程：申请邮箱后最多轮询1000000次状态，超限取消邮箱；此上限不是准确毫秒数。
 * 输出：仅CAN_TRANSMIT_OK返回1；邮箱不足/发送失败/超时/参数错返回0。
 * 限制：阻塞测试接口；当前robot_port使用自己的非阻塞发送状态机。
 */
uint8_t can_bus_send_frame(uint32_t id, const uint8_t *data, uint8_t length);

/* Polls FIFO0. Returns 1 and fills the outputs when a frame is available. */
/**
 * @brief 从 FIFO0 取一帧；空返回 0，取到返回 1；非空输出指针分别接收 ID、数据及长度。
 *
 * 输入：data非NULL时调用者需提供至少8字节空间；id/length允许NULL。
 * 过程：FIFO空立即返回0；否则弹出一帧，按标准/扩展类型选择ID并复制载荷。
 * 限制：此接口不筛除远程帧或业务ID，上层必须自行判定用途。
 */
uint8_t can_bus_receive(uint32_t *id, uint8_t *data, uint8_t *length);

/* ------------------------------------------------------------------ */
/* Internal loopback self-test                                         */
/* ------------------------------------------------------------------ */

/*
 * When CAN_SELF_TEST_LOOPBACK is enabled (see BSP/board_config.h) the CAN
 * controller runs in CAN_LOOPBACK_MODE: every transmitted frame is looped back
 * internally and received by the controller itself. This verifies the CAN
 * controller, the bit timing and the acceptance filter without a second node,
 * a transceiver or any bus wiring.
 *
 * Call can_self_test_step() from the main loop. It sends one frame, waits for
 * the looped-back copy, and compares it byte for byte.
 */
/**
 * @brief 非阻塞推进回环测试、超时与重试计数；需由调用者周期执行，现役主循环未调用。
 *
 * 过程：IDLE生成载荷/排队，WAIT检查返回帧或截止时间；比较ID、长度和全部8字节。
 * 超时：同一块最多重试CAN_SELF_TEST_MAX_RETRY次，耗尽后记失败并进入下一块。
 * 结果：所有块处理结束才设置done；rx_bad/fail非零时不能宣称测试通过。
 * 限制：需循环调用；不能同时让业务接收器消费同一FIFO。
 */
void can_self_test_step(void);

/* 0 while the self-test is still running, 1 when it has finished. */
/**
 * @brief 返回测试流程结束标志；该标志不等于所有块校验成功，还须检查失败/坏帧统计。
 *
 * 返回：can_st_passed的8位值；只说明整个块序列已处理完。
 * 读取：结合can_st_rx_ok、can_st_rx_bad、can_st_fail判断实际质量，不会清统计。
 */
uint8_t can_self_test_done(void);

/* Self-test counters, all watchable in the debugger. */
extern volatile uint32_t can_st_block;        /* blocks completed, 0..BLOCKS */
extern volatile uint32_t can_st_tx_attempts;  /* can_message_transmit calls   */
extern volatile uint32_t can_st_tx_queued;    /* transmit calls accepted      */
extern volatile uint32_t can_st_tx_nomailbox; /* transmit rejected: no mailbox */
extern volatile uint32_t can_st_tx_retries;   /* frames re-sent after timeout */
extern volatile uint32_t can_st_rx_ok;        /* received and matching        */
extern volatile uint32_t can_st_rx_bad;       /* received but wrong payload   */
extern volatile uint32_t can_st_timeout;      /* frames that needed a retry   */
extern volatile uint32_t can_st_fail;         /* blocks given up after retries */
extern volatile uint32_t can_st_passed;       /* 1 = 测试流程完成，仍须检查错误 */
extern volatile uint8_t  can_st_last_data[8]; /* last payload received        */

#endif /* BSP_CAN_BUS_H */
