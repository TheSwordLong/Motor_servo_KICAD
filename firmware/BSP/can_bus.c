/*
 * can_bus.c — CAN0 底层初始化和保留的测试接口：PD0 RX、PD1 TX，按 APB1=60 MHz 配置 500 kbit/s。
 * 当前机器人业务由 robot_port 的非阻塞轮询收发，测试帧/回环步骤不会自动执行。
 */
#include "can_bus.h"

#include <stddef.h>

#include "board_config.h"
#include "gd32f30x.h"
#include "systick.h"

/**
 * @brief 配置 CAN0 全重映射 PD0/PD1；不启用会占用晶振引脚的 PD01 重映射。
 *
 * 过程：启用AF/GPIOD时钟，CAN0全重映射；PD0上拉输入，PD1复用推挽输出。
 * 限制：GPIO_PD01_REMAP必须保持关闭，以免晶振引脚映射与CAN冲突；不代表已连接收发器。
 */
static void can_gpio_init(void)
{
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_GPIOD);

    /*
     * CAN0 full remap: PD0 = CAN_RX, PD1 = CAN_TX.
     *
     * The HXTAL stays on the dedicated OSC_IN/OSC_OUT pins (package pins 12/13),
     * so GPIO_PD01_REMAP must stay disabled. Enabling it would move the crystal
     * onto PD0/PD1 and collide with CAN0.
     */
    gpio_pin_remap_config(GPIO_CAN_FULL_REMAP, ENABLE);

    gpio_init(GPIOD, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    gpio_init(GPIOD, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
}

/**
 * @brief 配置接收过滤器接受全部标识符到 FIFO0；机器人 ID 由上层再筛选。
 *
 * 过程：过滤器0使用32位掩码模式，掩码全0，匹配帧进入FIFO0。
 * 限制：该过滤器不区分业务节点，后续robot_port仍检查标准帧、数据帧、长度和ID。
 */
static void can_filter_accept_all_init(void)
{
    can_filter_parameter_struct filter;

    can_struct_para_init(CAN_FILTER_STRUCT, &filter);
    filter.filter_number = 0U;
    filter.filter_mode = CAN_FILTERMODE_MASK;
    filter.filter_bits = CAN_FILTERBITS_32BIT;
    filter.filter_list_high = 0U;
    filter.filter_list_low = 0U;
    filter.filter_mask_high = 0U;
    filter.filter_mask_low = 0U;
    filter.filter_fifo_number = CAN_FIFO0;
    filter.filter_enable = ENABLE;
    can_filter_init(&filter);
}

/**
 * @brief 初始化 CAN0；成功返回 1，硬件初始化失败返回 0，模式由编译宏决定。
 *
 * 过程：配置GPIO、复位CAN；启用bus-off自动恢复，禁止硬件自动重发。
 * 时序：prescaler=8，BS1=12，BS2=2；仅当CAN输入60MHz时为500kbit/s。
 * 输出：can_init失败返回0，否则安装过滤器返回1；它不证明总线上有其他节点。
 */
uint8_t can_bus_init(void)
{
    can_parameter_struct parameter;

    can_gpio_init();
    rcu_periph_clock_enable(RCU_CAN0);
    can_deinit(CAN0);

    can_struct_para_init(CAN_INIT_STRUCT, &parameter);
    parameter.time_triggered = DISABLE;
    parameter.auto_bus_off_recovery = ENABLE;
    parameter.auto_wake_up = DISABLE;
    /* One button event makes one attempt; avoid repeated faults driving CAN bus-off. */
    parameter.auto_retrans = DISABLE;
    parameter.rec_fifo_overwrite = DISABLE;
    parameter.trans_fifo_order = DISABLE;
    /* This field uses CAN_*_MODE communication constants, not CAN_MODE_* state commands. */
#if (1 == CAN_SELF_TEST_LOOPBACK)
    /*
     * Loopback: the controller receives its own transmissions internally, so no
     * second node, transceiver or bus wiring is needed. Also enables silent
     * behaviour (rx only observes internally), which is what we want for a
     * self-test.
     */
    parameter.working_mode = CAN_LOOPBACK_MODE;
#else
    parameter.working_mode = CAN_NORMAL_MODE;
#endif

    /*
     * APB1 CAN clock = 60 MHz.
     * 60 MHz / (prescaler 8 * (1 + BS1 12 + BS2 2)) = 500 kbit/s.
     * Sample point = (1 + 12) / 15 = 86.7%.
     */
    parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    parameter.time_segment_1 = CAN_BT_BS1_12TQ;
    parameter.time_segment_2 = CAN_BT_BS2_2TQ;
    parameter.prescaler = 8U;

    if(SUCCESS != can_init(CAN0, &parameter)) {
        return 0U;
    }

    can_filter_accept_all_init();
    return 1U;
}

/**
 * @brief 发送 1～8 字节标准数据帧；返回 1 表示发送成功，0 表示参数/邮箱/发送失败；轮询等待有上限。
 *
 * 输入：id应由调用者限制为11位标准ID；data须可读length字节，length=1～8。
 * 过程：申请邮箱后最多轮询1000000次状态，超限取消邮箱；此上限不是准确毫秒数。
 * 输出：仅CAN_TRANSMIT_OK返回1；邮箱不足/发送失败/超时/参数错返回0。
 * 限制：阻塞测试接口；当前robot_port使用自己的非阻塞发送状态机。
 */
uint8_t can_bus_send_frame(uint32_t id, const uint8_t *data, uint8_t length)
{
    can_transmit_message_struct message;
    uint8_t mailbox;
    uint32_t timeout;
    can_transmit_state_enum state;
    uint8_t index;

    if((NULL == data) || (0U == length) || (length > 8U)) {
        return 0U;
    }

    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &message);
    message.tx_sfid = id;
    message.tx_ff = CAN_FF_STANDARD;
    message.tx_ft = CAN_FT_DATA;
    message.tx_dlen = length;

    for(index = 0U; index < length; index++) {
        message.tx_data[index] = data[index];
    }

    mailbox = can_message_transmit(CAN0, &message);
    if(CAN_NOMAILBOX == mailbox) {
        return 0U;
    }

    /* Queued is not the same as transmitted: wait for ACK or a bounded timeout. */
    timeout = 1000000U;
    do {
        state = can_transmit_states(CAN0, mailbox);
        if(CAN_TRANSMIT_OK == state) {
            return 1U;
        }
        if(CAN_TRANSMIT_FAILED == state) {
            return 0U;
        }
        timeout--;
    } while(0U != timeout);

    (void)can_transmission_stop(CAN0, mailbox);
    return 0U;
}

/**
 * @brief 发送固定测试 ID 的 GD32CAN 加一字节序号载荷；返回底层发送结果。
 *
 * 输入：sequence为8位测试序号，拼在7字节GD32CAN文本后。
 * 输出：固定ID为CAN_BUS_TEST_ID；返回底层发送结果，不执行电机业务。
 */
uint8_t can_bus_send_test(uint8_t sequence)
{
    uint8_t payload[8];

    payload[0] = 'G';
    payload[1] = 'D';
    payload[2] = '3';
    payload[3] = '2';
    payload[4] = 'C';
    payload[5] = 'A';
    payload[6] = 'N';
    payload[7] = sequence;

    return can_bus_send_frame(CAN_BUS_TEST_ID, payload, 8U);
}

/**
 * @brief 从 FIFO0 取一帧；空返回 0，取到返回 1；非空输出指针分别接收 ID、数据及长度。
 *
 * 输入：data非NULL时调用者需提供至少8字节空间；id/length允许NULL。
 * 过程：FIFO空立即返回0；否则弹出一帧，按标准/扩展类型选择ID并复制载荷。
 * 限制：此接口不筛除远程帧或业务ID，上层必须自行判定用途。
 */
uint8_t can_bus_receive(uint32_t *id, uint8_t *data, uint8_t *length)
{
    can_receive_message_struct message;
    uint8_t index;

    if(0U == can_receive_message_length_get(CAN0, CAN_FIFO0)) {
        return 0U;
    }

    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &message);
    can_message_receive(CAN0, CAN_FIFO0, &message);

    if(NULL != id) {
        *id = (CAN_FF_STANDARD == message.rx_ff) ? message.rx_sfid : message.rx_efid;
    }

    if(NULL != length) {
        *length = message.rx_dlen;
    }

    if(NULL != data) {
        for(index = 0U; index < message.rx_dlen; index++) {
            data[index] = message.rx_data[index];
        }
    }

    return 1U;
}

/* ------------------------------------------------------------------ */
/* Internal loopback self-test                                         */
/* ------------------------------------------------------------------ */

volatile uint32_t can_st_block = 0U;
volatile uint32_t can_st_tx_attempts = 0U;
volatile uint32_t can_st_tx_queued = 0U;
volatile uint32_t can_st_tx_nomailbox = 0U;
volatile uint32_t can_st_tx_retries = 0U;
volatile uint32_t can_st_rx_ok = 0U;
volatile uint32_t can_st_rx_bad = 0U;
volatile uint32_t can_st_timeout = 0U;
volatile uint32_t can_st_fail = 0U;
volatile uint32_t can_st_passed = 0U;
volatile uint8_t  can_st_last_data[8] = {0U};

/* Non-blocking states of one self-test frame. */
typedef enum {
    CAN_ST_IDLE = 0,
    CAN_ST_WAIT = 1
} can_self_test_state_enum;

static can_self_test_state_enum can_st_state = CAN_ST_IDLE;
static uint32_t can_st_deadline = 0U;
static uint8_t can_st_block_id = 0U;
static uint8_t can_st_retry = 0U;
static uint8_t can_st_tx_payload[8];

/*
 * Build the payload for one block.
 *
 * Every byte depends on block_id, so a payload received in block N can only
 * match block N. Re-receiving a stale frame from a previous block therefore
 * shows up as a mismatch instead of a false pass.
 */
/**
 * @brief 按块序号生成 8 字节测试内容，使迟到的旧块不易被误认为当前块。
 *
 * 输入：block_id为当前块序号，payload须能写8字节。
 * 过程：每字节按block_id*31+index*7+0x5A生成并截为8位，不访问硬件。
 */
static void can_st_build_payload(uint8_t block_id, uint8_t *payload)
{
    uint8_t index;

    for(index = 0U; index < 8U; index++) {
        payload[index] = (uint8_t)(block_id * 31U + index * 7U + 0x5AU);
    }
}

/* Drop every frame currently in FIFO0. */
/**
 * @brief 清空 FIFO0 当前积压帧，仅用于历史 CAN 测试准备。
 *
 * 过程：循环弹出FIFO0，直到当前为空；丢弃的数据不会再交给业务层。
 * 限制：仅在测试准备时使用；总线持续输入时循环耗时可能增加。
 */
static void can_st_flush_rx(void)
{
    can_receive_message_struct rx;

    while(0U != can_receive_message_length_get(CAN0, CAN_FIFO0)) {
        can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &rx);
        can_message_receive(CAN0, CAN_FIFO0, &rx);
    }
}

/* Queue one frame carrying can_st_tx_payload. Returns 1 when accepted. */
/**
 * @brief 尝试排入一个测试帧；成功返回 1 并开始等待，邮箱不足返回 0 留待下次。
 *
 * 输入：使用全局can_st_tx_payload和当前块状态，不接受外部参数。
 * 成功：更新queued计数，进入WAIT，deadline=当前ms+测试超时。
 * 失败：无邮箱只增加nomailbox并返回0，不进入等待；调用者下次再试。
 */
static uint8_t can_st_transmit(void)
{
    can_transmit_message_struct tx;
    uint8_t index;

    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &tx);
    tx.tx_sfid = CAN_BUS_TEST_ID;
    tx.tx_ff = CAN_FF_STANDARD;
    tx.tx_ft = CAN_FT_DATA;
    tx.tx_dlen = 8U;

    for(index = 0U; index < 8U; index++) {
        tx.tx_data[index] = can_st_tx_payload[index];
    }

    can_st_tx_attempts++;

    if(CAN_NOMAILBOX == can_message_transmit(CAN0, &tx)) {
        can_st_tx_nomailbox++;
        return 0U;
    }

    can_st_tx_queued++;
    can_st_state = CAN_ST_WAIT;
    can_st_deadline = sys_tick_ms + CAN_SELF_TEST_TIMEOUT_MS;
    return 1U;
}

/**
 * @brief 非阻塞推进回环测试、超时与重试计数；需由调用者周期执行，现役主循环未调用。
 *
 * 过程：IDLE生成载荷/排队，WAIT检查返回帧或截止时间；比较ID、长度和全部8字节。
 * 超时：同一块最多重试CAN_SELF_TEST_MAX_RETRY次，耗尽后记失败并进入下一块。
 * 结果：所有块处理结束才设置done；rx_bad/fail非零时不能宣称测试通过。
 * 限制：需循环调用；不能同时让业务接收器消费同一FIFO。
 */
void can_self_test_step(void)
{
    if(0U != can_st_passed) {
        return;   /* 测试块已全部处理，是否通过还需读取错误统计。 */
    }

    if(CAN_ST_IDLE == can_st_state) {
        if(can_st_block >= CAN_SELF_TEST_BLOCKS) {
            can_st_passed = 1U;
            return;
        }

        can_st_block_id = (uint8_t)can_st_block;
        can_st_build_payload(can_st_block_id, can_st_tx_payload);

        /* Start from a clean FIFO so no stale frame can be mistaken for this one. */
        if(0U == can_st_retry) {
            can_st_flush_rx();
        }

        (void)can_st_transmit();
        /* On no mailbox: stay idle and retry on the next call. */
        return;
    }

    /* Waiting for the looped-back frame. */
    {
        can_receive_message_struct rx;
        uint8_t index;
        uint8_t match = 1U;

        if(0U == can_receive_message_length_get(CAN0, CAN_FIFO0)) {
            if((int32_t)(sys_tick_ms - can_st_deadline) >= 0) {
                can_st_timeout++;

                /*
                 * Re-send the same payload instead of skipping the block. The
                 * payload depends on block_id, so a late frame from this block
                 * still matches and cannot corrupt the next block.
                 */
                if(can_st_retry < CAN_SELF_TEST_MAX_RETRY) {
                    can_st_retry++;
                    can_st_tx_retries++;
                    can_st_state = CAN_ST_IDLE;
                } else {
                    can_st_fail++;
                    can_st_retry = 0U;
                    can_st_state = CAN_ST_IDLE;
                    can_st_block++;
                }
            }
            return;
        }

        can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &rx);
        can_message_receive(CAN0, CAN_FIFO0, &rx);

        for(index = 0U; index < 8U; index++) {
            can_st_last_data[index] = (index < rx.rx_dlen) ? rx.rx_data[index] : 0U;

            if(can_st_last_data[index] != can_st_tx_payload[index]) {
                match = 0U;
            }
        }

        if((CAN_BUS_TEST_ID == rx.rx_sfid) && (8U == rx.rx_dlen) && (0U != match)) {
            can_st_rx_ok++;
        } else {
            can_st_rx_bad++;
        }

        can_st_retry = 0U;
        can_st_state = CAN_ST_IDLE;
        can_st_block++;
    }
}

/**
 * @brief 返回测试流程结束标志；该标志不等于所有块校验成功，还须检查失败/坏帧统计。
 *
 * 返回：can_st_passed的8位值；只说明整个块序列已处理完。
 * 读取：结合can_st_rx_ok、can_st_rx_bad、can_st_fail判断实际质量，不会清统计。
 */
uint8_t can_self_test_done(void)
{
    return (uint8_t)can_st_passed;
}
