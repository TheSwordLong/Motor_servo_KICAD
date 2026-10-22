/*
 * board_config.h — 板级编译选项。当前机器人入口为 robot_port；RS485/CAN 测试宏不等于当前主循环启用功能。
 * 引脚和电气说明来自工程记录，不能替代实物测量；USART0 的 Modbus 波特率另见 usart0.h。
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/*
 * Central board configuration for the GD32F303VET6 control board.
 *
 * Verified hardware (per RS485.kicad_sch / netlist exported 2026-09-22):
 *
 *   RS485_TX   PA2  (LQFP100 pin 25)  USART1_TX, default mapping, AF push-pull
 *   RS485_RX   PA3  (LQFP100 pin 26)  USART1_RX, default mapping
 *   RS485_DE   PA1  (LQFP100 pin 24)  plain push-pull GPIO, high = transmit
 *
 *   SWDIO      PA13 (pin 72)          kept as SWD
 *   SWCLK      PA14 (pin 76)          kept as SWD
 *
 * Signal path:
 *   TX: PA2 -> U9.3 VIA -> U9.14 VOA -> U10.4 DI
 *   RX: U10.1 RO -> U9.12 VIC -> U9.5 VOC -> PA3
 *   DE: PA1 -> R18 (10k series) -> U9.4 VIB -> U9.13 VOB -> R20 (10k series)
 *              -> U10.3 DE and U10.2 /RE
 *
 * U9  = pi131U30 three-channel digital isolator, 150 kbps version.
 * U10 = SIT3485ESA half-duplex RS-485 transceiver.
 *
 * IMPORTANT: the library name is USART1 (not USART2). PA1 is nominally
 * USART1_RTS, but this design uses it as a software-controlled DE GPIO, so
 * hardware RTS/CTS flow control must stay disabled.
 *
 * PA1 = 0 -> driver off, receiver on. PA1 = 1 -> driver on, receiver off.
 * The local receiver is therefore disabled while transmitting: the board does
 * NOT echo its own transmission.
 */

/* ------------------------------------------------------------------ */
/* Pin assignment                                                      */
/* ------------------------------------------------------------------ */

#define BOARD_RS485_GPIO_PORT       GPIOA
#define BOARD_RS485_TX_PIN          GPIO_PIN_2
#define BOARD_RS485_RX_PIN          GPIO_PIN_3
#define BOARD_RS485_DE_PIN          GPIO_PIN_1
#define BOARD_RS485_GPIO_CLOCK      RCU_GPIOA

/* DE level that enables the driver (transmit direction). */
#define BOARD_RS485_DE_TX_LEVEL     1U

/*
 * Force the DE pin to a fixed level and leave it there, ignoring the normal
 * transmit/receive switching. Use this to probe the transceiver direction
 * empirically.
 *
 *   0 = normal operation (DE idle low = receive, pulsed high to transmit)
 *   1 = force idle high (driver enabled direction)
 *
 * NOTE: per the schematic notes, PA1 = 1 turns the driver ON and the receiver
 * OFF, so forcing it high should DISABLE reception. If the board still receives
 * with PA1 high, the DE/RE polarity or the R20 wiring differs from that note.
 */
#define BOARD_RS485_FORCE_DE_IDLE_HIGH  0U

/* ------------------------------------------------------------------ */
/* Serial parameters                                                   */
/* ------------------------------------------------------------------ */

/*
 * Default 9600 8N1.
 *
 * U9 (pi131U30) is the 150 kbps version, so 9600 is the safe starting point.
 * Allowed compile-time values: 9600, 19200, 38400, 57600, 115200.
 * The first test round MUST start at 9600.
 */
#define BOARD_RS485_BAUDRATE        9600U

/* ------------------------------------------------------------------ */
/* DE switching timing                                                 */
/* ------------------------------------------------------------------ */

/*
 * Conservative starting values, NOT hardware guarantees. They must be tuned
 * against measured isolator delay plus the R18/R20 series-resistor delay.
 */
#define BOARD_RS485_DE_ASSERT_US    50U   /* DE high -> first start bit      */
#define BOARD_RS485_DE_HOLD_US      50U   /* last stop bit -> DE low         */

/* Default per-frame transmit timeout. Callers may override per call. */
#define BOARD_RS485_TX_TIMEOUT_MS   500U

/* Ring buffer size. The spec requires at least 512 bytes. */
#define BOARD_RS485_RX_BUF_SIZE     512U

/* Maximum command line accepted by the test shell (including NUL). */
#define BOARD_RS485_LINE_SIZE       64U

/* ------------------------------------------------------------------ */
/* Active firmware: robot_port + robot; legacy debug shells are inactive. */
/* MOTOR_TEST_SELFTEST_PWM belongs only to the retained historical motor_test
 * source. main.c never calls it; first boot keeps all four channels disabled. */
#define MOTOR_TEST_SELFTEST_PWM     0

/*
 * SW1 emergency-stop button on PA15.
 *
 * This board has NOTHING connected to PA15: it is only held high by the MCU
 * internal pull-up. A floating input is noise sensitive, and a spurious press
 * both blocks the main loop for the 20 ms debounce and stops all four motors,
 * so the button path is disabled here.
 *
 *   1 = poll PA15 and treat a low level as "stop all motors"
 *   0 = do not poll PA15 (recommended while nothing is wired to it)
 */
#define BUTTON_ESTOP_ENABLE         0

/*
 * The RS-485 test shell is not part of this build. Keep it at 0; see
 * docs/RS485_测试说明.md for how to bring the RS-485 driver back.
 */
#define RS485_TEST_ENABLE           0

/* Mode A / mode A2: period of the automatic line (RS-485 test shell only). */
#define BOARD_RS485_MODE_A_PERIOD_MS 1000U

/* RX mode: capacity of the global g_rs485_line accumulator. */
#define BOARD_RS485_LINE_BUF_SIZE   128U

/*
 * RX mode handshake: interval between "ready" frames sent while waiting for the
 * host. The board stops transmitting as soon as any byte arrives.
 */
#define BOARD_RS485_READY_PERIOD_MS 100U

/*
 * Optional advanced test modes. Keep these OFF for the first round; enable
 * them only after modes A and B pass.
 */
#define RS485_TEST_MODE_C_BINARY    0   /* binary integrity test  */
#define RS485_TEST_MODE_D_MODBUS    0   /* Modbus RTU slave       */

/* ------------------------------------------------------------------ */
/* CAN self-test                                                       */
/* ------------------------------------------------------------------ */

/*
 * Internal loopback self-test for CAN0.
 *
 *   1 = the CAN controller runs in CAN_LOOPBACK_MODE. Every transmitted frame
 *       is looped back inside the controller and received by itself, so the
 *       controller, bit timing and acceptance filter can be verified with no
 *       second node, no transceiver and no bus wiring.
 *
 *   0 = normal mode, needs a real transceiver and a second node (or a CAN
 *       analyser) to acknowledge frames.
 */
#define CAN_SELF_TEST_LOOPBACK      0

/* Frames sent by the self-test before it declares completion. */
#define CAN_SELF_TEST_BLOCKS        100U

/* Per-frame loopback timeout, milliseconds. */
#define CAN_SELF_TEST_TIMEOUT_MS    50U

/*
 * How many times a frame is re-sent when the loopback copy does not arrive in
 * time, before the block is given up and counted in can_st_fail. The payload
 * is unchanged across retries, so a late frame still matches its own block.
 */
#define CAN_SELF_TEST_MAX_RETRY     3U

/*
 * Normal mode: automatic transmit period, milliseconds.
 *
 * This board has no SW1 button, so in normal mode the firmware sends one test
 * frame (ID 0x321, payload "GD32CAN" + sequence) automatically at this rate.
 * Set to 0 to disable the automatic sender and only receive.
 */
#define CAN_AUTO_TX_PERIOD_MS       1000U

/*
 * Receive-to-transmit forwarding: every received frame is sent back out with
 * this offset added to its 11-bit ID, so the origin can distinguish the reply
 * from its own transmission.
 *
 * CAUTION: enable this on ONE side only. If both nodes forward, a frame bounces
 * between them forever.
 */
#define CAN_ECHO_ID_OFFSET          0x100U

/* ------------------------------------------------------------------ */
/* Clock assumption                                                    */
/* ------------------------------------------------------------------ */

/*
 * The board has an 8 MHz crystal, but its load capacitors and startup margin
 * are not yet measured. The existing project clock configuration is reused
 * (system_clock_120m_hxtal), and the USART baud divider is derived from the
 * actual APB1 frequency via rcu_clock_freq_get(CK_APB1) rather than from a
 * hard-coded SystemCoreClock assumption.
 *
 * NOTE: the current CMSIS clock setup busy-waits forever if HXTAL never
 * becomes stable (system_gd32f30x.c:911). A timeout plus IRC8M fallback is
 * specified but NOT implemented here; see README for the open item.
 */
#define BOARD_EXPECTED_SYSCLK_HZ    120000000U

#endif /* BOARD_CONFIG_H */
