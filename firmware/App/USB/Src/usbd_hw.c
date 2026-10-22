#include "usbd_hw.h"

#include "gd32f30x.h"
#include "systick.h"

void usb_dp_force_disconnect(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);

    /*
     * The board has a fixed 1.5 kOhm pull-up from PA12 (D+) to 3.3 V.
     * Hold D+ low long enough for the host to detect a real disconnect.
     */
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
    gpio_bit_reset(GPIOA, GPIO_PIN_12);
    delay_1ms(20U);
}

void usb_dp_release_connect(void)
{
    /* Release PA12 only after the USB device core is ready. */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
}

void usb_rcu_config(void)
{
    /* 120 MHz PLL / 2.5 = 48 MHz USB clock. */
    rcu_usb_clock_config(RCU_CKUSB_CKPLL_DIV2_5);
    rcu_periph_clock_enable(RCU_USBD);
}

void usb_nvic_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 2U, 0U);
}
