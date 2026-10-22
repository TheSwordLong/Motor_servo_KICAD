#include "usb_cdc_port.h"

#include <stddef.h>
#include <string.h>

#include "cdc_acm_core.h"
#include "usbd_core.h"
#include "usbd_hw.h"

usb_dev usbd_cdc;

void usb_cdc_init(void)
{
    usb_dp_force_disconnect();
    usb_rcu_config();
    usbd_init(&usbd_cdc, &cdc_desc, &cdc_class);
    usb_nvic_config();
    usbd_connect(&usbd_cdc);
    usb_dp_release_connect();
}

uint8_t usb_cdc_is_configured(void)
{
    return (USBD_CONFIGURED == usbd_cdc.cur_status) ? 1U : 0U;
}

uint16_t usb_cdc_write(const uint8_t *data, uint16_t length)
{
    usb_cdc_handler *cdc;

    if((NULL == data) || (0U == length)) {
        return 0U;
    }

    if(USBD_CONFIGURED != usbd_cdc.cur_status) {
        return 0U;
    }

    cdc = (usb_cdc_handler *)usbd_cdc.class_data[CDC_COM_INTERFACE];
    if((NULL == cdc) || (0U == cdc->packet_sent)) {
        return 0U;
    }

    if(length > CDC_ACM_DATA_PACKET_SIZE) {
        length = CDC_ACM_DATA_PACKET_SIZE;
    }

    memcpy(cdc->data, data, length);
    cdc->packet_sent = 0U;
    usbd_ep_send(&usbd_cdc, CDC_IN_EP, cdc->data, length);

    return length;
}
