#ifndef USBD_CONF_H
#define USBD_CONF_H

#include "gd32f30x.h"

/* The board has a fixed 1.5 kOhm pull-up from USB D+ (PA12) to 3.3 V. */
#define USB_DP_PULLUP_FIXED

#define USBD_CFG_MAX_NUM                   1U
#define USBD_ITF_MAX_NUM                   2U

#define CDC_COM_INTERFACE                  0U

#define CDC_CMD_EP                         EP_IN(2U)
#define CDC_IN_EP                          EP_IN(1U)
#define CDC_OUT_EP                         EP_OUT(3U)

#define EP0_RX_ADDR                        0x40U
#define EP0_TX_ADDR                        0x80U
#define INT_TX_ADDR                        0xC0U
#define BULK_RX_ADDR                       0x100U
#define BULK_TX_ADDR                       0x140U

#define CDC_ACM_CMD_PACKET_SIZE            8U
#define CDC_ACM_DATA_PACKET_SIZE           64U

#define EP_COUNT                           4U
#define USB_STRING_COUNT                   4U
#define BTABLE_OFFSET                      0x0000U

#endif /* USBD_CONF_H */
