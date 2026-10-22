#ifndef USB_CDC_PORT_H
#define USB_CDC_PORT_H

#include <stdint.h>

void usb_cdc_init(void);
uint8_t usb_cdc_is_configured(void);

/*
 * Start one non-blocking CDC IN transfer (up to 64 bytes).
 * Returns the number of accepted bytes, or 0 if USB is not ready/busy.
 */
uint16_t usb_cdc_write(const uint8_t *data, uint16_t length);

#endif /* USB_CDC_PORT_H */
