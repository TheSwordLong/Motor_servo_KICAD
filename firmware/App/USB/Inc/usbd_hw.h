#ifndef USBD_HW_H
#define USBD_HW_H

void usb_rcu_config(void);
void usb_nvic_config(void);
void usb_dp_force_disconnect(void);
void usb_dp_release_connect(void);

#endif /* USBD_HW_H */
