#include <stddef.h>

void usb_cdc_serial_init(void);
void usb_cdc_serial_deinit(void);

/* loop on this until it returns nonzero */
int usb_cdc_serial_dtr_is_high(void);

/* connection lasts until this returns nonzero */
int usb_cdc_serial_dtr_has_gone_low(void);

const void * usb_cdc_serial_rx_staging_area(void);
size_t usb_cdc_serial_rx_filled(void);
void usb_cdc_serial_rx_rearm(void);

void * usb_cdc_serial_tx_staging_area(const size_t size_wanted);
void usb_cdc_serial_tx_start(const size_t size);
int usb_cdc_serial_tx_still_sending(void);

void unaligned_memcpy(void *, const void * restrict, size_t count);
