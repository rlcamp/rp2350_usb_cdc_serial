#include <stddef.h>

void usb_cdc_serial_init(void);
void usb_cdc_serial_deinit(void);

/* loop on this until it returns nonzero */
int usb_cdc_serial_dtr_is_high(void);

/* connection lasts until this returns nonzero */
int usb_cdc_serial_dtr_has_gone_low(void);

int usb_cdc_serial_rts_has_gone_low(void);

const void * usb_cdc_serial_rx_staging_area(void);
size_t usb_cdc_serial_rx_filled(void);
void usb_cdc_serial_rx_rearm(void);

void * usb_cdc_serial_tx_acquire(const size_t size_wanted);
void usb_cdc_serial_tx_start(const void * pointer, const size_t size);
int usb_cdc_serial_tx_still_sending(void);

/* do not mix calls to this with regular acquire function */
void * usb_cdc_serial_tx_acquire_half(const size_t size_wanted);

void unaligned_memcpy(void *, const void * restrict, size_t count);
