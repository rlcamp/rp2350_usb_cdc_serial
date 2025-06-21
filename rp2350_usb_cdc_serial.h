#include <stddef.h>

void usb_cdc_serial_init(void);
char get_dtr(void);

const void * usb_cdc_serial_rx_staging_area(void);
size_t usb_cdc_serial_rx_filled(void);
void usb_cdc_serial_rx_rearm(void);

void * usb_cdc_serial_tx_staging_area(void);
void usb_cdc_serial_tx_start(const size_t size);
int usb_cdc_serial_tx_still_sending(void);

void unaligned_memcpy(void *, const void * restrict, size_t count);

/* mostly for debug */
extern unsigned char enumerated;
