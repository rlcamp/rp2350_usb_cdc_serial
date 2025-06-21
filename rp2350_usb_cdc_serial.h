#include <stddef.h>

void usb_cdc_serial_init(void);
char get_dtr(void);

const void * usb_cdc_serial_out_staging_area(void);
size_t usb_cdc_serial_out_filled(void);
void usb_cdc_serial_out_rearm(void);

void * usb_cdc_serial_in_staging_area(void);
void usb_cdc_serial_in_start_sending(const size_t size);
int usb_cdc_serial_in_still_sending(void);

void unaligned_memcpy(void *, const void * restrict, size_t count);

/* mostly for debug */
extern unsigned char enumerated;
