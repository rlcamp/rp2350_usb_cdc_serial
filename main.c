#include "rp2350_usb_cdc_serial.h"

#include "pico/stdio_uart.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"

#include "RP2350.h"

#include <stdio.h>

static void yield(void) {
    /* this can be replaced with yield() from cortex_m_cooperative_multitasking */
    __dsb();
    __wfe();
}

int main(void) {
    set_sys_clock_hz(96000000, true);

    stdio_uart_init();
    printf("standalone usb cdc demo\n");
    usb_cdc_serial_init();

    while (!(*(volatile unsigned char *)&enumerated))
        yield();

    printf("enumerated\r\n");

    while (1) {
        while (!get_dtr())
            yield();

        printf("connected\r\n");

        static const char wherry[] = "In 1520 Sir Alexander Stewart of Invernahyle was fishing off the small island next to Castle Stalker when he was surprised and murdered by a party of Campbells. Tradition has it that the nurse of his baby son, Donald Stewart, hid the baby in the Castle and when the Campbells left the nurse returned, found the baby still alive and took refuge in Morven. In around 1620 the Castle passed into the hands of the Campbells of Airds as a result of a drunken wager by the 7th Stewart Chief, Duncan, in exchange for an eight-oared wherry.\r\n";
        void * staging_area = usb_cdc_serial_tx_staging_area(sizeof(wherry) - 1);
        if (!staging_area) panic("output too large");
        const void * out_buf = usb_cdc_serial_rx_staging_area();

        unaligned_memcpy(staging_area, wherry, sizeof(wherry) - 1);

        for (size_t ipass = 0; ipass < 2 && get_dtr(); ipass++) {
            usb_cdc_serial_tx_start(sizeof(wherry) - 1);

            while (get_dtr() && usb_cdc_serial_tx_still_sending())
                yield();
        }

        while (get_dtr()) {
            const size_t bytes_to_echo = usb_cdc_serial_rx_filled();

            if (bytes_to_echo && !usb_cdc_serial_tx_still_sending()) {
                unaligned_memcpy(staging_area, out_buf, bytes_to_echo);

                usb_cdc_serial_tx_start(bytes_to_echo);
                while (__dsb(), get_dtr() && usb_cdc_serial_tx_still_sending())
                    yield();
                usb_cdc_serial_rx_rearm();
                printf("forwarded %zu bytes\r\n", bytes_to_echo);
            }

            yield();
        }

        printf("disconnected\r\n");
    }
}

