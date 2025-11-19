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

    while (1) {
        while (!usb_cdc_serial_dtr_is_high())
            yield();

        printf("connected\r\n");

        for (size_t ipass = 0; ipass < 2; ipass++) {
            static const char wherry[] = "In 1520 Sir Alexander Stewart of Invernahyle was fishing off the small island next to Castle Stalker when he was surprised and murdered by a party of Campbells. Tradition has it that the nurse of his baby son, Donald Stewart, hid the baby in the Castle and when the Campbells left the nurse returned, found the baby still alive and took refuge in Morven. In around 1620 the Castle passed into the hands of the Campbells of Airds as a result of a drunken wager by the 7th Stewart Chief, Duncan, in exchange for an eight-oared wherry.\r\n";

            while (!usb_cdc_serial_dtr_has_gone_low() && usb_cdc_serial_tx_still_sending())
                yield();

            if (usb_cdc_serial_dtr_has_gone_low()) break;

            void * staging_area = usb_cdc_serial_tx_staging_area(sizeof(wherry) - 1);
            unaligned_memcpy(staging_area, wherry, sizeof(wherry) - 1);

            usb_cdc_serial_tx_start(staging_area, sizeof(wherry) - 1);
        }

        const void * out_buf = usb_cdc_serial_rx_staging_area();

        while (!usb_cdc_serial_dtr_has_gone_low()) {
            const size_t bytes_to_echo = usb_cdc_serial_rx_filled();
            if (!bytes_to_echo) {
                yield();
                continue;
            }

            while (!usb_cdc_serial_dtr_has_gone_low() && usb_cdc_serial_tx_still_sending())
                yield();

            if (usb_cdc_serial_dtr_has_gone_low()) break;

            void * staging_area = usb_cdc_serial_tx_staging_area(bytes_to_echo);

            unaligned_memcpy(staging_area, out_buf, bytes_to_echo);

            usb_cdc_serial_tx_start(staging_area, bytes_to_echo);
            usb_cdc_serial_rx_rearm();

            printf("forwarded %zu bytes\r\n", bytes_to_echo);
        }

        printf("disconnected\r\n");
    }
}

