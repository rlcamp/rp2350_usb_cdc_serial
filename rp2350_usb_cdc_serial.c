/* Minimum viable low overhead USB CDC ACM I/O for RP2350
 Copyright (c) 2025 Richard Campbell

 Partially derived from dev_lowlevel.c in github.com/raspberrypi.pico-examples
 Copyright (c) 2020 Raspberry Pi (Trading) Ltd.

 SPDX-License-Identifier: BSD-3-Clause
 */

#include "rp2350_usb_cdc_serial.h"

/* so that we can reset into bootloader when dtr lowers at 1200 baud */
#include "pico/bootrom.h"

#include "hardware/structs/usb.h"
#include "hardware/regs/usb.h"
#include "hardware/irq.h"
#include "hardware/resets.h"
#include "hardware/sync.h"

/* special macros to modify addresses for set/clear without rmw on rp2350 */
#define usb_hw_set ((usb_hw_t *)hw_set_alias_untyped(usb_hw))
#define usb_hw_clear ((usb_hw_t *)hw_clear_alias_untyped(usb_hw))

#define USB_DIR_OUT 0x00U
#define USB_DIR_IN 0x80U

#define DESCRIPTOR_INTERFACE 0x24

#define USB_TRANSFER_TYPE_CONTROL 0x0
#define USB_TRANSFER_TYPE_BULK 0x2
#define USB_TRANSFER_TYPE_INTERRUPT 0x3

#define USB_DT_DEVICE 0x01
#define USB_DT_CONFIG 0x02
#define USB_DT_STRING 0x03
#define USB_DT_INTERFACE 0x04
#define USB_DT_ENDPOINT 0x05

#define USB_REQUEST_SET_ADDRESS 0x05
#define USB_REQUEST_GET_DESCRIPTOR 0x06
#define USB_REQUEST_SET_CONFIGURATION 0x09

#define USB_DESCRIPTOR_TYPE_ENDPOINT 0x05

#define USB_CONFIG_BUS_POWERED 0x80

#define CDC_SET_LINE_CODING 0x20
#define CDC_GET_LINE_CODING 0x21
#define CDC_SET_CONTROL_LINE_STATE 0x22
#define CDC_SEND_BREAK 0x23

struct usb_setup_packet {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} __packed;

struct usb_device_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
} __packed;

struct usb_configuration_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t bMaxPower;
} __packed;

struct usb_interface_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
} __packed;

struct usb_endpoint_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
} __packed;

struct interface_association_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bFirstInterface;
    uint8_t bInterfaceCount;
    uint8_t bFunctionClass;
    uint8_t bFunctionSubClass;
    uint8_t bFunctionProtocol;
    uint8_t iFunction;
} __attribute((packed));

struct cdc_header_functional_descriptor {
    /* usbcdc11.pdf table 26 */
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint16_t bcdCDC;
} __attribute((packed));

struct union_functional_descriptor {
    /* usbcdc11.pdf table 33 */
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bControlInterface;
    uint8_t bSubordinateInterface;
} __attribute((packed));

struct call_management_functional_descriptor {
    /* usbcdc11.pdf table 27 */
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bmCapabilities;
    uint8_t bDataInterface;
} __attribute((packed));

struct acm_functional_descriptor {
    /* usbcdc11.pdf table 28 */
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bmCapabilities;
} __attribute((packed));

struct cdc_line_info {
    uint32_t dwDTERate;
    uint8_t bCharFormat;
    uint8_t bParityType;
    uint8_t bDataBits;
} __attribute((packed));

#define CDC0_INTERFACE_ACM 0
#define CDC0_INTERFACE_DATA 1

unsigned char enumerated = 0;

static unsigned char __attribute((aligned(8))) ep0_buf[64];

static struct usb_endpoint_configuration {
    /* pointers to registers and places in special memory for this endpoint */
    volatile uint32_t * ep_ctrl;
    volatile uint32_t * ep_buf_ctrl;
    unsigned char * dpram_start;

    union {
        struct { unsigned char * dpram_cursor, * dpram_stop; };
        struct { const unsigned char * in_cursor, * in_stop; };
    };

    /* we need this every time we set ep ctrl */
    uint8_t transfer_type_bits;

    /* toggles on every packet (unless overridden for ep0) */
    unsigned char next_pid;
} * ep0_out = &(struct usb_endpoint_configuration) {
    .transfer_type_bits = USB_TRANSFER_TYPE_CONTROL,
    .ep_buf_ctrl = &usb_dpram->ep_buf_ctrl[0].out,
    .dpram_start = usb_dpram->ep0_buf_a,
}, * ep0_in = &(struct usb_endpoint_configuration) {
    .transfer_type_bits = USB_TRANSFER_TYPE_CONTROL,
    .ep_buf_ctrl = &usb_dpram->ep_buf_ctrl[0].in,
    .dpram_start = usb_dpram->ep0_buf_a,
}, * ep1_in = &(struct usb_endpoint_configuration) {
    .transfer_type_bits = USB_TRANSFER_TYPE_INTERRUPT,
    /* note ep_ctrls count from one less than endpoint number */
    .ep_ctrl = &usb_dpram->ep_ctrl[0].in,
    .ep_buf_ctrl = &usb_dpram->ep_buf_ctrl[1].in,
    .dpram_start = usb_dpram->epx_data + 1 * 64,
}, * ep2_out = &(struct usb_endpoint_configuration) {
    .transfer_type_bits = USB_TRANSFER_TYPE_BULK,
    .ep_ctrl = &usb_dpram->ep_ctrl[1].out,
    .ep_buf_ctrl = &usb_dpram->ep_buf_ctrl[2].out,
    .dpram_start = usb_dpram->epx_data + 2 * 64,
}, * ep2_in = &(struct usb_endpoint_configuration) {
    .transfer_type_bits = USB_TRANSFER_TYPE_BULK,
    .ep_ctrl = &usb_dpram->ep_ctrl[1].in,
    .ep_buf_ctrl = &usb_dpram->ep_buf_ctrl[2].in,
    .dpram_start = usb_dpram->epx_data + 3 * 64,
};

static inline uint32_t usb_buffer_offset(volatile unsigned char * buf) {
    /* return offset within dpram of a given absolute pointer */
    return (uint32_t)buf ^ (uint32_t)usb_dpram;
}

void usb_cdc_serial_init(void) {
    /* reset peripheral */
    reset_unreset_block_num_wait_blocking(RESET_USBCTRL);

    /* clear dpram */
    memset(usb_dpram, 0, sizeof(*usb_dpram));

    /* enable usb interrupt in nvic */
    irq_set_enabled(USBCTRL_IRQ, true);

    /* enable usb pin mux */
    usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;

    /* force vbus detect */
    usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;

    /* enable in device mode */
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;

    /* one interrupt per EP0 transaction (single buffered) */
    usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS;

    /* enable interrupts we will react to */
    usb_hw->inte = USB_INTS_BUFF_STATUS_BITS | USB_INTS_BUS_RESET_BITS | USB_INTS_SETUP_REQ_BITS;

    *ep1_in->ep_ctrl = (EP_CTRL_ENABLE_BITS |
                        EP_CTRL_INTERRUPT_PER_BUFFER |
                        (USB_TRANSFER_TYPE_INTERRUPT << EP_CTRL_BUFFER_TYPE_LSB) |
                        usb_buffer_offset(ep1_in->dpram_start));

    *ep2_out->ep_ctrl = (EP_CTRL_ENABLE_BITS |
                         EP_CTRL_INTERRUPT_PER_BUFFER |
                         (USB_TRANSFER_TYPE_BULK << EP_CTRL_BUFFER_TYPE_LSB) |
                         usb_buffer_offset(ep2_out->dpram_start));

    /* TODO: why do we need this? */
    *ep2_in->ep_ctrl = (EP_CTRL_ENABLE_BITS |
                        EP_CTRL_INTERRUPT_PER_BUFFER |
                        (USB_TRANSFER_TYPE_BULK << EP_CTRL_BUFFER_TYPE_LSB) |
                        usb_buffer_offset(ep2_in->dpram_start));

    /* pull up on dp to indicate full speed */
    usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

static void usb_start_out_transfer(struct usb_endpoint_configuration * ep, const size_t len) {
    /* handle a single OUT packet at a time */
    assert(len <= 64);

    *ep->ep_buf_ctrl = (len | USB_BUF_CTRL_AVAIL |
                        (ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID));
    ep->next_pid ^= 1;
}

void unaligned_memcpy(void * dstv, const void * restrict srcv, size_t count) {
    unsigned char * restrict dst = dstv;
    const unsigned char * restrict src = srcv;

    while (count--)
        *dst++ = *src++;
}

static void usb_double_buffered_in_transfer_continue(struct usb_endpoint_configuration * ep) {
    const size_t len_remaining = ep->dpram_stop - ep->dpram_cursor;
    if (len_remaining > 64) {
        const size_t len_now = len_remaining < 128 ? len_remaining : 128;

        const uint32_t val_lo = (64 | USB_BUF_CTRL_FULL |
                                 (ep->dpram_cursor == ep->dpram_start ? USB_BUF_CTRL_SEL : 0) |
                                 (ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID));

        *ep->ep_ctrl = (EP_CTRL_ENABLE_BITS
                        | EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER
                        | (ep->transfer_type_bits << EP_CTRL_BUFFER_TYPE_LSB)
                        | EP_CTRL_DOUBLE_BUFFERED_BITS
                        | usb_buffer_offset(ep->dpram_cursor));
        ep->dpram_cursor += len_now;

        ep->next_pid ^= 1;

        const uint32_t val_hi = ((len_now - 64) | USB_BUF_CTRL_FULL |
                                 (ep->dpram_cursor == ep->dpram_stop ? USB_BUF_CTRL_LAST : 0) |
                                 (ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID));

        ep->next_pid ^= 1;

        *ep->ep_buf_ctrl = val_hi << 16 | val_lo;
        asm volatile("nop;nop;nop" :::);
        *ep->ep_buf_ctrl = ((val_hi | USB_BUF_CTRL_AVAIL) << 16) | (val_lo | USB_BUF_CTRL_AVAIL);

    } else {
        *ep->ep_ctrl = (EP_CTRL_ENABLE_BITS
                        | EP_CTRL_INTERRUPT_PER_BUFFER
                        | (ep->transfer_type_bits << EP_CTRL_BUFFER_TYPE_LSB)
                        | usb_buffer_offset(ep->dpram_cursor));

        const size_t len_now = ep->dpram_stop - ep->dpram_cursor;

        ep->dpram_cursor += len_now;

        const uint32_t val_lo = (len_now | USB_BUF_CTRL_FULL | USB_BUF_CTRL_LAST |
                                 (ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID));
        ep->next_pid ^= 1;

        *ep->ep_buf_ctrl = val_lo;
        asm volatile("nop;nop;nop" :::);
        *ep->ep_buf_ctrl = val_lo | USB_BUF_CTRL_AVAIL;
    }
}

static void usb_double_buffered_in_transfer_start(struct usb_endpoint_configuration * ep,
                                                  const size_t len_total) {

    ep->dpram_cursor = ep->dpram_start;
    ep->dpram_stop = ep->dpram_cursor + len_total;

    usb_double_buffered_in_transfer_continue(ep);
}

static void usb_single_buffered_in_transfer_continue(struct usb_endpoint_configuration * ep) {
    const size_t len_remaining = ep->in_stop - ep->in_cursor;
    const uint16_t len_now = len_remaining < 64 ? len_remaining : 64;

    const uint32_t val = (len_now | USB_BUF_CTRL_FULL |
                          (ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID));

    unaligned_memcpy(ep->dpram_start, ep->in_cursor, len_now);
    ep->in_cursor += len_now;

    ep->next_pid ^= 1;

    *ep->ep_buf_ctrl = val;
    asm volatile("nop;nop;nop" :::);
    *ep->ep_buf_ctrl = val | USB_BUF_CTRL_AVAIL;
}

static void usb_start_in_transfer(struct usb_endpoint_configuration *ep, const void * buf, const size_t len_total) {
    ep->in_cursor = buf;
    ep->in_stop = ep->in_cursor + len_total;

    usb_single_buffered_in_transfer_continue(ep);
}

static void usb_handle_device_descriptor(volatile struct usb_setup_packet *pkt) {
    const struct usb_device_descriptor device_descriptor __attribute((aligned(8))) = {
        .bLength = sizeof(struct usb_device_descriptor),
        .bDescriptorType = USB_DT_DEVICE,
        .bcdUSB = 0x200,
        .bDeviceClass = 0xEF, /* misc */
        .bDeviceSubClass = 2, /* common */
        .bDeviceProtocol = 1, /* iad */
        .bMaxPacketSize0 = 64,
        .idVendor = 0x0000,
        .idProduct = 0x0001,
        .bcdDevice = 0x100,
        .iManufacturer = 1,
        .iProduct = 2,
        .iSerialNumber = 0,
        .bNumConfigurations = 1
    };

    usb_start_in_transfer(ep0_in, &device_descriptor,
                          MIN(sizeof(struct usb_device_descriptor), pkt->wLength));
}

static void usb_handle_config_descriptor(volatile struct usb_setup_packet *pkt) {
    /* do a horrible thing to get both packed and aligned buffers */
    static const struct __attribute((aligned(8))) { struct __attribute((packed)) {
        struct usb_configuration_descriptor config;
        struct interface_association_descriptor iad;
        struct usb_interface_descriptor cif;
        struct cdc_header_functional_descriptor header;
        struct acm_functional_descriptor acm;
        struct union_functional_descriptor unionfd;
        struct call_management_functional_descriptor call_management;
        struct usb_endpoint_descriptor cifin;
        struct usb_interface_descriptor dif;
        struct usb_endpoint_descriptor in;
        struct usb_endpoint_descriptor out;
    } contents; } __attribute((packed)) config_descriptor_buffer = { .contents = {
        .config = {
            .bLength = sizeof(struct usb_configuration_descriptor),
            .bDescriptorType = USB_DT_CONFIG,
            .wTotalLength = sizeof(config_descriptor_buffer.contents),
            .bNumInterfaces = 2,
            .bConfigurationValue = 1,
            .iConfiguration = 0,
            .bmAttributes = USB_CONFIG_BUS_POWERED,
            .bMaxPower = 100 / 2
        },
        .iad = {
            .bLength = sizeof(struct interface_association_descriptor),
            .bDescriptorType = 11,
            .bFirstInterface = 0,
            .bInterfaceCount = 2,
            .bFunctionClass = 0x02,
            .bFunctionSubClass = 0x02,
            .bFunctionProtocol = 0,
            .iFunction = 0
        },
        .cif = {
            /* table 9-12 */
            .bLength = sizeof(struct usb_interface_descriptor),
            .bDescriptorType = USB_DT_INTERFACE,
            .bInterfaceNumber = CDC0_INTERFACE_ACM,
            .bAlternateSetting = 0,
            .bNumEndpoints = 1,
            .bInterfaceClass = 0x02, /* communication interface class */
            .bInterfaceSubClass = 0x02, /* abstract control model */
            .bInterfaceProtocol = 0, /* none */
            .iInterface = 0
        },
        .header = {
            .bFunctionLength = sizeof(struct cdc_header_functional_descriptor),
            .bDescriptorType = DESCRIPTOR_INTERFACE,
            .bDescriptorSubtype = 0x00, /* header functional descriptor from table 25 */
            .bcdCDC = 0x110, /* value for usbcdc11.pdf dated jan 19, 1999 */
        },
        .acm = {
            .bFunctionLength = sizeof(struct acm_functional_descriptor),
            .bDescriptorType = DESCRIPTOR_INTERFACE,
            .bDescriptorSubtype = 0x02, /* acm functional descriptor from table 25 */
            .bmCapabilities = 0b110 /* set line coding, set control line state, get line coding (todo) */
        },
        .unionfd = {
            .bFunctionLength = sizeof(struct union_functional_descriptor),
            .bDescriptorType = DESCRIPTOR_INTERFACE,
            .bDescriptorSubtype = 0x06, /* union functional descriptor from table 25 */
            .bControlInterface = CDC0_INTERFACE_ACM,
            .bSubordinateInterface = CDC0_INTERFACE_DATA
        },
        .call_management = {
            .bFunctionLength = sizeof(struct call_management_functional_descriptor),
            .bDescriptorType = DESCRIPTOR_INTERFACE,
            .bDescriptorSubtype = 0x01, /* call mgmt functional descriptor from table 25 */
            .bmCapabilities = 1, /* handle call */
            .bDataInterface = 1
        },
        .cifin = {
            .bLength = sizeof(struct usb_endpoint_descriptor),
            .bDescriptorType = USB_DESCRIPTOR_TYPE_ENDPOINT,
            .bEndpointAddress = 1 | 0x80,
            .bmAttributes = USB_TRANSFER_TYPE_INTERRUPT,
            .wMaxPacketSize = 16,
            .bInterval = 16
        },
        .dif = {
            .bLength = sizeof(struct usb_interface_descriptor),
            .bDescriptorType = USB_DT_INTERFACE,
            .bInterfaceNumber = CDC0_INTERFACE_DATA,
            .bAlternateSetting = 0,
            .bNumEndpoints = 2,
            .bInterfaceClass = 0x0A, /* data interface class (table 18) */
            .bInterfaceSubClass = 0,
            .bInterfaceProtocol = 0,
            .iInterface = 0
        },
        .in = {
            .bLength = sizeof(struct usb_endpoint_descriptor),
            .bDescriptorType = USB_DESCRIPTOR_TYPE_ENDPOINT,
            .bEndpointAddress = 2 | 0x00,
            .bmAttributes = USB_TRANSFER_TYPE_BULK,
            .wMaxPacketSize = 64,
            .bInterval = 0
        },
        .out = {
            .bLength = sizeof(struct usb_endpoint_descriptor),
            .bDescriptorType = USB_DESCRIPTOR_TYPE_ENDPOINT,
            .bEndpointAddress = 2 | 0x80,
            .bmAttributes = USB_TRANSFER_TYPE_BULK,
            .wMaxPacketSize = 64,
            .bInterval = 0
        }
    }};

    /* host retries this with the actual full length once it knows it */
    const size_t len = (pkt->wLength >= sizeof(config_descriptor_buffer.contents) ? sizeof(config_descriptor_buffer.contents) :
                        pkt->wLength >= sizeof(struct usb_configuration_descriptor) ? sizeof(struct usb_configuration_descriptor) :
                        pkt->wLength);

    usb_start_in_transfer(ep0_in, &config_descriptor_buffer, len);
}

static void usb_handle_string_descriptor(volatile struct usb_setup_packet *pkt) {
    /* zero if language request, nonzero if string descriptor index */
    unsigned istring = pkt->wValue & 0xff;
    size_t len = 0;

    static const char * strings[] = {
        "Raspberry Pi",
        "Pico Test Device"
    };

    if (!istring) {
        static const unsigned char lang_descriptor[] = {
            4, /* length */
            USB_DT_STRING,
            0x09, 0x04 /* english */
        };

        len = 4;
        unaligned_memcpy(ep0_buf, lang_descriptor, len);
    }
    else if (istring - 1 < sizeof(strings) / sizeof(strings[0])) {
        /* handle expansion of C string to utf-16 string descriptor thing */
        const char * string = strings[istring - 1];
        unsigned char * dest = ep0_buf;
        len = 2;
        for (; *string && len < sizeof(ep0_buf); len += 2) {
            dest[len + 0] = *string++;
            dest[len + 1] = 0;
        }

        dest[0] = len;
        dest[1] = USB_DT_STRING;
    }

    usb_start_in_transfer(ep0_in, ep0_buf, MIN(len, pkt->wLength));
}

static void usb_acknowledge_out_request(void) {
    usb_start_in_transfer(ep0_in, NULL, 0);
}

static unsigned sofnum_at_dtr_high = 0;
static unsigned hack_has_elapsed = 0;

static unsigned char should_set_cdc_line_state = 0;
static uint8_t cdc_line_state;

static unsigned char should_set_dev_addr = 0;
static uint8_t dev_addr = 0;

static struct cdc_line_info __attribute((aligned(8))) cdc_line_info = { .dwDTERate = 115200, .bDataBits = 8 };
_Static_assert(sizeof(cdc_line_info) == 7, "wtf");

char get_dtr(void) {
    if (!hack_has_elapsed) return 0;
    return (*(volatile uint8_t *)&cdc_line_state) & 0x1;
}

static void usb_handle_setup_packet(void) {
    volatile struct usb_setup_packet * pkt = (volatile struct usb_setup_packet *) &usb_dpram->setup_packet;
    const uint8_t req_direction = pkt->bmRequestType;
    const uint8_t req = pkt->bRequest;

    /* ep0 next pid should always be 1 in response to a setup packet */
    ep0_in->next_pid = 1;
    ep0_out->next_pid = 1;

    if (CDC_SET_CONTROL_LINE_STATE == req) {
        /* if the port is closed (DTR goes low...) */
        if ((cdc_line_state & 0x01) && !(pkt->wValue & 0x01)) {
            /* and the baud rate is 1200 bps... */
            if (cdc_line_info.dwDTERate == 1200)
                /* reset into bootloader */
                rom_reset_usb_boot_extra(-1, 0, false);

            hack_has_elapsed = 0;
        }

        else if (!(cdc_line_state & 0x01) && (pkt->wValue & 0x01)) {
            /* hack: we need to wait some time between seeing dtr go high and letting
             calling code know about it */
            sofnum_at_dtr_high = usb_hw->sof_rd;

            usb_hw_set->inte = USB_INTE_DEV_SOF_BITS;
        }

        cdc_line_state = pkt->wValue & 0xFF;
        usb_acknowledge_out_request();
    }
    else if (CDC_SET_LINE_CODING == req) {
        should_set_cdc_line_state = 1;
        usb_start_out_transfer(ep0_out, MIN(pkt->wLength, sizeof(cdc_line_info)));
    }
    else if (CDC_SEND_BREAK == req)
    /* TODO: */
        usb_acknowledge_out_request();
    else if (req_direction == USB_DIR_OUT) {
        if (req == USB_REQUEST_SET_ADDRESS) {
            /* send a zlp as address 0 first, then, when that is acknowledged, set the actual addr */
            dev_addr = (pkt->wValue & 0xff);
            should_set_dev_addr = 1;
            usb_acknowledge_out_request();
        }
        else if (req == USB_REQUEST_SET_CONFIGURATION) {
            usb_acknowledge_out_request();

            /* indicate to host that ep2 out can receive */
            usb_start_out_transfer(ep2_out, 64);

            enumerated = 1;
        }
        else usb_acknowledge_out_request();
    } else if (req_direction == USB_DIR_IN) {
        if (req == USB_REQUEST_GET_DESCRIPTOR) {
            const uint16_t descriptor_type = pkt->wValue >> 8;

            if (USB_DT_DEVICE == descriptor_type)
                usb_handle_device_descriptor(pkt);
            else if (USB_DT_CONFIG == descriptor_type)
                usb_handle_config_descriptor(pkt);
            else if (USB_DT_STRING == descriptor_type)
                usb_handle_string_descriptor(pkt);
            else {
                /* stall ep0 in, otherwise host thinks we actually want high speed */
                usb_hw_set->ep_stall_arm = USB_EP_STALL_ARM_EP0_IN_BITS;
                *ep0_in->ep_buf_ctrl = USB_BUF_CTRL_STALL;
            }
        }
    }
}

static size_t rx_buf_filled = 0;

size_t usb_cdc_serial_rx_filled(void) {
    return *(volatile size_t *)&rx_buf_filled;
}

void usb_cdc_serial_rx_rearm(void) {
    *(volatile size_t *)&rx_buf_filled = 0;
    /* indicate to host that ep2 out can receive */
    usb_start_out_transfer(ep2_out, 64);
}

int usb_cdc_serial_tx_still_sending(void) {
    return !!(*(void * volatile *)&ep2_in->dpram_stop);
}

void * usb_cdc_serial_tx_staging_area(void) {
    return ep2_in->dpram_start;
}

const void * usb_cdc_serial_rx_staging_area(void) {
    return ep2_out->dpram_start;
}

void usb_cdc_serial_tx_start(const size_t size) {
    usb_double_buffered_in_transfer_start(ep2_in, size);
}

static unsigned usb_get_endpoint_buff_status_mask(uint8_t addr) {
    return 1U << ((addr & ~USB_DIR_IN) * 2 + !(addr & USB_DIR_IN));
}

static void usb_handle_buff_status(void) {
    const uint32_t buffers = usb_hw->buf_status;
    uint32_t remaining_buffers = buffers;

    const unsigned ep0_in_mask = usb_get_endpoint_buff_status_mask(USB_DIR_IN | 0);
    if (remaining_buffers & ep0_in_mask) {
        usb_hw_clear->buf_status = ep0_in_mask;
        remaining_buffers &= ~ep0_in_mask;

        if (ep0_in->in_cursor != ep0_in->in_stop)
            usb_single_buffered_in_transfer_continue(ep0_in);
        else {
            if (should_set_dev_addr) {
                usb_hw->dev_addr_ctrl = dev_addr;
                should_set_dev_addr = 0;
            }
            else
            /* receive a zlp from host */
                usb_start_out_transfer(ep0_out, 0);

            ep0_in->in_cursor = NULL;
            ep0_in->in_stop = NULL;
        }
    }

    const unsigned ep0_out_mask = usb_get_endpoint_buff_status_mask(USB_DIR_OUT | 0);
    if (remaining_buffers & ep0_out_mask) {
        usb_hw_clear->buf_status = ep0_out_mask;
        remaining_buffers &= ~ep0_out_mask;

        if (should_set_cdc_line_state) {
            unaligned_memcpy(&cdc_line_info, usb_dpram->ep0_buf_a, sizeof(cdc_line_info));
            should_set_cdc_line_state = 0;

            usb_acknowledge_out_request();
        }
    }

    const unsigned ep2_in_mask = usb_get_endpoint_buff_status_mask(USB_DIR_IN | 2);
    if (remaining_buffers & ep2_in_mask) {
        usb_hw_clear->buf_status = ep2_in_mask;
        remaining_buffers &= ~ep2_in_mask;

        if (ep2_in->dpram_cursor != ep2_in->dpram_stop)
            usb_double_buffered_in_transfer_continue(ep2_in);
        else {
            ep2_in->dpram_cursor = NULL;
            ep2_in->dpram_stop = NULL;
        }
    }

    const unsigned ep2_out_mask = usb_get_endpoint_buff_status_mask(USB_DIR_OUT | 2);
    if (remaining_buffers & ep2_out_mask) {
        usb_hw_clear->buf_status = ep2_out_mask;
        remaining_buffers &= ~ep2_out_mask;

        rx_buf_filled = (*ep2_out->ep_buf_ctrl) & USB_BUF_CTRL_LEN_MASK;
    }

    /* TODO: something to handle remaining buffers? */
}

void isr_usbctrl(void) {
    const uint32_t status = usb_hw->ints;
    uint32_t handled = 0;

    if (status & USB_INTS_SETUP_REQ_BITS) {
        handled |= USB_INTS_SETUP_REQ_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
        usb_handle_setup_packet();
    }

    if (status & USB_INTS_BUFF_STATUS_BITS) {
        handled |= USB_INTS_BUFF_STATUS_BITS;
        usb_handle_buff_status();
    }

    if (status & USB_INTS_BUS_RESET_BITS) {
        handled |= USB_INTS_BUS_RESET_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;

        dev_addr = 0;
        usb_hw->dev_addr_ctrl = 0;
        enumerated = 0;
    }

    if (status & USB_INTS_DEV_SOF_BITS) {
        handled |= USB_INTS_DEV_SOF_BITS;
        const unsigned sofnum = usb_hw->sof_rd;
        if (sofnum - sofnum_at_dtr_high >= 200) {
            hack_has_elapsed = 1;

            usb_hw_clear->inte = USB_INTE_DEV_SOF_BITS;
        }
    }

    if (usb_hw->sie_status & USB_SIE_STATUS_DATA_SEQ_ERROR_BITS)
        usb_hw->sie_status = USB_SIE_STATUS_DATA_SEQ_ERROR_BITS;

    if (status ^ handled)
        panic("unhandled irq(s): 0x%x\n", (uint) (status ^ handled));
}
