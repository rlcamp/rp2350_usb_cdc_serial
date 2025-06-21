# Minimum viable RP2350 USB CDC ACM Serial implementation

This code is loosely derived from [https://github.com/raspberrypi/pico-examples/tree/master/usb/device/dev_lowlevel/], and adds double buffering for IN transactions and the necessary USB descriptors to enable proper USB CDC ACM functionality.
