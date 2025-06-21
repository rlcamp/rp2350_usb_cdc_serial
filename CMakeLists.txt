# invoke using: mkdir -p build && cd build && cmake .. -DPICO_BOARD=pico2 && cd ..

cmake_minimum_required(VERSION 3.13)

if (NOT DEFINED ENV{PICO_SDK_PATH})
  set(ENV{PICO_SDK_PATH} "~/Downloads/pico-sdk/")
endif()

if (NOT DEFINED PICO_TOOLCHAIN_PATH)
    execute_process(
        COMMAND bash -c "dirname $((find $\{HOME\}/Downloads -name arm-none-eabi-gcc | sort; find /Applications/ArmGNUToolchain -name arm-none-eabi-gcc | sort) | tail -n1)"
        OUTPUT_VARIABLE PICO_TOOLCHAIN_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif()

# initialize the SDK directly
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# add some warning flags
set("CMAKE_C_FLAGS" "${CMAKE_C_FLAGS}  -Wall -Wextra -Wshadow")

add_executable(rp2350_usb_cdc_serial
    rp2350_usb_cdc_serial.c
    main.c
)

target_link_libraries(rp2350_usb_cdc_serial PRIVATE pico_stdlib hardware_resets hardware_irq cmsis_core)
pico_add_extra_outputs(rp2350_usb_cdc_serial)
