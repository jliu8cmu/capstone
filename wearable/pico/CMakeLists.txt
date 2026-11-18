cmake_minimum_required(VERSION 3.13)

# Pull in the Pico SDK (set PICO_SDK_PATH in your environment or on the cmake line)
include(pico_sdk_import.cmake)

project(wearable_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

# Shared, chip-independent code
set(CORE_DIR ${CMAKE_CURRENT_LIST_DIR}/../core)

add_executable(wearable_pico
    main.c
    imu.c
    flashlog.c
    ${CORE_DIR}/crc16.c
    ${CORE_DIR}/packet.c
    ${CORE_DIR}/ringbuffer.c
    ${CORE_DIR}/builder.c
)

target_include_directories(wearable_pico PRIVATE ${CORE_DIR})

target_compile_options(wearable_pico PRIVATE -Wall -Wextra)

target_link_libraries(wearable_pico
    pico_stdlib
    hardware_spi
    hardware_uart
    hardware_adc
    hardware_sync
    hardware_flash
)

# Debug printf over USB, not over a UART pin (UART1 is used for the DK link)
pico_enable_stdio_usb(wearable_pico 1)
pico_enable_stdio_uart(wearable_pico 0)

# Produce wearable_pico.uf2 (plus .elf/.bin/.hex)
pico_add_extra_outputs(wearable_pico)