cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wearable_nrf)

set(CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../core)

target_sources(app PRIVATE
    src/main.c
    src/imu.c
    ${CORE_DIR}/crc16.c
    ${CORE_DIR}/packet.c
    ${CORE_DIR}/ringbuffer.c
    ${CORE_DIR}/builder.c
)

target_include_directories(app PRIVATE ${CORE_DIR})
