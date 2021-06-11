cmake_minimum_required(VERSION 3.13)

include(/usr/share/pico-sdk/external/pico_sdk_import.cmake)

project(mpu6050 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(main main.c mpu6050.c)
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
pico_add_extra_outputs(main)
target_link_libraries(main pico_stdlib hardware_i2c)

add_executable(raw raw.c mpu6050.c)
pico_enable_stdio_usb(raw 1)
pico_enable_stdio_uart(raw 0)
pico_add_extra_outputs(raw)
target_link_libraries(raw pico_stdlib hardware_i2c)

add_executable(calibrate calibrate.c mpu6050.c)
pico_enable_stdio_usb(calibrate 1)
pico_enable_stdio_uart(calibrate 0)
pico_add_extra_outputs(calibrate)
target_link_libraries(calibrate pico_stdlib hardware_i2c)
