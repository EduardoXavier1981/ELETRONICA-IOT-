cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(motor C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

#Caso seja a Raspberry Pi Pico W
set(PICO_BOARD pico_w)

pico_sdk_init()

add_executable(motor main.c)

pico_enable_stdio_usb(motor 1)
pico_enable_stdio_uart(motor 0)

target_include_directories(motor PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(motor
        tinyusb_board
        hardware_pwm
        pico_stdlib
    )
pico_add_extra_outputs(motor)
