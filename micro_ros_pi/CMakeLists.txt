cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_control C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(libmicroros)

add_executable(pico_control
    pico_control.cpp
    pico_uart_transport.c
)
target_link_libraries(pico_control
    pico_stdlib
    microros
    hardware_pio
    quadrature_encoder_pico
    motor_pico
    pid_pico
)

target_include_directories(pico_control PUBLIC
    libmicroros/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(pico_control 1)
pico_enable_stdio_uart(pico_control 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

add_subdirectory(pico_quadrature_encoder)
add_subdirectory(motor)
add_subdirectory(pid_controller)

# Generate UF2
pico_add_extra_outputs(pico_control)
