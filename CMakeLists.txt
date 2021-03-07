cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(piface_control_display_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_executable(piface_control_display
  piface_control_display.c
)

pico_enable_stdio_usb(piface_control_display 1)
pico_enable_stdio_uart(piface_control_display 1) 
pico_add_extra_outputs(piface_control_display)
target_link_libraries(piface_control_display pico_stdlib hardware_spi)