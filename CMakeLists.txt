cmake_minimum_required(VERSION 3.13)

include(lib/platform.cmake)

# Stop Pico SDK from trying to fetch/build picotool on this machine
set(PICO_SDK_FETCH_PICOTOOL OFF)

project(nightowl C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(lib)

add_executable(nightowl-firmware nightowl-firmware.cpp
)

target_link_libraries(nightowl-firmware pico_stdlib hardware_adc)

pico_enable_stdio_usb(nightowl-firmware 1)
pico_enable_stdio_uart(nightowl-firmware 0)
