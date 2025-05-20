cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(pico_can C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_can
    main.c
    mcp2515.c
)

target_link_libraries(pico_can pico_stdlib hardware_spi)

pico_add_extra_outputs(pico_can)