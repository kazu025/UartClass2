cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/pico_sdk_import.cmake)
project(uart_test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# UartDmaモジュール追加
add_subdirectory(uartdma)

# === send バイナリ ===
add_executable(send 
sender/send_sequence.cpp
)

# set_target_properties(send PROPERTIES PICO_BOARD pico)
#target_compile_definitions(recv PRIVATE PICO_W=0)

target_include_directories(send PRIVATE include)

target_link_libraries(send 
	pico_stdlib
	hardware_uart
	hardware_dma
	uartdma
)

pico_enable_stdio_usb(send 1)
pico_enable_stdio_uart(send 0)

pico_add_extra_outputs(send)

# === recv バイナリ ===
add_executable(recv 
receiver/recv_sequencecheck.cpp
)

#set_target_properties(recv PROPERTIES PICO_BOARD pico_w)
#target_compile_definitions(recv PRIVATE PICO_W=1)

target_include_directories(recv PRIVATE include)

target_link_libraries(recv 
	pico_stdlib
	hardware_uart
	hardware_dma
	pico_cyw43_arch_none
	uartdma
)

pico_enable_stdio_usb(recv 1)
pico_enable_stdio_uart(recv 0)


pico_add_extra_outputs(recv)

