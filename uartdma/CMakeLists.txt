add_library(uartdma
	UartDma.cpp
)

target_include_directories(uartdma
	PUBLIC
	${CMAKE_CURRENT_LIST_DIR}	# CMAKE_CURRENT_LIST_DIR:CmakelistsのおかれているDIR
)

target_link_libraries(uartdma
pico_stdlib
hardware_uart
hardware_dma
)
