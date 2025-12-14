#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define UART_ID uart0
#define BAUDRATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

int main() {
    stdio_init_all();

    uart_init(UART_ID, BAUDRATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
	sleep_ms(1000);
    printf("=== UART RX POLLING START ===\n");
	int cnt  = 0;
	bool flag = true;
    while (1) {
        if (uart_is_readable(UART_ID)) {
            uint8_t c = uart_getc(UART_ID);
			if(flag) {flag = false; continue;}
			cnt ++;
			//printf("%d\r\n", cnt);
			if(cnt == 100) break;
        }
    }
	printf("\r\ntest end!\r\n");
	while(1) tight_loop_contents();
}

#if(0)
#include <stdio.h>
#include "pico/stdlib.h"
#include "UartDma.h"
#include "led.h"

#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif
int main() {
    stdio_init_all();
    sleep_ms(1000);
	led_init();
    // UART0, TX=GPIO0(未使用), RX=GPIO1
    UartDma uart(uart0, 921600, 0, 1, 1024, 4096);
    uart.init();
	sleep_ms(1000);

    printf("=== PICO B UART DMA RX START ===\n");
	uint32_t prev = 0;
	bool first = true;
	uint8_t buf[4];

	while (true) {
		int n = uart.read_byte();
		if (n < 0) {
			tight_loop_contents();
			continue;
    	}
		led_sw();
		uint8_t a = (uint8_t)(n & 0x00ff);
		if (!first) {
			if (a != prev) {
			printf("MISS: prev=%u now=%u diff=%u\n",
                   prev, a, a - prev);
        	}
    	} else {
    	    first = false;
    	}
	    prev = a + 1;
	}
}
#endif