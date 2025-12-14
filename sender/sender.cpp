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
    UartDma uart(uart0, 921600, 0, 1, 256, 256);
    uart.init();
	led_init();
	sleep_ms(1000);
    uint8_t seq = 0;
	int cnt = 0;
	printf("=== PICO A Uart Dma Tx Start\r\n");
    while (cnt < 100) {
        uart.write_byte(seq);
        seq++;
		printf("%02x\r\n", seq);
		led_sw();
		cnt ++;
    }
	printf("\r\ntest end!\r\n");
	while(true) tight_loop_contents();
}
