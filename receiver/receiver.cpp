#include <stdio.h>
#include "pico/stdlib.h"
#include "UartDma.h"
#include "led.h"
#include "pico/sync.h"
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif
int main() {
    stdio_init_all();
    sleep_ms(1000);
	led_init();
    // UART0, TX=GPIO0(未使用), RX=GPIO1
    UartDma uart(uart0, 115200, 0, 1, 1024, 4096);
    uart.init();
	sleep_ms(1000);

    printf("=== PICO B UART DMA RX START ===\n");

	while (true) {
#if(0)
		if(uart.read_byte() < 0){
			__wfi();
		}else{
			int b;
			while((b=uart.read_byte())>=0){
				printf("%c ", (char)b);
			}
			led_sw();
		}
#else
			int b;
			while((b=uart.read_byte())>=0){
				printf("%c ", (char)b);
			}
			led_sw();
#endif
	}
}