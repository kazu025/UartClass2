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
    UartDma uart(uart0, 115200, 0, 1, 256, 256);
    uart.init();
	led_init();
	sleep_ms(1000);
    uint8_t seq = 0;
	printf("=== PICO A Uart Dma Tx Start\r\n");
    while (true) {
        uart.write_byte((uint8_t)seq);
        seq++;
		sleep_ms(50);
		printf("%d\r\n",(uint8_t)seq);
		led_sw();
    }
	while(true) tight_loop_contents();
}
