#include "pico/stdlib.h"
#include <stdio.h>
#include "UartDma.h"

int main() {
    stdio_init_all();
    sleep_ms(1000);

//    UartDma uart(uart0, 115200, 0, 1, 1024, 1024); // TX=GP0, RX=GP1例
    UartDma uart(uart0, 921600, 0, 1, 1024, 1024); // TX=GP0, RX=GP1例
    uart.init();

    absolute_time_t t0 = get_absolute_time();
    uint64_t total = 0;

    uint8_t buf[256];
    while (true) {
        int n = uart.read_bytes(buf, sizeof(buf));
        if (n > 0) {
            total += n;
            // エコー（PC側で目視できる）
            uart.write_buffer(buf, n);
        }

        if (absolute_time_diff_us(t0, get_absolute_time()) > 1000000) {
            printf("RX total=%llu\n", total);
            t0 = get_absolute_time();
        }
        tight_loop_contents();
    }
}
