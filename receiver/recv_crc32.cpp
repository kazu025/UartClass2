#include "pico/stdlib.h"
#include <stdio.h>
#include "UartDma.h"

static uint32_t crc32_table[256];
static bool crc32_init_done = false;
constexpr size_t BLOCK = 4096;
static uint8_t payload[BLOCK];
static uint8_t crc_buf[4];
static const int PRINT_INTERVAL = 30 * 1000; // ms
static void crc32_init() {
	for (uint32_t i = 0; i < 256; i++) {
		uint32_t crc = i;
		for (uint32_t j = 0; j < 8; j++)
			crc = crc & 1? 0xEDB88320 ^ (crc >> 1) : crc >> 1;
		crc32_table[i] = crc;
	}
	crc32_init_done = true;
}
static uint32_t crc32_calc(const uint8_t *data, size_t len) {
	if (!crc32_init_done) crc32_init();
	uint32_t crc = 0xFFFFFFFFu;
	for (size_t i = 0; i < len; i++) {
		uint32_t idx = (uint8_t)(crc ^ data[i]) & 0xFFu;
		crc = crc32_table[idx] ^ (crc >> 8);
	}
	return crc ^ 0xFFFFFFFFu;
}

bool read_exact(UartDma& ud, uint8_t* buf, size_t len) {
	size_t received = 0;
	while (received < len) {
		int n = ud.read_bytes(buf + received, len - received);
		if (n > 0) {
			received += n;
		}
	}
	return true;
}
int main() {
    stdio_init_all();
    sleep_ms(1000);
    UartDma uart(uart0, 115200, 0, 1, 4096, 4096); // TX=GP0, RX=GP1例
//    UartDma uart(uart0, 921600, 0, 1, 4096, 4096); // TX=GP0, RX=GP1例
    uart.init();
    sleep_ms(1000);

    absolute_time_t next_print = make_timeout_time_ms(PRINT_INTERVAL);
    uint64_t total = 0;
    uint8_t buf[256];
	uint32_t ok = 0, ng = 0;
    sleep_ms(1000);
	printf("=== UartDma RX + CRC32 Check Sample ===\n");
    while (true) {
		read_exact(uart, payload, BLOCK);
		read_exact(uart, crc_buf, 4);
		uint32_t received_crc =
			 (uint32_t)crc_buf[0] |
			((uint32_t)crc_buf[1] << 8) |
			((uint32_t)crc_buf[2] << 16) |
			((uint32_t)crc_buf[3] << 24);

		uint32_t calc_crc = crc32_calc(payload, BLOCK);
		total++;
		if (calc_crc == received_crc) {
			ok++;
		} else {
			printf("!!! total:%llu ok:%lu ng:%lu received_crc:%08x calc_crc:%08x\n",
				total, ok, ng, received_crc, calc_crc);
			ng++;
		}
        if (absolute_time_diff_us(get_absolute_time(), next_print) <= 0) {
            printf("T:%llu OK:%lu NG:%lu\n", total, ok, ng);
            next_print = make_timeout_time_ms(PRINT_INTERVAL);

		}
        tight_loop_contents();
    }
}
