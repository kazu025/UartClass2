#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include "UartDma.h"

static constexpr uint32_t PRINT_INTERVAL_MS = 30 * 1000; 
static uint32_t crc32_table[256];
static bool crc32_init_done = false;


static void crc32_init() {
	for (uint32_t i = 0; i < 256; i++) {
		uint32_t c = i;
		for (int k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
		crc32_table[i] = c;
	}
	crc32_init_done = true;
}

static inline uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
	for (size_t i = 0; i < len; i++) {
		uint8_t idx = (uint8_t)((crc ^ data[i]) & 0xFFu);
		crc = crc32_table[idx] ^ (crc >> 8);
	}
	return crc;
}

static inline uint32_t xorshift32(uint32_t& x) {
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return x;
}

int main(){
	stdio_init_all();
	sleep_ms(1000);
	if(!crc32_init_done) crc32_init();
	//uart0: TX=GPIO0, RX=GPIO1
//	static uint32_t const UART_BAUD = 921600;
	static uint32_t const UART_BAUD = 460800;
	//static uint32_t const UART_BAUD = 115200;
//	static uint32_t const UART_BAUD = 115200*4;
	UartDma uart(uart0, UART_BAUD, 0, 1, 16384, 1024); // 送信バッファ大きめ
	uart.init();

	constexpr uint32_t MAGIC = 0xA55A5AA5u;
	constexpr size_t BLOCK = 256;
	constexpr uint32_t N = 1'000'000;
	//constexpr size_t BLOCK = 4096;
	//constexpr uint32_t N = 60000;
	//constexpr uint32_t N = 480000;
	uint8_t payload[BLOCK];
	uint8_t header[8];
	uint8_t crc4[4];

	sleep_ms(1000);

	printf("=== TX SEQ+CRC32 baud: %lu N: %lu payload size: %lu ===\n", UART_BAUD, N, BLOCK);
//	constexpr uint32_t N = 2000;

	absolute_time_t next_print = make_timeout_time_ms(PRINT_INTERVAL_MS);
	
	for(uint32_t seq = 0; seq < N; seq++){
		// SEQ 作成
		header[0] = (uint8_t)(MAGIC & 0xFF);
		header[1] = (uint8_t)((MAGIC >> 8) & 0xFF);
		header[2] = (uint8_t)((MAGIC >> 16) & 0xFF);
		header[3] = (uint8_t)((MAGIC >> 24) & 0xFF);
		header[4] = (uint8_t)(seq & 0xFF);
		header[5] = (uint8_t)((seq >> 8) & 0xFF);
		header[6] = (uint8_t)((seq >> 16) & 0xFF);
		header[7] = (uint8_t)((seq >> 24) & 0xFF);
		// payload 生成(決定論的：seqから作る)
		uint32_t r = 0x12345678u ^ seq;
		for(size_t i=0; i<BLOCK; i++){
			payload[i] = (uint8_t)(i & 0xFFu);
			//payload[i] = (uint8_t)(xorshift32(r) & 0xFFu);
		}
		
		// CRC32 計算(header + payload)
		uint32_t crc = 0xFFFFFFFFu;
		crc = crc32_update(crc, header, sizeof(header));
		crc = crc32_update(crc, payload, BLOCK);
		crc ^= 0xFFFFFFFFu;
		crc4[0] = (uint8_t)(crc & 0xFF);
		crc4[1] = (uint8_t)((crc >> 8) & 0xFF);
		crc4[2] = (uint8_t)((crc >> 16) & 0xFF);
		crc4[3] = (uint8_t)((crc >> 24) & 0xFF);
		// 送信
		uart.write_buffer_blocking(header, sizeof(header));
		uart.write_buffer_blocking(payload, BLOCK);
		uart.write_buffer_blocking(crc4, sizeof(crc4));

		
		// 定期表示
		if(absolute_time_diff_us(get_absolute_time(), next_print) < 0){
			printf("Tx seq: %lu/%lu\n", seq, N);
			next_print = make_timeout_time_ms(PRINT_INTERVAL_MS);	
		}
	}
	// 最後の送信待ち（UARTからできるまで待つ flush APIがない前提の安全策）
	sleep_ms(2000);
	printf("=== TX SEQ+CRC32 send done No.:%lu drop:%lu===\n", N, uart.get_tx_full_hit_count());
	while(1) sleep_ms(1000);
	return 0;
}