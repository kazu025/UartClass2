#include <cstdio>
#include <cstring>
#include "pico/stdlib.h"
#include "UartDma.h"
#include "led.h"
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif
// UART設定（必要に応じて変更）
static uart_inst_t* const UART_ID = uart0;   // constexpr は不可
static constexpr uint32_t BAUD = 115200;
static constexpr uint TX_PIN = 0;            // uart0 TX = GPIO0
static constexpr uint RX_PIN = 1;            // uart0 RX = GPIO1（今回は使わなくてもOK）

// UartDma が 2^N を要求するなら 2の累乗にする
static constexpr size_t TX_BUF_SIZE = 1024;
static constexpr size_t RX_BUF_SIZE = 2048;

// 文字列送信ヘルパ（UartDma::write_buffer を使う）
static inline void uartdma_write_str(UartDma& ud, const char* s) {
    ud.write_buffer(reinterpret_cast<const uint8_t*>(s), strlen(s));
}

int main() {
    stdio_init_all();
    sleep_ms(1500); // USBシリアル接続待ち（任意）
    led_init();
    printf("\n=== UartDma TX Sample ===\n");
    printf("UART=%s BAUD=%u TX=%u RX=%u\n",
           (UART_ID == uart0) ? "uart0" : "uart1",
           BAUD, TX_PIN, RX_PIN);
    printf("Open a serial terminal on PC for UART RX (e.g. /dev/ttyUSB* or ttyACM*, 115200bps)\n");
	UartDma ud(UART_ID, BAUD, TX_PIN, RX_PIN, TX_BUF_SIZE, RX_BUF_SIZE);
    ud.init();

    sleep_ms(100); // 安定待ち（任意）
    uint32_t counter = 0;

    while (true) {
        // 1) 定期的にテキストを送る（動作確認しやすい）
#if (0)
		char line[128];
        snprintf(line, sizeof(line), "Hello from Pico (UartDma TX)! count=%lu\r\n",
                 (unsigned long)counter++);
        uartdma_write_str(ud, line);
#else
        // 2) 連番バイトを送る（スループット/取りこぼし確認用）
        //    PC側で 00 01 02 ... が続くかを見る
		uint8_t seq[256];
		uint8_t v=0;
        for (size_t i = 0; i < sizeof(seq); i++, v++) seq[i] = v;
        ud.write_buffer(seq, sizeof(seq));
		for(int i=0; i<256; i++){
			printf("%d\n", seq[i]);
		}
        sleep_ms(100);
		led_sw();
#endif
		tight_loop_contents();
	}
}
