#pragma once
#include <stdint.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#define UART_BASE UART0_BASE
#define UART_ID uart0
#define UART_TX_PIN	0
#define UART_RX_PIN 1
#define BAUDRATE 115200
/*
 * UartDma
 * RP2040 向け UART DMA 安全ラッパ
 *
 * 特徴：
 *  - RX DMA はリングバッファ方式（常時循環）
 *  - TX DMA はリングバッファ方式（必要時のみ自動起動）
 *  - 最初の0xFF（UART線がHigh → RX FIFO初期値）を自動破棄
 *  - ISRは static trampoline → instance メソッド方式で安全
 *  - 複数UARTに対応（インスタンスごとに完全に分離）
 */
class UartDma{
public:
	UartDma(uart_inst_t *uart, uint32_t baudrate, uint tx_pin, uint rx_pin,
		size_t tx_buf_size=256, size_t rx_buf_size=256);
	// === Uart + DMA初期化 ===
	void init();
	// ====================================
	// 	RX
	// ====================================
	/* 1byte取得 データなし:-1 */
	int read_byte();
	/*まとめ読み (最大maxlen) 戻り値：読み取ったバイト数*/
	int read_bytes(uint8_t* dst, int maxlen);
	// ====================================
	// 	TX
	// ====================================
	/* 1バイト送信(リングバッファへ積む) */
	void write_byte(uint8_t b);
	/* 文字列送信(NULL終端) */
	void write_string(const char *s);
	/* まとめ送信 */
	void write_buffer(const uint8_t* data, size_t len);
private:
	// 内部状態
	uart_inst_t* uart_;
	uint32_t baudrate_;
	uint tx_pin_;
	uint rx_pin_;
	
	uint8_t* rx_buf_;
	uint8_t* tx_buf_;
	size_t rx_size_;
	size_t tx_size_;
	
	int dma_rx_chan_;
	int dma_tx_chan_;
	
	volatile int16_t rx_read_pos_;
	volatile uint16_t tx_head_;
	volatile uint16_t tx_tail_;

	volatile bool tx_dma_running_;
	volatile bool drop_first_rx_byte_;
	
	bool inited_;
	volatile uint32_t tx_dma_active_count_;
private:
	// TX DMA 起動(必要なら自動実行)
	void start_tx_dma_if_needed();
	// DMA IRQハンドラ
	void dma_irq_handler();
	// グローバルIRQ から呼ばれるtrampoline
	static void dma_irq_trampoline();	// static -> ISR
	// インスタンスをIRQとむずびつけるため保持
	static UartDma* instance_;
};