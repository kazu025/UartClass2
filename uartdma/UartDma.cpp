#include "UartDma.h"
#include "hardware/sync.h"

UartDma* UartDma::instance_ = nullptr;

UartDma::UartDma(uart_inst_t* uart, uint32_t baudrate,
	uint tx_pin, uint rx_pin, size_t tx_buf_size, size_t rx_buf_size) :
	uart_(uart), baudrate_(baudrate), tx_pin_(tx_pin), rx_pin_(rx_pin),
	tx_size_(tx_buf_size), rx_size_(rx_buf_size),
	rx_read_pos_(0), tx_head_(0), tx_tail_(0), tx_dma_running_(false), drop_first_rx_byte_(true),
	inited_(false){
		rx_buf_ = new uint8_t[rx_buf_size];
		tx_buf_ = new uint8_t[tx_buf_size];
		tx_dma_active_count_ = 0;
}

void UartDma::init(){
	if(inited_) return;
	uart_init(uart_, baudrate_);
	gpio_set_function(tx_pin_, GPIO_FUNC_UART);
	gpio_set_function(rx_pin_, GPIO_FUNC_UART);

	uart_set_fifo_enabled(uart_, true); // FIFO ON

	// claim channels
	dma_rx_chan_ = dma_claim_unused_channel(true);
	dma_tx_chan_ = dma_claim_unused_channel(true);

	// RX
	dma_channel_config cr = dma_channel_get_default_config(dma_rx_chan_);
	channel_config_set_transfer_data_size(&cr, DMA_SIZE_8);
	channel_config_set_read_increment(&cr, false);
	channel_config_set_write_increment(&cr, true);
	channel_config_set_dreq(&cr, uart_get_dreq(uart_, false));
	// rx_size_ = 2^N (N=__builtin_ctz())
	channel_config_set_ring(&cr, true, __builtin_ctz(rx_size_)); // ring size = 2^N
	dma_channel_configure(dma_rx_chan_, &cr, 
		rx_buf_,
		(const void*)(UART_BASE + UART_UARTDR_OFFSET),
		rx_size_, true);
	// TX
	dma_channel_config ct = dma_channel_get_default_config(dma_tx_chan_);
	channel_config_set_transfer_data_size(&ct, DMA_SIZE_8);
	channel_config_set_read_increment(&ct, true);
	channel_config_set_write_increment(&ct, false);
	channel_config_set_dreq(&ct, uart_get_dreq(uart_, true));
	// configure TX channel but dont start(count = 0)
	dma_channel_configure(dma_tx_chan_, &ct, 
		(void*)(UART_BASE + UART_UARTDR_OFFSET),	// write addr (UART)
		tx_buf_,										// read addr (mem)
		0, false);
	
	/* --- IRQ registration depending on channel number --- */
	/* DMA channel 0~15 => IRQ0, 16~31 => IRQ1 */
	int irq_num = (dma_tx_chan_ < 16) ? DMA_IRQ_0 : DMA_IRQ_1;
	if(irq_num == DMA_IRQ_0){
		irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_trampoline);
		irq_set_enabled(DMA_IRQ_0, true);
		dma_channel_set_irq0_enabled(dma_tx_chan_, true);
	} else {
		irq_set_exclusive_handler(DMA_IRQ_1, dma_irq_trampoline);
		irq_set_enabled(DMA_IRQ_1, true);
		dma_channel_set_irq1_enabled(dma_tx_chan_, true);
	}
	instance_ = this;
	// enable UART DMA (both RX/TX)
	inited_ = true;
    // debug print: which channels we got (safe here)
    // printf("UartDma init: rx chan=%d tx chan=%d\n", dma_rx_chan_, dma_tx_chan_);
	uart_get_hw(uart_)->dmacr = (1u << UART_UARTDMACR_RXDMAE_LSB) | (1u << UART_UARTDMACR_TXDMAE_LSB);
}

int UartDma::read_byte(){
	// compute current DMA write position from hardware write_addr
	uintptr_t waddr = dma_hw->ch[dma_rx_chan_].write_addr;
	uint16_t pos = (waddr - (uintptr_t)rx_buf_) & (rx_size_ -1);
	if(pos == rx_read_pos_) return -1;

	uint8_t b = rx_buf_[rx_read_pos_];
	rx_read_pos_  = (rx_read_pos_ + 1) & (rx_size_ - 1);
	if(drop_first_rx_byte_){
		drop_first_rx_byte_ = false;
		return -1;
	}
	return b;
}

int UartDma::read_bytes(uint8_t* dst, int maxlen){
	int count =  0;
	while(count < maxlen){
		int b = read_byte();
		if(b < 0) break;
		dst[count++] = (uint8_t)b;
	}
	return count;
}

void UartDma::write_byte(uint8_t b){
	uint16_t save = save_and_disable_interrupts(); // critical section
	uint16_t next = (tx_head_ + 1) & (tx_size_ - 1);
	if(next == tx_tail_){
		restore_interrupts(save);
		return ;
	}
	tx_buf_[tx_head_] = b;
	tx_head_ = next;
	start_tx_dma_if_needed();
	restore_interrupts(save);
}

void UartDma::write_string(const char* s){
	while(*s) write_byte(*s++);
}

void UartDma::write_buffer(const uint8_t* data, size_t len){
	for(size_t i = 0; i<len; i++) write_byte(*data++);
}

void UartDma::start_tx_dma_if_needed(){
	/*	最初から割り込み禁止するのはムダが多い。
	    でも、割り込み禁止しないと race condition が発生する。
	    軽量チェック＋安全チェック の２段構えにしてある。*/
    if(tx_dma_running_) return;
    if(tx_tail_ == tx_head_) return;

    uint32_t now = tx_tail_;
    uint32_t count = (tx_head_ >= now) ? (tx_head_ - now) : (tx_size_ - now);
    if(count == 0) return;

	uint32_t irq_state = save_and_disable_interrupts();
	if(tx_dma_running_ || tx_tail_ == tx_head_){
		restore_interrupts(irq_state);
		return;
	}
	// Recompute now/count under lock because they may have changed
	now = tx_tail_;
	count = (tx_head_ >= now) ? (tx_head_ - now) : (tx_size_ - now);
    if(count == 0){
        restore_interrupts(irq_state);
        return;
    }
	// Wait briefly for UART FIFO to be writable so DMA's first beat will be accepted.
    // Do not block long inside interrupt-disabled region: perform a short busy wait.
    // We perform FIFO check outside long critical region when possible, but we do minimal check here.
    // (The serialize of DMA programming below is what's critical.)
    if(!uart_is_writable(uart_)){
        // if FIFO not writable, do a tiny spin (but don't hang)
        uint32_t t0 = time_us_32();
        while(!uart_is_writable(uart_)){
            if(time_us_32() - t0 > 1000) break; // 1 ms timeout
            tight_loop_contents();
        }
    }

	// store transfer length so IRQ knows how many bytes were scheduled
    tx_dma_active_count_ = count;

    dma_channel_set_read_addr(dma_tx_chan_, &tx_buf_[now], false);
    dma_channel_set_write_addr(dma_tx_chan_, (void*)(UART_BASE + UART_UARTDR_OFFSET), false);
    dma_channel_set_trans_count(dma_tx_chan_, count, true);

    tx_dma_running_ = true;
	restore_interrupts(irq_state);
}


void UartDma::dma_irq_handler(){
    // Clear the interrupt bit for the TX channel
	if(dma_tx_chan_ < 16) {
 	   dma_hw->ints0 = 1u << dma_tx_chan_;
	} else {
		dma_hw->ints1 = 1u << (dma_tx_chan_ - 16);
	}

 	// Sent bytes = value saved at start
 	uint32_t sent = tx_dma_active_count_;
	if(sent){
		tx_tail_ = (tx_tail_ + sent) & (tx_size_ - 1);
		tx_dma_active_count_ = 0;
	}
	if(tx_tail_ != tx_head_){
		tx_dma_running_ = false;
		// start next chunk (in ISR). start_tx_dma_if_needed will do very small critical
		start_tx_dma_if_needed();
	} else {
		tx_dma_running_ = false;
	}
}

void UartDma::dma_irq_trampoline(){
	if(instance_) instance_->dma_irq_handler();
}