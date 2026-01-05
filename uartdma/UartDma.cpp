#include "UartDma.h"
#include "hardware/sync.h"
#include <cstdio>
#include <cstdlib>

UartDma* UartDma::instance_by_dma_chan_[32]= { nullptr};
bool UartDma::dma_irq_installed_ = false;
UartDma::UartDma(uart_inst_t* uart, uint32_t baudrate,
	uint tx_pin, uint rx_pin, size_t tx_buf_size, size_t rx_buf_size) :
	uart_(uart), baudrate_(baudrate), tx_pin_(tx_pin), rx_pin_(rx_pin),
	tx_size_(tx_buf_size), rx_size_(rx_buf_size),
	rx_read_pos_(0), tx_head_(0), tx_tail_(0), tx_dma_running_(false), drop_first_rx_byte_(true),
	inited_(false), tx_dma_active_count_(0)
{
	rx_buf_ = static_cast<uint8_t*>(aligned_alloc_for_dma(rx_size_, rx_size_));
	tx_buf_ = static_cast<uint8_t*>(aligned_alloc_for_dma(tx_size_, tx_size_));
	printf("rx_buf_=%p aligned=%u mod=%u\n", rx_buf_,
		reinterpret_cast<uintptr_t>(rx_buf_) ,
		reinterpret_cast<uintptr_t>(rx_buf_) % rx_size_);

}
UartDma::~UartDma(){
	if(inited_){
		dma_channel_abort(dma_rx_chan_);
		dma_channel_abort(dma_tx_chan_);
		dma_channel_unclaim(dma_rx_chan_);
		dma_channel_unclaim(dma_tx_chan_);
		aligned_free_for_dma(rx_buf_);
		aligned_free_for_dma(tx_buf_);
		inited_ = false;
	}
}
void UartDma::init(){
	if(inited_) return;
	uart_init(uart_, baudrate_);
	gpio_set_function(tx_pin_, GPIO_FUNC_UART);
	gpio_set_function(rx_pin_, GPIO_FUNC_UART);

	uart_set_fifo_enabled(uart_, true); // FIFO ON

	assert(is_power_of_two(tx_size_));
	assert(is_power_of_two(rx_size_));
	if(rx_size_ < 2 || rx_size_ > 32768 || !is_power_of_two(rx_size_) || !is_power_of_two(tx_size_)){
		printf("Error: tx_size_ and rx_size_ must be power of two\n");
		printf("Error: rx_size_ must be 2 to 32768\n");
		return;
	}
	// claim channels
	dma_rx_chan_ = dma_claim_unused_channel(true);
	dma_tx_chan_ = dma_claim_unused_channel(true);

	auto *hw = uart_get_hw(uart_);
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
		&hw->dr,
		0xffffffffu,
		/*rx_size_*/
		true); // 
	// TX
	dma_channel_config ct = dma_channel_get_default_config(dma_tx_chan_);
	channel_config_set_transfer_data_size(&ct, DMA_SIZE_8);
	channel_config_set_read_increment(&ct, true);
	channel_config_set_write_increment(&ct, false);
	channel_config_set_dreq(&ct, uart_get_dreq(uart_, true));
	// configure TX channel but dont start(count = 0)
	dma_channel_configure(dma_tx_chan_, &ct, 
		&hw->dr,		// write addr (UART)
		tx_buf_,		// read addr (mem)
		0, false);
	
	if(!dma_irq_installed_){
		irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_trampoline);
		irq_set_exclusive_handler(DMA_IRQ_1, dma_irq_trampoline);
		irq_set_enabled(DMA_IRQ_0, true);
		irq_set_enabled(DMA_IRQ_1, true);
		dma_irq_installed_ = true;
	}
	// RXの割り込みは使わない
	auto enable_dma_irq_for_ch = [](int ch){
		if(ch < 16) dma_channel_set_irq0_enabled(ch, true);
		else        dma_channel_set_irq1_enabled(ch, true);
	};
	enable_dma_irq_for_ch(dma_tx_chan_);

	instance_by_dma_chan_[dma_tx_chan_] = this;
	inited_ = true;
	
	uart_get_hw(uart_)->dmacr = (1u << UART_UARTDMACR_RXDMAE_LSB) | (1u << UART_UARTDMACR_TXDMAE_LSB);
	tx_full_hit_count_ = 0;
	TX_DMA_CHUNK_MAX = 1024; // 1回のDMA転送最大値
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
		if(b == 0x00 || b == 0xFF) {
			printf("Dropped initial 0xFF or 0x00 byte\n");
			return -1; /* 最初のゴミを捨てる */
		}
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

bool UartDma::write_byte(uint8_t b){
	uint32_t save = save_and_disable_interrupts(); // critical section
	uint16_t next = (tx_head_ + 1) & (tx_size_ - 1);
	if(next == tx_tail_){
		tx_full_hit_count_++;
//		printf("TX buffer full, dropping byte[%d]\n", tx_full_hit_count_);
		restore_interrupts(save);
		return false;
	}
	tx_buf_[tx_head_] = b;
	tx_head_ = next;
	start_tx_dma_locked();
	restore_interrupts(save);
	//start_tx_dma_if_needed();
	return true;
}
void UartDma::write_byte_blocking(uint8_t b){
	while(!write_byte(b)){
		tight_loop_contents();
	}
}
void UartDma::write_buffer_blocking(const uint8_t* data, size_t len){
	for(size_t i = 0; i<len; i++){
		write_byte_blocking(*data++);
	}
}

void UartDma::write_string(const char* s){
	while(*s) write_byte(*s++);
}

void UartDma::write_buffer(const uint8_t* data, size_t len){
	for(size_t i = 0; i<len; i++) write_byte(*data++);
}
bool UartDma::write_frame_blocking(const uint8_t* frame, size_t len){
	if(len == 0) return false;
	if(len > tx_size_ -1) return false;	// frame too large
	while(true){
		uint32_t save = save_and_disable_interrupts(); // critical section
		uint16_t free = (tx_tail_ - tx_head_ - 1) & (tx_size_ - 1);
		restore_interrupts(save);
		if(free >= len) break;
		tight_loop_contents();
	}
	// 書き込み
	write_buffer_blocking(frame, len);
	return true;
}
/**/
void UartDma::start_tx_dma_locked(){
	if(tx_dma_running_ || tx_tail_ == tx_head_){
		return;
	}
	uint16_t head = tx_head_, tail = tx_tail_;
	uint32_t count = (head >= tail) ? (head - tail) : (tx_size_ - tail);
	tx_dma_running_ = true;
	if(count > TX_DMA_CHUNK_MAX) count = TX_DMA_CHUNK_MAX;	// チャンク制限
	tx_dma_active_count_ = count;
	auto *hw = uart_get_hw(uart_);
	dma_channel_set_read_addr(dma_tx_chan_, &tx_buf_[tail], false);
	dma_channel_set_write_addr(dma_tx_chan_, &hw->dr, false);
	dma_channel_set_trans_count(dma_tx_chan_, count, true);	
}	
/* ロック有 通常用*/
void UartDma::start_tx_dma_if_needed(){
	uint32_t irq_state = save_and_disable_interrupts();

    if(tx_dma_running_ || tx_tail_ == tx_head_){
		restore_interrupts(irq_state);
		return;
	}

    uint32_t now = tx_tail_;
    uint32_t count = (tx_head_ >= now) ? (tx_head_ - now) : (tx_size_ - now);

    if(count == 0){
        restore_interrupts(irq_state);
        return;
    }
	if(count > TX_DMA_CHUNK_MAX) count = TX_DMA_CHUNK_MAX;	// チャンク制限
	tx_dma_active_count_ = count;

	auto *hw = uart_get_hw(uart_);
    dma_channel_set_read_addr(dma_tx_chan_, &tx_buf_[now], false);
    dma_channel_set_write_addr(dma_tx_chan_, &hw->dr, false);
	dma_channel_set_trans_count(dma_tx_chan_, count, true);

    tx_dma_running_ = true;
	restore_interrupts(irq_state);
}

/* ロック無し ISR用*/
void UartDma::start_tx_dma_if_needed_isr() {
    if (tx_dma_running_ || tx_tail_ == tx_head_) return;
	
    uint32_t now = tx_tail_;
    uint32_t count = (tx_head_ >= now) ? (tx_head_ - now) : (tx_size_ - now);
    if (count == 0) return;
	if (count > TX_DMA_CHUNK_MAX) count = TX_DMA_CHUNK_MAX;	// チャンク制限

    tx_dma_active_count_ = count;
    auto *hw = uart_get_hw(uart_);
    dma_channel_set_read_addr(dma_tx_chan_, &tx_buf_[now], false);
    dma_channel_set_write_addr(dma_tx_chan_, &hw->dr, false);
	tx_dma_running_ = true;
    dma_channel_set_trans_count(dma_tx_chan_, count, true);
}

void UartDma::dma_irq_handler(){
 	// Sent bytes = value saved at start
 	uint32_t sent = tx_dma_active_count_;
	if(sent){
		tx_tail_ = (tx_tail_ + sent) & (tx_size_ - 1);
		tx_dma_active_count_ = 0;
	}
	tx_dma_running_ = false;
	start_tx_dma_if_needed_isr();
}

void UartDma::dma_irq_trampoline(){
	// IRQ0(0-15)
	uint32_t ints0 = dma_hw->ints0;
	while(ints0){
		int ch = __builtin_ctz(ints0);
		dma_hw->ints0 = 1u << ch;
		if(instance_by_dma_chan_[ch]){
			instance_by_dma_chan_[ch]->dma_irq_handler();
		}
		ints0 &= ints0 - 1;
	}
	// IRQ1(16-31)
	uint32_t ints1 = dma_hw->ints1;
	while(ints1){
		int ch = __builtin_ctz(ints1) + 16;
		dma_hw->ints1 = 1u << (ch - 16);
		if(instance_by_dma_chan_[ch]){
			instance_by_dma_chan_[ch]->dma_irq_handler();
		}
		ints1 &= ints1 - 1;
	}
}

bool UartDma::is_power_of_two(size_t x){
	return x && ((x & (x-1)) == 0);
}
void * UartDma::aligned_alloc_for_dma(size_t size, size_t alignment){
	void* raw = malloc(size + alignment - 1 + sizeof(void*));
	if (!raw) return nullptr;
	uintptr_t raw_addr = reinterpret_cast<uintptr_t>(raw) + sizeof(void*);
	uintptr_t aligned = (raw_addr + (alignment - 1)) & ~(alignment - 1);
	void** store = reinterpret_cast<void**>(aligned - sizeof(void*));
	*store = raw;
	return reinterpret_cast<void*>(aligned);
}

void UartDma::aligned_free_for_dma(void* ptr){
	if (!ptr) return;
	void** store = reinterpret_cast<void**>(reinterpret_cast<uintptr_t>(ptr) - sizeof(void*));
	void* raw = *store;
	free(raw);
}
uint32_t UartDma::get_tx_full_hit_count() const {
	return tx_full_hit_count_;
}