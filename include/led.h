#include "pico/stdlib.h"
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 500
#endif

/* --- LED 初期化 --- */
inline int led_init(void){
#if defined(PICO_DEFAULT_LED_PIN)
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
	return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
if(cyw43_arch_init()){
		printf("cyw_43_init() error!\n");
		return -1;
	}
	return 0;
#endif
}
/* toggle on/off */
inline void led_sw(void){
	static bool flag = true;
#if defined(PICO_DEFAULT_LED_PIN)
    gpio_put(PICO_DEFAULT_LED_PIN, flag);
#elif defined(CYW43_WL_GPIO_LED_PIN)
	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, flag);
#endif
	flag = !flag;
}
/* led on/off */
inline void led_onoff(bool flag){
#if defined(PICO_DEFAULT_LED_PIN)
    gpio_put(PICO_DEFAULT_LED_PIN, flag);
#elif defined(CYW43_WL_GPIO_LED_PIN)
	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, flag);
#endif
}