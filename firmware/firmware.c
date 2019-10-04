#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include "serial.h"

void setup(serial_t *serial) {
	// GPIO-C
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC,
	              GPIO_MODE_OUTPUT_2_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL,
	              GPIO13);

  // Serial
  serial_configure(serial, USART1, 384000, 0);
}

int main(void) {
  serial_t serial;
  setup(&serial);

	while (1) {
		gpio_set(GPIOC, GPIO13);
		for (int i = 0; i < 1000000; ++i) __asm__("nop");
		gpio_clear(GPIOC, GPIO13);
		for (int i = 0; i <  500000; ++i) __asm__("nop");
	}
}
