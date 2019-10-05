#include "serial.h"

void setup(serial_t *serial) {
	// GPIO-C
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC,
	              GPIO_MODE_OUTPUT_2_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL,
	              GPIO13);

  // Serial
  serial_configure(serial, 38400, 0);
  serial_connect(serial);
}

int main() {
  serial_t serial;
  setup(&serial);

  for (size_t i = 0; i < 30; i++) {
    serial_write_byte(&serial, '-');
  }
  serial_write_byte(&serial, '\r');
  serial_write_byte(&serial, '\n');

  while (1) {
		/* gpio_set(GPIOC, GPIO13); */
		/* for (int i = 0; i < 1000000; ++i) __asm__("nop"); */
		/* gpio_clear(GPIOC, GPIO13); */
		/* for (int i = 0; i <  500000; ++i) __asm__("nop"); */

    uint8_t data = 0;
    serial_read_byte(&serial, &data);
    serial_write_byte(&serial, data);
    serial_write_byte(&serial, '\r');
    serial_write_byte(&serial, '\n');
  }

  return 0;
}
