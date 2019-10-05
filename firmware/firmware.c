#include <string.h>
#include <stdlib.h>

#include "serial.h"

#define BUF_MAX_SIZE 100

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
  for (size_t i = 0; i < 30; i++) {
    serial_write_byte(serial, '-');
  }
  serial_write_string(serial, "\r\n");
}

int main() {
  serial_t serial;
  setup(&serial);

  while (1) {
    uint8_t data = 0;
    char buf[BUF_MAX_SIZE] = {0};

    while (1) {
      serial_read_byte(&serial, &data);
      if (data == '\r' || data == '\n') {
        break;
      }

      buf[strlen(buf)] = data;
    }

    serial_write_byte(&serial, '[');
    serial_write_string(&serial, buf);
    serial_write_byte(&serial, ']');
    serial_write_string(&serial, "\r\n");
  }

  return 0;
}
