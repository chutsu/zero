#include <string.h>
#include <stdlib.h>

#include "serial.h"
#include "i2c.h"
#include "mpu6050.h"

#define BUF_MAX_SIZE 100

void setup(serial_t *serial, i2c_t *i2c) {
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

  // I2C
  i2c_configure(i2c);
}

void command_handler(serial_t *serial) {
  uint8_t data = 0;
  char buf[BUF_MAX_SIZE] = {0};

  while (1) {
    serial_read_byte(serial, &data);
    if (data == '\r' || data == '\n') {
      break;
    }
    buf[strlen(buf)] = data;
  }
}

int main() {
  serial_t serial;
  i2c_t i2c;
  setup(&serial, &i2c);

  serial_write_string(&serial, "ping start\r\n");
  uint8_t buf = 0;
  serial_write_string(&serial, "set slave\r\n");
  i2c_set_slave(&i2c, MPU6050_ADDRESS);
  serial_write_string(&serial, "read byte\r\n");
  i2c_read_byte(&i2c, MPU6050_REG_WHO_AM_I, &buf);
  serial_write_string(&serial, "ping done\r\n");

  char str[10] = {0};
  /* buf = 0x68; */
  buf = (buf >> 1);
  itoa(buf, str, 10);
  serial_write_string(&serial, "ping addr: ");
  serial_write_string(&serial, str);
  serial_write_string(&serial, "\r\n");

  while (1) {

  }

  return 0;
}
