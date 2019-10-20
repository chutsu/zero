#include <string.h>
#include <stdlib.h>

#include "serial.h"
#include "i2c.h"
#include "mpu6050.h"

#define BUF_MAX_SIZE 100

void setup(serial_t *serial, i2c_t *i2c) {
	rcc_clock_setup_in_hse_8mhz_out_72mhz();  // For the "blue pill"

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
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_I2C1);
	gpio_set_mode(GPIOB,
								GPIO_MODE_OUTPUT_50_MHZ,
								GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
								GPIO6|GPIO7);			// I2C
	gpio_set(GPIOB,GPIO6|GPIO7);		// Idle high
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
  /* i2c_start(&i2c, MPU6050_ADDRESS, 1); */
  /* const uint8_t buf = i2c_read_byte(&i2c, MPU6050_REG_WHO_AM_I, 1); */
	/* i2c_stop(&i2c); */

	/* i2c_start(&i2c, MPU6050_ADDRESS, 0); */
	/* uint8_t buf = i2c_read_byte(&i2c, MPU6050_REG_WHO_AM_I, 1); */
	/* i2c_send_stop(i2c.conn); */

	i2c_send_start(i2c.conn);
	i2c_send_7bit_address(i2c.conn, MPU6050_ADDRESS, I2C_READ);
	/* i2c_send_data(i2c.conn, MPU6050_REG_WHO_AM_I); */
	i2c_send_7bit_address(i2c.conn, MPU6050_REG_WHO_AM_I, I2C_READ);
	uint8_t buf = i2c_get_data(i2c.conn);
	i2c_send_stop(i2c.conn);

	char str[10];
	snprintf(str, sizeof(str), "%d", buf);

  serial_write_string(&serial, "ping addr: ");
  serial_write_string(&serial, str);
  serial_write_string(&serial, "\r\n");
  serial_write_string(&serial, "ping end\r\n");

  while (1) {

  }

  return 0;
}
