#include "i2c.h"

void i2c_configure(i2c_t *i2c) {
  i2c->conn = I2C1;
  i2c->timeout = 1000;

  i2c_reset(i2c->conn);
  i2c_peripheral_disable(I2C1);
  i2c_set_standard_mode(i2c->conn);  // 100kHz mode
  i2c_set_clock_frequency(i2c->conn, I2C_CR2_FREQ_36MHZ);  // APB Freq
  i2c_set_trise(i2c->conn, 36);  // 1000 ns
  i2c_set_dutycycle(i2c->conn, I2C_CCR_DUTY_DIV2);  // 1000 ns
  i2c_set_ccr(i2c->conn, 180);  // 100 kHz <= 180 * 1/36M
  i2c_peripheral_enable(I2C1);
}

void i2c_wait_busy(const i2c_t *i2c) {
  while (I2C_SR2(i2c->conn) & I2C_SR2_BUSY);
}

void i2c_start(const i2c_t *i2c, const uint8_t addr, const int rw) {
	i2c_wait_busy(i2c);			// Block until not busy
	I2C_SR1(i2c->conn) &= ~I2C_SR1_AF;	// Clear Acknowledge failure
	i2c_clear_stop(i2c->conn);		// Do not generate a Stop
	if (rw == 0) {
		i2c_enable_ack(i2c->conn);
	}
	i2c_send_start(i2c->conn);		// Generate a Start/Restart

	// Loop until ready:
	while (!((I2C_SR1(i2c->conn) & I2C_SR1_SB)
	  && (I2C_SR2(i2c->conn) & (I2C_SR2_MSL|I2C_SR2_BUSY)))) {
	}

	// Send Address & R/W flag:
	i2c_send_7bit_address(i2c->conn, addr, rw == 0 ? I2C_READ : I2C_WRITE);

	// Wait until completion, NAK or timeout
	while (!(I2C_SR1(i2c->conn) & I2C_SR1_ADDR)) {
		if (I2C_SR1(i2c->conn) & I2C_SR1_AF) {
			i2c_send_stop(i2c->conn);
			I2C_SR1(i2c->conn);
			I2C_SR2(i2c->conn); 	// Clear flags
		}
	}

	I2C_SR2(i2c->conn);		// Clear flags
}

uint8_t i2c_read_byte(const i2c_t *i2c, const uint8_t reg_addr, const int last) {
	/* Send read request */
	i2c_send_data(i2c->conn, 0x00);
	i2c_send_start(i2c->conn);
	i2c_send_7bit_address(i2c->conn, reg_addr, I2C_READ);
	I2C_SR2(i2c->conn);		// Clear flags

	/* Read byte */
	if (last) {
		i2c_disable_ack(i2c->conn);
	}
	return i2c_get_data(i2c->conn);
}

void i2c_stop(const i2c_t *i2c) {
	i2c_send_stop(i2c->conn);
}

void i2c_read_bytes(const i2c_t *i2c,
                      const uint8_t reg_addr,
                      uint8_t *data,
                      size_t length) {
  i2c_enable_ack(i2c->conn);
  i2c_send_start(i2c->conn);
  i2c_send_data(i2c->conn, reg_addr);
  for (size_t i = 0; i < length; i++) {
    data[i] = i2c_get_data(i2c->conn);
  }
  i2c_send_stop(i2c->conn);
}

void i2c_write_byte(const i2c_t *i2c,
                    const uint8_t reg_addr,
                    const uint8_t byte) {
  i2c_send_start(i2c->conn);
  i2c_send_data(i2c->conn, reg_addr);
  i2c_send_data(i2c->conn, byte);
  i2c_send_stop(i2c->conn);
}

void i2c_write_raw_byte(const i2c_t *i2c, const uint8_t byte) {
  i2c_send_start(i2c->conn);
  i2c_send_data(i2c->conn, byte);
  i2c_send_stop(i2c->conn);
}

void i2c_write_bytes(const i2c_t *i2c,
                       const uint8_t reg_addr,
                       const uint8_t *data,
                       const size_t length) {
  i2c_send_start(i2c->conn);
  i2c_send_data(i2c->conn, reg_addr);
  for (size_t i = 0; i < length; i++) {
    i2c_send_data(i2c->conn, data[i]);
  }
  i2c_send_stop(i2c->conn);
}
