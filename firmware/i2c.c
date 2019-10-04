#include "i2c.h"

void i2c_configure(i2c_t *i2c) {
  i2c->connfd = I2C1;
  i2c->ticks = 1000;

  rcc_periph_clock_enable(RCC_GPIOB);
  i2c_peripheral_disable(i2c->connfd);
  i2c_reset(i2c->connfd);
  I2C_CR1(i2c->connfd) &= ~I2C_CR1_STOP; // Clear stop
  i2c_set_standard_mode(i2c->connfd);  // 100kHz mode
  i2c_set_clock_frequency(i2c->connfd, I2C_CR2_FREQ_36MHZ);  // APB Freq
  i2c_set_trise(i2c->connfd, 36);  // 1000 ns
  i2c_set_dutycycle(i2c->connfd, I2C_CCR_DUTY_DIV2);  // 1000 ns
  i2c_set_ccr(i2c->connfd, 180);  // 100 kHz <= 180 * 1/36M
}

void i2c_wait_busy(const i2c_t *i2c) {
  while (I2C_SR2(i2c->connfd) & I2C_SR2_BUSY);
}

void i2c_set_slave(const i2c_t *i2c, const uint8_t slave_addr) {
  i2c_wait_busy(i2c);           // Block till ready
  I2C_SR1(i2c) &= ~I2C_SR1_AF;  // Clear Ack failure
  i2c_clear_stop(i2c->connfd);
  i2c_send_start(i2c->connfd);
  i2c_send_7bit_address(i2c->connfd, slave_addr, I2C_WRITE);
}

void i2c_read_byte(const i2c_t *i2c,
                   const uint8_t reg_addr,
                   uint8_t *data) {
  i2c_enable_ack(i2c->connfd);
  i2c_send_data(i2c->connfd, reg_addr);
  *data = i2c_get_data(i2c->connfd);
  i2c_send_stop(i2c->connfd);
}

void i2c_read_bytes(const i2c_t *i2c,
                      const uint8_t reg_addr,
                      uint8_t *data,
                      size_t length) {
  i2c_enable_ack(i2c->connfd);
  i2c_send_start(i2c->connfd);
  i2c_send_data(i2c->connfd, reg_addr);
  for (size_t i = 0; i < length; i++) {
    data[i] = i2c_get_data(i2c->connfd);
  }
  i2c_send_stop(i2c->connfd);
}

void i2c_write_byte(const i2c_t *i2c,
                    const uint8_t reg_addr,
                    const uint8_t byte) {
  i2c_send_start(i2c->connfd);
  i2c_send_data(i2c->connfd, reg_addr);
  i2c_send_data(i2c->connfd, byte);
  i2c_send_stop(i2c->connfd);
}

void i2c_write_raw_byte(const i2c_t *i2c, const uint8_t byte) {
  i2c_send_start(i2c->connfd);
  i2c_send_data(i2c->connfd, byte);
  i2c_send_stop(i2c->connfd);
}

void i2c_write_bytes(const i2c_t *i2c,
                       const uint8_t reg_addr,
                       const uint8_t *data,
                       const size_t length) {
  i2c_send_start(i2c->connfd);
  i2c_send_data(i2c->connfd, reg_addr);
  for (size_t i = 0; i < length; i++) {
    i2c_send_data(i2c->connfd, data[i]);
  }
  i2c_send_stop(i2c->connfd);
}
