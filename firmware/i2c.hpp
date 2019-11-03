#ifndef I2C_HPP
#define I2C_HPP

#include <stdlib.h>
#include <stdint.h>

#include <Wire.h>

/* DEFINES */
#define I2C_BUF_MAX 1024

void i2c_setup();
uint8_t i2c_read_byte(const uint8_t dev_addr, const uint8_t reg_addr);
void i2c_write_byte(const uint8_t dev_addr,
                    const uint8_t reg_addr,
                    const uint8_t value);
void i2c_read_bytes(const uint8_t dev_addr,
									  const uint8_t reg_addr,
										const size_t length,
										uint8_t *data);
void i2c_write_bytes(const uint8_t dev_addr,
                     const uint8_t reg_addr,
                     const uint8_t *data,
                     const size_t length);

#endif /* I2C_H */