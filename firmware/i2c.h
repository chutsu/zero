#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdlib.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include "core.h"

/* ERROR MESSAGES */
#define I2C_INIT_FAILED "failed to initialize I2C!"

/* DEFINES */
#define I2C_BUF_MAX 1024

typedef struct i2c_t {
  uint32_t connfd;
  uint32_t ticks;
} i2c_t;

void i2c_configure(i2c_t *i2c);
void i2c_set_slave(const i2c_t *i2c, const uint8_t slave_addr);
void i2c_read_bytes(const i2c_t *i2c,
                    const uint8_t reg_addr,
                    uint8_t *data,
                    size_t length);
void i2c_read_byte(const i2c_t *i2c, const uint8_t reg_addr, uint8_t *data);
void i2c_write_byte(const i2c_t *i2c, const uint8_t reg_addr, const uint8_t byte);
void i2c_write_raw_byte(const i2c_t *i2c, const uint8_t byte);
void i2c_write_bytes(const i2c_t *i2c,
                       const uint8_t reg_addr,
                       const uint8_t *data,
                       const size_t length);

#endif /* I2C_H */
