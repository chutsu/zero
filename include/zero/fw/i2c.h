#ifndef ZERO_FW_I2C_H
#define ZERO_FW_I2C_H

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>

/* ERROR MESSAGES */
#define I2C_INIT_FAILED "failed to initialize I2C!"

/* DEFINES */
#define I2C_BUF_MAX 1024

typedef struct i2c_t {
  int fd;
} i2c_t;

int8_t i2c_setup(i2c_t *i2c);
int8_t i2c_set_slave(const i2c_t *i2c, const uint8_t slave_addr);
int8_t i2c_read_bytes(const i2c_t *i2c,
                      const uint8_t reg_addr,
                      uint8_t *data,
                      size_t length);
int8_t i2c_read_byte(const i2c_t *i2c, const uint8_t reg_addr, uint8_t *data);
int8_t i2c_write_byte(const i2c_t *i2c, const uint8_t reg_addr, const uint8_t byte);
int8_t i2c_write_raw_byte(const i2c_t *i2c, const uint8_t byte);
int8_t i2c_write_bytes(const i2c_t *i2c,
                    const uint8_t reg_addr,
                    const uint8_t *data,
                    const size_t length);

#endif /* ZERO_FW_I2C_H */
