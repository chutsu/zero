#ifndef ZERO_DRIVER_I2C_H
#define ZERO_DRIVER_I2C_H

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

int i2c_setup(i2c_t *i2c);
int i2c_set_slave(const i2c_t *i2c, const char slave_addr);
int i2c_read_bytes(const i2c_t *i2c,
                   const char reg_addr,
                   char *data,
                   size_t length);
int i2c_read_byte(const i2c_t *i2c, const char reg_addr, char *data);
int i2c_write_byte(const i2c_t *i2c, const char reg_addr, const char byte);
int i2c_write_raw_byte(const i2c_t *i2c, const char byte);
int i2c_write_bytes(const i2c_t *i2c,
                    const char reg_addr,
                    const char *data,
                    const size_t length);

#endif /* ZERO_DRIVER_I2C_H */
