#include "zero/fw/i2c.h"

int8_t i2c_init(i2c_t *i2c) {
  /* Setup */
  int8_t adapter_nr = 1; /* Probably dynamically determined */
  char path[20];
  memset(path, '\0', sizeof(char) * 20);
  snprintf(path, 19, "/dev/i2c-%d", adapter_nr);

  /* Open i2c connection */
  int8_t fd = open(path, O_RDWR);
  if (fd < 0) {
    return -1;
  } else {
    i2c->fd = fd;
  }

  return 0;
}

int8_t i2c_set_slave(const i2c_t *i2c, const uint8_t slave_addr) {
  return ioctl(i2c->fd, I2C_SLAVE, slave_addr);
}

int8_t i2c_read_byte(const i2c_t *i2c, const uint8_t reg_addr, uint8_t *data) {
  uint8_t buf[1] = {reg_addr};
  if (write(i2c->fd, buf, 1) != 1) {
    return -1;
  }

  if (read(i2c->fd, data, 1) != 1) {
    return -2;
  }

  return 0;
}

int8_t i2c_read_bytes(const i2c_t *i2c,
                   const uint8_t reg_addr,
                   uint8_t *data,
                   size_t length) {
  uint8_t buf[1];

  buf[0] = reg_addr;
  if (write(i2c->fd, buf, 1) != 1) {
    return -1;
  }

  if (read(i2c->fd, data, length) != (int) length) {
    return -2;
  }

  return 0;
}

int8_t i2c_write_byte(const i2c_t *i2c, const uint8_t reg_addr, const uint8_t byte) {
  uint8_t buf[2];

  buf[0] = reg_addr;
  buf[1] = byte;
  if (write(i2c->fd, buf, 2) != 1) {
    return -1;
  }

  return 0;
}

int8_t i2c_write_raw_byte(const i2c_t *i2c, const uint8_t byte) {
  if (write(i2c->fd, &byte, 1) != 1) {
    return -1;
  }

  return 0;
}

int8_t i2c_write_bytes(const i2c_t *i2c,
                       const uint8_t reg_addr,
                       const uint8_t *data,
                       const size_t length) {
  /* Create buf */
  uint8_t buf[I2C_BUF_MAX];
  memset(buf, '\0', sizeof(char) * I2C_BUF_MAX);
  buf[0] = reg_addr;
  for (int8_t i = 1; i < (int) length + 1; i++) {
    buf[i] = data[i];
  }

  /* Write bytes */
  if (write(i2c->fd, buf, length + 1) != 1) {
    return -1;
  }

  return 0;
}
