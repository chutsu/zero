#include "i2c.hpp"

void i2c_setup() {
  Wire.begin();
}

uint8_t i2c_read_byte(const uint8_t dev_addr, const uint8_t reg_addr) {
  // Request
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();

  // Response
  uint8_t size = 1;
  uint8_t last = 1;
  Wire.beginTransmission(dev_addr);
  Wire.requestFrom(dev_addr, size, last);
  const uint8_t data = Wire.read();
  Wire.endTransmission();

  return data;
}

void i2c_write_byte(const uint8_t dev_addr,
                    const uint8_t reg_addr,
                    const uint8_t value) {
  // Request
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.write(value);
  Wire.endTransmission();
}

void i2c_read_bytes(const uint8_t dev_addr,
                    const uint8_t reg_addr,
                    const size_t length,
                    uint8_t *data) {
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);

  Wire.requestFrom(dev_addr, length);
  for (size_t i = 0; i < length; i++) {
    data[i] = Wire.read();
  }

  Wire.endTransmission();
}

void i2c_write_bytes(const uint8_t dev_addr,
                     const uint8_t reg_addr,
                     const uint8_t *data,
                     const size_t length) {
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.write(data, length);
  Wire.endTransmission();
}
