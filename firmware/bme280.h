#ifndef BME280_H
#define BME280_H

#include "i2c.h"

typedef struct bme280_config_t {


} bme280_config_t;

typedef struct bme280_t {
  i2c_t *i2c;
} bme280_t;

void bme280_configure(bme280_t *sensor, i2c_t *i2c);

#endif /* ZERO_FW_BME280_H */
