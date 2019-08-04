#ifndef ZERO_FW_BME280_H
#define ZERO_FW_BME280_H

#include "zero/fw/i2c.h"

typedef struct bme280_config_t {


} bme280_config_t;

typedef struct bme280_t {
  i2c_t *i2c;
} bme280_t;

void bme280_init(bme280_t *sensor, i2c_t *i2c);
int bme280_setup(bme280_t *sensor, bme280_config_t *config);

#endif /* ZERO_FW_BME280_H */
