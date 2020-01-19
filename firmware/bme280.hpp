#ifndef BME280_HPP
#define BME280_HPP

#include <stdint.h>

#include "i2c.hpp"

/* BME280 I2C Addresses */
#define BME280_ADDR (0x77)      // Primary I2C Address
#define BME280_ADDR_ALT (0x76)  // Alternate Address

/* BME280 Registers */
#define BME280_REG_DIG_T1 0x88
#define BME280_REG_DIG_T2 0x8A
#define BME280_REG_DIG_T3 0x8C
#define BME280_REG_DIG_P1 0x8E
#define BME280_REG_DIG_P2 0x90
#define BME280_REG_DIG_P3 0x92
#define BME280_REG_DIG_P4 0x94
#define BME280_REG_DIG_P5 0x96
#define BME280_REG_DIG_P6 0x98
#define BME280_REG_DIG_P7 0x9A
#define BME280_REG_DIG_P8 0x9C
#define BME280_REG_DIG_P9 0x9E
#define BME280_REG_DIG_H1 0xA1
#define BME280_REG_DIG_H2 0xE1
#define BME280_REG_DIG_H3 0xE3
#define BME280_REG_DIG_H4 0xE4
#define BME280_REG_DIG_H5 0xE5
#define BME280_REG_DIG_H6 0xE7
#define BME280_REG_CHIPID 0xD0
#define BME280_REG_VERSION 0xD1
#define BME280_REG_SOFTRESET 0xE0
#define BME280_REG_CAL26 0xE1    // R calibration stored in 0xE1-0xF0
#define BME280_REG_CONTROLHUMID 0xF2
#define BME280_REG_STATUS 0XF3
#define BME280_REG_CONTROL 0xF4
#define BME280_REG_CONFIG 0xF5
#define BME280_REG_PRESSUREDATA 0xF7
#define BME280_REG_TEMPDATA 0xFA
#define BME280_REG_HUMIDDATA 0xFD

/* Sampling rates */
#define BME280_SAMPLING_NONE 0b000
#define BME280_SAMPLING_X1 0b001
#define BME280_SAMPLING_X2 0b010
#define BME280_SAMPLING_X4 0b011
#define BME280_SAMPLING_X8 0b100
#define BME280_SAMPLING_X16 0b101

/* Power modes */
#define BME280_MODE_SLEEP 0b00
#define BME280_MODE_FORCED 0b01
#define BME280_MODE_NORMAL 0b11

/* Filter values */
#define BME280_FILTER_OFF 0b000
#define BME280_FILTER_X2 0b001
#define BME280_FILTER_X4 0b010
#define BME280_FILTER_X8 0b011
#define BME280_FILTER_X16 0b10

/* Standby duration in ms */
#define BME280_STANDBY_MS_0_5 0b000
#define BME280_STANDBY_MS_10 0b110
#define BME280_STANDBY_MS_20 0b111
#define BME280_STANDBY_MS_62_5 0b001
#define BME280_STANDBY_MS_125 0b010
#define BME280_STANDBY_MS_250 0b011
#define BME280_STANDBY_MS_500 0b100
#define BME280_STANDBY_MS_1000 0b101

/* BME280 Calibration Data */
typedef struct {
  uint16_t dig_T1;  // temperature compensation value
  int16_t dig_T2;   // temperature compensation value
  int16_t dig_T3;   // temperature compensation value

  uint16_t dig_P1;  // pressure compensation value
  int16_t dig_P2;   // pressure compensation value
  int16_t dig_P3;   // pressure compensation value
  int16_t dig_P4;   // pressure compensation value
  int16_t dig_P5;   // pressure compensation value
  int16_t dig_P6;   // pressure compensation value
  int16_t dig_P7;   // pressure compensation value
  int16_t dig_P8;   // pressure compensation value
  int16_t dig_P9;   // pressure compensation value

  uint8_t dig_H1;   // humidity compensation value
  int16_t dig_H2;   // humidity compensation value
  uint8_t dig_H3;   // humidity compensation value
  int16_t dig_H4;   // humidity compensation value
  int16_t dig_H5;   // humidity compensation value
  int8_t dig_H6;    // humidity compensation value
} bme280_calib_data;

/* BME280 */
typedef struct bme280_t {
} bme280_t;

float bme280_read_temperature(bme280_t *sensor) {

}

float bme280_read_pressure(bme280_t *sensor) {

}

float bme280_read_humidity(bme280_t *sensor) {

}

void bme280_setup(bme280_t *sensor) {

}

#endif // ZERO_FW_BME280_HPP
