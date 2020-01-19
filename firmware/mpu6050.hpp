#ifndef MPU6050_HPP
#define MPU6050_HPP

// #include <math.h>
#include <time.h>
#include <string.h>

#include "core.hpp"
#include "i2c.hpp"

// GENERAL
#define MPU6050_ADDRESS 0x68
#define MPU6050_ADDRESS_AD0_LOW 0x68  // addr pin low (GND) [default]
#define MPU6050_ADDRESS_AD0_HIGH 0x69 // addr pin high (VCC)

// REGISTER ADDRESSES
#define MPU6050_REG_XG_OFFS_TC 0x00
#define MPU6050_REG_YG_OFFS_TC 0x01
#define MPU6050_REG_ZG_OFFS_TC 0x02
#define MPU6050_REG_X_FINE_GAIN 0x03
#define MPU6050_REG_Y_FINE_GAIN 0x04
#define MPU6050_REG_Z_FINE_GAIN 0x05
#define MPU6050_REG_XA_OFFS_H 0x06
#define MPU6050_REG_XA_OFFS_L_TC 0x07
#define MPU6050_REG_YA_OFFS_H 0x08
#define MPU6050_REG_YA_OFFS_L_TC 0x09
#define MPU6050_REG_ZA_OFFS_H 0x0A
#define MPU6050_REG_ZA_OFFS_L_TC 0x0B
#define MPU6050_REG_XG_OFFS_USRH 0x13
#define MPU6050_REG_XG_OFFS_USRL 0x14
#define MPU6050_REG_YG_OFFS_USRH 0x15
#define MPU6050_REG_YG_OFFS_USRL 0x16
#define MPU6050_REG_ZG_OFFS_USRH 0x17
#define MPU6050_REG_ZG_OFFS_USRL 0x18
#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_FF_THR 0x1D
#define MPU6050_REG_FF_DUR 0x1E
#define MPU6050_REG_MOT_THR 0x1F
#define MPU6050_REG_MOT_DUR 0x20
#define MPU6050_REG_ZRMOT_THR 0x21
#define MPU6050_REG_ZRMOT_DUR 0x22
#define MPU6050_REG_FIFO_EN 0x23
#define MPU6050_REG_I2C_MST_CTRL 0x24
#define MPU6050_REG_I2C_SLV0_ADDR 0x25
#define MPU6050_REG_I2C_SLV0_REG 0x26
#define MPU6050_REG_I2C_SLV0_CTRL 0x27
#define MPU6050_REG_I2C_SLV1_ADDR 0x28
#define MPU6050_REG_I2C_SLV1_REG 0x29
#define MPU6050_REG_I2C_SLV1_CTRL 0x2A
#define MPU6050_REG_I2C_SLV2_ADDR 0x2B
#define MPU6050_REG_I2C_SLV2_REG 0x2C
#define MPU6050_REG_I2C_SLV2_CTRL 0x2D
#define MPU6050_REG_I2C_SLV3_ADDR 0x2E
#define MPU6050_REG_I2C_SLV3_REG 0x2F
#define MPU6050_REG_I2C_SLV3_CTRL 0x30
#define MPU6050_REG_I2C_SLV4_ADDR 0x31
#define MPU6050_REG_I2C_SLV4_REG 0x32
#define MPU6050_REG_I2C_SLV4_DO 0x33
#define MPU6050_REG_I2C_SLV4_CTRL 0x34
#define MPU6050_REG_I2C_SLV4_DI 0x35
#define MPU6050_REG_I2C_MST_STATUS 0x36
#define MPU6050_REG_INT_PIN_CFG 0x37
#define MPU6050_REG_INT_ENABLE 0x38
#define MPU6050_REG_DMP_INT_STATUS 0x39
#define MPU6050_REG_INT_STATUS 0x3A
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_ACCEL_XOUT_L 0x3C
#define MPU6050_REG_ACCEL_YOUT_H 0x3D
#define MPU6050_REG_ACCEL_YOUT_L 0x3E
#define MPU6050_REG_ACCEL_ZOUT_H 0x3F
#define MPU6050_REG_ACCEL_ZOUT_L 0x40
#define MPU6050_REG_TEMP_OUT_H 0x41
#define MPU6050_REG_TEMP_OUT_L 0x42
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_GYRO_XOUT_L 0x44
#define MPU6050_REG_GYRO_YOUT_H 0x45
#define MPU6050_REG_GYRO_YOUT_L 0x46
#define MPU6050_REG_GYRO_ZOUT_H 0x47
#define MPU6050_REG_GYRO_ZOUT_L 0x48
#define MPU6050_REG_EXT_SENS_DATA_00 0x49
#define MPU6050_REG_EXT_SENS_DATA_01 0x4A
#define MPU6050_REG_EXT_SENS_DATA_02 0x4B
#define MPU6050_REG_EXT_SENS_DATA_03 0x4C
#define MPU6050_REG_EXT_SENS_DATA_04 0x4D
#define MPU6050_REG_EXT_SENS_DATA_05 0x4E
#define MPU6050_REG_EXT_SENS_DATA_06 0x4F
#define MPU6050_REG_EXT_SENS_DATA_07 0x50
#define MPU6050_REG_EXT_SENS_DATA_08 0x51
#define MPU6050_REG_EXT_SENS_DATA_09 0x52
#define MPU6050_REG_EXT_SENS_DATA_10 0x53
#define MPU6050_REG_EXT_SENS_DATA_11 0x54
#define MPU6050_REG_EXT_SENS_DATA_12 0x55
#define MPU6050_REG_EXT_SENS_DATA_13 0x56
#define MPU6050_REG_EXT_SENS_DATA_14 0x57
#define MPU6050_REG_EXT_SENS_DATA_15 0x58
#define MPU6050_REG_EXT_SENS_DATA_16 0x59
#define MPU6050_REG_EXT_SENS_DATA_17 0x5A
#define MPU6050_REG_EXT_SENS_DATA_18 0x5B
#define MPU6050_REG_EXT_SENS_DATA_19 0x5C
#define MPU6050_REG_EXT_SENS_DATA_20 0x5D
#define MPU6050_REG_EXT_SENS_DATA_21 0x5E
#define MPU6050_REG_EXT_SENS_DATA_22 0x5F
#define MPU6050_REG_EXT_SENS_DATA_23 0x60
#define MPU6050_REG_MOT_DETECT_STATUS 0x61
#define MPU6050_REG_I2C_SLV0_DO 0x63
#define MPU6050_REG_I2C_SLV1_DO 0x64
#define MPU6050_REG_I2C_SLV2_DO 0x65
#define MPU6050_REG_I2C_SLV3_DO 0x66
#define MPU6050_REG_I2C_MST_DELAY_CTRL 0x67
#define MPU6050_REG_SIGNAL_PATH_RESET 0x68
#define MPU6050_REG_MOT_DETECT_CTRL 0x69
#define MPU6050_REG_USER_CTRL 0x6A
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_PWR_MGMT_2 0x6C
#define MPU6050_REG_BANK_SEL 0x6D
#define MPU6050_REG_MEM_START_ADDR 0x6E
#define MPU6050_REG_MEM_R_W 0x6F
#define MPU6050_REG_DMP_CFG_1 0x70
#define MPU6050_REG_DMP_CFG_2 0x71
#define MPU6050_REG_FIFO_COUNTH 0x72
#define MPU6050_REG_FIFO_COUNTL 0x73
#define MPU6050_REG_FIFO_R_W 0x74
#define MPU6050_REG_WHO_AM_I 0x75

typedef struct mpu6050_t {
  int8_t ok = 0;

  float accel_sensitivity = 0.0f;
  float gyro_sensitivity = 0.0f;
  float accel[3] = {0.0f};
  float gyro[3] = {0.0f};

  float temperature = 0.0f;
  float sample_rate = 0.0f;
} mpu6050_t;

/** Get MPU6050 address */
int8_t mpu6050_ping() {
  return i2c_read_byte(MPU6050_ADDRESS, MPU6050_REG_WHO_AM_I);
}

/**
 * Get `DPLF_CFG` (Digital Low-Pass Filter) config
 *
 * DPLF_CFG    Accelerometer
 * ----------------------------------------
 *             Bandwidth(Hz) | Delay(ms)
 * 0           260             0
 * 1           184             2.0
 * 2           94              3.0
 * 3           44              4.9
 * 4           21              8.5
 * 5           10              13.8
 * 6           5               19.0
 * 7           RESERVED        RESERVED
 *
 * DPLF_CFG    Gyroscope
 * ----------------------------------------------
 *             Bandwidth(Hz) | Delay(ms) | Fs(kHz)
 * 0           256             0.98        8
 * 1           188             1.9         1
 * 2           98              2.8         1
 * 3           42              4.8         1
 * 4           20              8.3         1
 * 5           10              13.4        1
 * 6           5               18.5        1
 * 7           RESERVED        RESERVED    8
 */
uint8_t mpu6050_get_dplf() {
  uint8_t data = i2c_read_byte(MPU6050_ADDRESS, MPU6050_REG_CONFIG);
  data = (data & 0b00000111);
  return data;
}

/**
 * Set `DPLF_CFG` (Digital Low-Pass Filter) config
 *
 * DPLF_CFG    Accelerometer
 * ----------------------------------------
 *             Bandwidth(Hz) | Delay(ms)
 * 0           260             0
 * 1           184             2.0
 * 2           94              3.0
 * 3           44              4.9
 * 4           21              8.5
 * 5           10              13.8
 * 6           5               19.0
 * 7           RESERVED        RESERVED
 *
 * DPLF_CFG    Gyroscope
 * ----------------------------------------------
 *             Bandwidth(Hz) | Delay(ms) | Fs(kHz)
 * 0           256             0.98        8
 * 1           188             1.9         1
 * 2           98              2.8         1
 * 3           42              4.8         1
 * 4           20              8.3         1
 * 5           10              13.4        1
 * 6           5               18.5        1
 * 7           RESERVED        RESERVED    8
 */
void mpu6050_set_dplf(const uint8_t setting) {
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_REG_CONFIG, setting);
}

/** Get sample rate division */
uint8_t mpu6050_get_sample_rate_div() {
  return i2c_read_byte(MPU6050_ADDRESS, MPU6050_REG_SMPLRT_DIV);
}

/** Set sample rate */
void mpu6050_set_sample_rate_div(const int8_t div) {
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_REG_SMPLRT_DIV, div);
}

/** Set sample rate division */
void mpu6050_set_sample_rate(float sample_rate) {
  // gyro_output_rate = 8kHz (iff DPLF_CFG: 0 or 7) or 1kHz
  uint8_t dlpf_cfg = mpu6050_get_dplf();
  float gyro_rate = ((dlpf_cfg == 0) || (dlpf_cfg == 7)) ? 8000.0f : 1000.0f;

  // Calculate and set sample rate divider needed to get desired sample rate.
  // The equation is given in MPU6050 register map documentation (page 12 of
  // 46). Under the sample rate divider register section.
  //
  // sample_rate = gyro_output_rate / (1 + smplrt_div)
  // smplrt_div = (gyro_output_rate / sample_rate) - 1
  float smplrt_div = (gyro_rate / sample_rate) - 1;
  mpu6050_set_sample_rate_div(smplrt_div);
}

/** Get sample rate */
float mpu6050_get_sample_rate() {
  // Get sample rate divider
  uint8_t smplrt_div = mpu6050_get_sample_rate_div();

  // Get gyro sample rate
  uint8_t dlpf_cfg = mpu6050_get_dplf();
  float gyro_rate = ((dlpf_cfg == 0) || (dlpf_cfg == 7)) ? 8000.0f : 1000.0f;

  // Calculate sample rate. The equation is given in MPU6050 register map
  // documentation (page 12 of 46). Under the sample rate divider register
  // section.
  return gyro_rate / (1 + smplrt_div);
}

/** Set gyro range */
void mpu6050_set_gyro_range(const int8_t range) {
  uint8_t data = range << 3;
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_REG_GYRO_CONFIG, data);
}

/** Get gyro range */
uint8_t mpu6050_get_gyro_range() {
  uint8_t data = i2c_read_byte(MPU6050_ADDRESS, MPU6050_REG_GYRO_CONFIG);
  data = (data >> 3) & 0b00000011;
  return data;
}

/** Set accelerometer range */
void mpu6050_set_accel_range(const int8_t range) {
  // assert(range > 3 || range < 0);
  uint8_t data = range << 3;
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_REG_ACCEL_CONFIG, data);
}

/** Get accelerometer range */
uint8_t mpu6050_get_accel_range() {
  uint8_t data = i2c_read_byte(MPU6050_ADDRESS, MPU6050_REG_ACCEL_CONFIG);
  data  = (data >> 3) & 0b00000011;
  return data;
}

/** Get IMU data */
void mpu6050_get_data(mpu6050_t *imu) {
  // Read data
  uint8_t data[14] = {0};
  memset(data, '\0', 14);
  i2c_read_bytes(MPU6050_ADDRESS, MPU6050_REG_ACCEL_XOUT_H, 14, data);

  // Accelerometer
  const double g = 9.81; // Gravitational constant
  const int16_t raw_ax = (data[0] << 8) | data[1];
  const int16_t raw_ay = (data[2] << 8) | data[3];
  const int16_t raw_az = (data[4] << 8) | data[5];
  imu->accel[0] = (raw_ax / imu->accel_sensitivity) * g;
  imu->accel[1] = (raw_ay / imu->accel_sensitivity) * g;
  imu->accel[2] = (raw_az / imu->accel_sensitivity) * g;

  // Temperature
  const int16_t raw_temp = (data[6] << 8) | (data[7]);
  imu->temperature = raw_temp / 340.0 + 36.53;

  // Gyroscope
  const int16_t raw_gx = (data[8] << 8) | (data[9]);
  const int16_t raw_gy = (data[10] << 8) | (data[11]);
  const int16_t raw_gz = (data[12] << 8) | (data[13]);
  imu->gyro[0] = deg2rad(raw_gx / imu->gyro_sensitivity);
  imu->gyro[1] = deg2rad(raw_gy / imu->gyro_sensitivity);
  imu->gyro[2] = deg2rad(raw_gz / imu->gyro_sensitivity);

  // Set last_updated
  // imu->last_updated = clock();
}

/** Initialize MPU6050 sensor */
void mpu6050_setup(mpu6050_t *imu) {
  // Configure Digital low-pass filter
  uint8_t dlpf_cfg = 0;
  mpu6050_set_dplf(dlpf_cfg);

  // Set power management register
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_REG_PWR_MGMT_1, 0x00);

  // Configure Gyroscope range
  const uint8_t gyro_range = 0;
  mpu6050_set_gyro_range(gyro_range);
  switch (gyro_range) {
  case 0: imu->gyro_sensitivity = 131.0; break;
  case 1: imu->gyro_sensitivity = 65.5; break;
  case 2: imu->gyro_sensitivity = 32.8; break;
  case 3: imu->gyro_sensitivity = 16.4; break;
  }

  // Configure accel range
  const uint8_t accel_range = 0;
  mpu6050_set_accel_range(accel_range);
  switch (accel_range) {
  case 0: imu->accel_sensitivity = 16384.0; break;
  case 1: imu->accel_sensitivity = 8192.0; break;
  case 2: imu->accel_sensitivity = 4096.0; break;
  case 3: imu->accel_sensitivity = 2048.0; break;
  }

  // Configure sample rate
  float sample_rate = 400;
  mpu6050_set_sample_rate(sample_rate);
  imu->sample_rate = mpu6050_get_sample_rate();
}

// void print_mpu6050_config() {
//   Serial.print("Accel sensitivity: ");
//   Serial.print(imu.accel_sensitivity);
//   Serial.print("\n\r");
//
//   Serial.print("Gyro sensitivity: ");
//   Serial.print(imu.gyro_sensitivity);
//   Serial.print("\n\r");
//
//   Serial.print("DPLF: ");
//   Serial.print(mpu6050_get_dplf());
//   Serial.print("\n\r");
//
//   Serial.print("Ping: ");
//   Serial.print(mpu6050_ping());
//   Serial.print("\n\r");
//
//   Serial.print("Sample rate div: ");
//   Serial.print(mpu6050_get_sample_rate_div());
//   Serial.print("\n\r");
//
//   Serial.print("Sample rate: ");
//   Serial.print(imu.sample_rate);
//   Serial.print("\n\r");
// }

#endif /* MPU6050_H */
