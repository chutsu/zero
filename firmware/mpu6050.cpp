#include "mpu6050.hpp"

void mpu6050_init(mpu6050_t *imu) {
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

int8_t mpu6050_ping() {
  return i2c_read_byte(MPU6050_ADDRESS, MPU6050_REG_WHO_AM_I);
}

uint8_t mpu6050_get_dplf() {
  uint8_t data = i2c_read_byte(MPU6050_ADDRESS, MPU6050_REG_CONFIG);
  data = (data & 0b00000111);
  return data;
}

void mpu6050_set_dplf(const uint8_t setting) {
  /*
      DPLF_CFG    Accelerometer
      ----------------------------------------
                  Bandwidth(Hz) | Delay(ms)
      0           260             0
      1           184             2.0
      2           94              3.0
      3           44              4.9
      4           21              8.5
      5           10              13.8
      6           5               19.0
      7           RESERVED        RESERVED


      DPLF_CFG    Gyroscope
      ----------------------------------------------
                  Bandwidth(Hz) | Delay(ms) | Fs(kHz)
      0           256             0.98        8
      1           188             1.9         1
      2           98              2.8         1
      3           42              4.8         1
      4           20              8.3         1
      5           10              13.4        1
      6           5               18.5        1
      7           RESERVED        RESERVED    8
  */
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_REG_CONFIG, setting);
}

uint8_t mpu6050_get_sample_rate_div() {
  return i2c_read_byte(MPU6050_ADDRESS, MPU6050_REG_SMPLRT_DIV);
}

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

void mpu6050_set_sample_rate_div(const int8_t div) {
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_REG_SMPLRT_DIV, div);
}

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

void mpu6050_set_gyro_range(const int8_t range) {
  uint8_t data = range << 3;
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_REG_GYRO_CONFIG, data);
}

uint8_t mpu6050_get_gyro_range() {
  uint8_t data = i2c_read_byte(MPU6050_ADDRESS, MPU6050_REG_GYRO_CONFIG);
  data = (data >> 3) & 0b00000011;
  return data;
}

void mpu6050_set_accel_range(const int8_t range) {
  // assert(range > 3 || range < 0);
  uint8_t data = range << 3;
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_REG_ACCEL_CONFIG, data);
}

uint8_t mpu6050_get_accel_range() {
  uint8_t data = i2c_read_byte(MPU6050_ADDRESS, MPU6050_REG_ACCEL_CONFIG);
  data  = (data >> 3) & 0b00000011;
	return data;
}

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
