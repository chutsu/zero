#include "zero/fw/mpu6050.h"

int8_t mpu6050_init(mpu6050_t *imu, const mpu6050_config_t *config) {
  int8_t retval = 0;

  // Setup i2c
  i2c_set_slave(imu->i2c, MPU6050_ADDRESS);

  // Set dplf
  mpu6050_set_dplf(imu, config->dplf);
  retval = mpu6050_get_dplf(imu);
  if (retval > 7 || retval < 0) {
    return -1;
  } else {
    imu->dplf_config = retval;
    LOG_INFO("dplf config: %d", imu->dplf_config);
  }

  // Set power management register
  i2c_write_byte(imu->i2c, MPU6050_RA_PWR_MGMT_1, 0x00);

  // Configure gyro range
  int8_t gyro_range = 0;
  if (mpu6050_set_gyro_range(imu, config->gyro_range) != 0) {
    LOG_ERROR("Failed to set accel range!");
    return -1;
  }
  if (mpu6050_get_gyro_range(imu, &gyro_range) != 0) {
    LOG_ERROR("Failed to obtain accel range!");
    return -1;
  }
  switch (gyro_range) {
  case 0: imu->gyro_sensitivity = 131.0; break;
  case 1: imu->gyro_sensitivity = 65.5; break;
  case 2: imu->gyro_sensitivity = 32.8; break;
  case 3: imu->gyro_sensitivity = 16.4; break;
  default: LOG_ERROR("Invalid gyro range [%d]", gyro_range); goto error;
  }

  // Configure accel range
  int8_t accel_range;
  if (mpu6050_set_accel_range(imu, config->accel_range) != 0) {
    LOG_ERROR("Failed to set gyro range!");
    goto error;
  }
  if (mpu6050_get_accel_range(imu, &accel_range) != 0) {
    LOG_ERROR("Failed to obtain accel range!");
    goto error;
  }
  switch (accel_range) {
  case 0: imu->accel_sensitivity = 16384.0; break;
  case 1: imu->accel_sensitivity = 8192.0; break;
  case 2: imu->accel_sensitivity = 4096.0; break;
  case 3: imu->accel_sensitivity = 2048.0; break;
  default: LOG_ERROR("Invalid accel range [%d]", accel_range); return -3;
  }

  // Get sample rate
  imu->sample_rate = mpu6050_get_sample_rate(imu);

  return 0;
error:
  return -1;
}

int8_t mpu6050_ping(const mpu6050_t *imu) {
  uint8_t buf;
  i2c_set_slave(imu->i2c, MPU6050_ADDRESS);
  i2c_read_byte(imu->i2c, MPU6050_RA_WHO_AM_I, &buf);
  LOG_INFO("MPU6050 ADDRESS: 0x%02X\n", buf);
  return 0;
}

int8_t mpu6050_get_data(mpu6050_t *imu) {
  // Read data
  uint8_t raw_data[14];
  memset(raw_data, '\0', 14);
  i2c_set_slave(imu->i2c, MPU6050_ADDRESS);
  int8_t retval = i2c_read_bytes(imu->i2c, MPU6050_RA_ACCEL_XOUT_H, raw_data, 14);
  if (retval != 0) {
    return -1;
  }

  // Accelerometer
  const double g = 9.81; // Gravitational constant
  const int8_t raw_x = (raw_data[0] << 8) | (raw_data[1]);
  const int8_t raw_y = (raw_data[2] << 8) | (raw_data[3]);
  const int8_t raw_z = (raw_data[4] << 8) | (raw_data[5]);
  imu->accel[0] = (raw_x / imu->accel_sensitivity) * g;
  imu->accel[1] = (raw_y / imu->accel_sensitivity) * g;
  imu->accel[2] = (raw_z / imu->accel_sensitivity) * g;

  // Temperature
  const int8_t raw_temp = (raw_data[6] << 8) | (raw_data[7]);
  imu->temperature = raw_temp / 340.0 + 36.53;

  // Gyroscope
  const int8_t gyro_raw_x = (raw_data[8] << 8) | (raw_data[9]);
  const int8_t gyro_raw_y = (raw_data[10] << 8) | (raw_data[11]);
  const int8_t gyro_raw_z = (raw_data[12] << 8) | (raw_data[13]);
  imu->gyro[0] = deg2rad(gyro_raw_x / imu->gyro_sensitivity);
  imu->gyro[1] = deg2rad(gyro_raw_y / imu->gyro_sensitivity);
  imu->gyro[2] = deg2rad(gyro_raw_z / imu->gyro_sensitivity);

  // Set last_updated
  imu->last_updated = clock();

  return 0;
}

int8_t mpu6050_set_dplf(const mpu6050_t *imu, const uint8_t setting) {
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

  // Check setting range
  if (setting > 7 || setting < 0) {
    return -2;
  }

  // Set DPLF
  i2c_set_slave(imu->i2c, MPU6050_ADDRESS);
  int8_t retval = i2c_write_byte(imu->i2c, MPU6050_RA_CONFIG, (char) setting);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int8_t mpu6050_get_dplf(const mpu6050_t *imu) {
  // Get dplf config
  uint8_t data[1] = {0x00};
  i2c_set_slave(imu->i2c, MPU6050_ADDRESS);
  int8_t retval = i2c_read_bytes(imu->i2c, MPU6050_RA_CONFIG, data, 1);
  if (retval != 0) {
    return -1;
  }
  LOG_INFO("GOT DPLF: %d", data[0]);
  data[0] = data[0] & 0b00000111;

  return data[0];
}

int8_t mpu6050_set_sample_rate_div(const mpu6050_t *imu, const int8_t div) {
  i2c_set_slave(imu->i2c, MPU6050_ADDRESS);
  int8_t retval = i2c_write_byte(imu->i2c, MPU6050_RA_SMPLRT_DIV, div);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int8_t mpu6050_get_sample_rate_div(const mpu6050_t *imu) {
  i2c_set_slave(imu->i2c, MPU6050_ADDRESS);
  uint8_t data = 0;
  int8_t retval = i2c_read_byte(imu->i2c, MPU6050_RA_SMPLRT_DIV, &data);
  if (retval != 0) {
    return -1;
  }
  return data;
}

int8_t mpu6050_get_sample_rate(const mpu6050_t *imu) {
  // Get sample rate divider
  uint16_t sample_div = 0;
  int8_t rate_div = mpu6050_get_sample_rate_div(imu);
  if (rate_div != -1 || rate_div != -2) {
    sample_div = (float) rate_div;
  } else {
    return -1;
  }

  // Get gyro sample rate
  uint16_t gyro_rate = 0;
  uint8_t dlpf_cfg = mpu6050_get_sample_rate_div(imu);
  if (dlpf_cfg == 0 || dlpf_cfg == 7) {
    gyro_rate = 8000;
  } else if (dlpf_cfg >= 1 || dlpf_cfg <= 6) {
    gyro_rate = 1000;
  } else {
    return -2;
  }

  // Calculate sample rate
  return gyro_rate / (1 + sample_div);
}

int8_t mpu6050_set_gyro_range(const mpu6050_t *imu, const int8_t range) {
  // Pre-check
  if (range > 3 || range < 0) {
    return -2;
  }

  // Set sample rate
  uint8_t data = range << 3;
  i2c_set_slave(imu->i2c, MPU6050_ADDRESS);
  int8_t retval = i2c_write_byte(imu->i2c, MPU6050_RA_GYRO_CONFIG, data);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int8_t mpu6050_get_gyro_range(const mpu6050_t *imu, int8_t *range) {
  // Get gyro config
  uint8_t data = 0x00;
  i2c_set_slave(imu->i2c, MPU6050_ADDRESS);
  int8_t retval = i2c_read_byte(imu->i2c, MPU6050_RA_GYRO_CONFIG, &data);
  if (retval != 0) {
    return -1;
  }

  // Get gyro range bytes
  *range = (data >> 3) & 0b00000011;

  return 0;
}

int8_t mpu6050_set_accel_range(const mpu6050_t *imu, const int8_t range) {
  // Pre-check
  if (range > 3 || range < 0) {
    return -2;
  }

  // Set sample rate
  uint8_t data = range << 3;
  i2c_set_slave(imu->i2c, MPU6050_ADDRESS);
  int8_t retval = i2c_write_byte(imu->i2c, MPU6050_RA_ACCEL_CONFIG, data);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int8_t mpu6050_get_accel_range(const mpu6050_t *imu, int8_t *range) {
  // Get accel config
  uint8_t data = 0x00;
  i2c_set_slave(imu->i2c, MPU6050_ADDRESS);
  int8_t retval = i2c_read_byte(imu->i2c, MPU6050_RA_ACCEL_CONFIG, &data);
  if (retval != 0) {
    return -1;
  }

  // Get accel range bytes
  *range = (data >> 3) & 0b00000011;

  return 0;
}
