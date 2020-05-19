#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <assert.h>

#include <vector>
#include <string>

#include <Wire.h>
#include <Arduino.h>

/*******************************************************************************
 *                                  CORE
 ******************************************************************************/

#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

inline float deg2rad(const float d) { return d * (M_PI / 180.0); }

inline float rad2deg(const float r) { return r * (180.0 / M_PI); }

inline int8_t int8(const uint8_t *data, const size_t offset=0) {
  return (int8_t)(data[offset]);
}

inline uint8_t uint8(const uint8_t *data, const size_t offset=0) {
  return (uint8_t)(data[offset]);
}

inline int16_t int16(const uint8_t *data, const size_t offset=0) {
  return (int16_t)((data[offset + 1] << 8) | (data[offset]));
}

inline uint16_t uint16(const uint8_t *data, const size_t offset=0) {
  return (uint16_t)((data[offset + 1] << 8) | (data[offset]));
}

inline int32_t int32(const uint8_t *data, const size_t offset=0) {
	return (int32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
			(data[offset + 1] << 8) | (data[offset]));
}

inline uint32_t uint32(const uint8_t *data, const size_t offset=0) {
  return (uint32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
      (data[offset + 1] << 8) | (data[offset]));
}

void euler321(const double euler[3], double C[3 * 3]) {
  assert(euler != NULL);
  assert(C != NULL);

  const float phi = euler[0];
  const float theta = euler[1];
  const float psi = euler[2];

  /* 1st row */
  C[0] = cos(psi) * cos(theta);
  C[1] = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  C[2] = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
  /* 2nd row */
  C[3] = sin(psi) * cos(theta);
  C[4] = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  C[5] = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
  /* 3rd row */
  C[6] = -sin(theta);
  C[7] = cos(theta) * sin(phi);
  C[8] = cos(theta) * cos(phi);
}

void quat2euler(const float q[4], float euler[3]) {
  assert(q != NULL);
  assert(euler != NULL);

  const float qw = q[0];
  const float qx = q[1];
  const float qy = q[2];
  const float qz = q[3];

  const float qw2 = qw * qw;
  const float qx2 = qx * qx;
  const float qy2 = qy * qy;
  const float qz2 = qz * qz;

  const float t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  const float t2 = asin(2 * (qy * qw - qx * qz));
  const float t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  euler[0] = t1;
  euler[1] = t2;
  euler[2] = t3;
}

void rot2quat(const float C[3 * 3], float q[4]) {
  assert(C != NULL);
  assert(q != NULL);

  const float C00 = C[0];
  const float C01 = C[1];
  const float C02 = C[2];
  const float C10 = C[3];
  const float C11 = C[4];
  const float C12 = C[5];
  const float C20 = C[6];
  const float C21 = C[7];
  const float C22 = C[8];

  const float tr = C00 + C11 + C22;
  float S = 0.0f;
  float qw = 0.0f;
  float qx = 0.0f;
  float qy = 0.0f;
  float qz = 0.0f;

  if (tr > 0) {
    S = sqrt(tr + 1.0) * 2; // S=4*qw
    qw = 0.25 * S;
    qx = (C21 - C12) / S;
    qy = (C02 - C20) / S;
    qz = (C10 - C01) / S;
  } else if ((C00 > C11) && (C[0] > C22)) {
    S = sqrt(1.0 + C[0] - C11 - C22) * 2; // S=4*qx
    qw = (C21 - C12) / S;
    qx = 0.25 * S;
    qy = (C01 + C10) / S;
    qz = (C02 + C20) / S;
  } else if (C11 > C22) {
    S = sqrt(1.0 + C11 - C[0] - C22) * 2; // S=4*qy
    qw = (C02 - C20) / S;
    qx = (C01 + C10) / S;
    qy = 0.25 * S;
    qz = (C12 + C21) / S;
  } else {
    S = sqrt(1.0 + C22 - C[0] - C11) * 2; // S=4*qz
    qw = (C10 - C01) / S;
    qx = (C02 + C20) / S;
    qy = (C12 + C21) / S;
    qz = 0.25 * S;
  }

  q[0] = qw;
  q[1] = qx;
  q[2] = qy;
  q[3] = qz;
}

void tf_trans(const float T[16], float r[3]) {
  r[0] = T[3];
  r[1] = T[7];
  r[2] = T[11];
}

void tf_rot(const float T[16], float C[9]) {
  C[0] = T[0]; C[1] = T[1]; C[2] = T[2];
  C[3] = T[3]; C[4] = T[4]; C[5] = T[5];
  C[6] = T[6]; C[7] = T[7]; C[8] = T[8];
}

void tf_quat(const float T[4 * 4], float q[4]) {
  assert(T != NULL);
  assert(q != NULL);
  assert(T != q);

  float C[3 * 3] = {0};
  tf_rot(T, C);
  rot2quat(C, q);
}

/* void __assert(const char *__func, */
/*               const char *__file, */
/*               int __lineno, const char *__sexp) { */
/*   // transmit diagnostic informations through serial link. */
/*   Serial.println(__func); */
/*   Serial.println(__file); */
/*   Serial.println(__lineno, DEC); */
/*   Serial.println(__sexp); */
/*   Serial.flush(); */
/*   // abort program execution. */
/*   abort(); */
/* } */

/*******************************************************************************
 *                                 SERIAL
 ******************************************************************************/

class serial_t {
public:
  Stream *serial_in;
  Print *serial_out;

  serial_t() {
    serial_in = &Serial;
    serial_out = &Serial;
    Serial.begin(115200);
  }

  void read_byte(uint8_t &b) {
    b = serial_in->read();
  }

  void read_bytes(uint8_t *data, const size_t len) {
    serial_in->readBytes(data, len);
  }

  void write_byte(const uint8_t b) {
    serial_out->write(b);
  }

  void write_bytes(const uint8_t *data, const size_t len) {
    serial_out->write(data, len);
  }

  /**
  * Simple printf for writing to an Arduino serial port.  Allows specifying
  * Serial..Serial3.
  *
  * const HardwareSerial&, the serial port to use (Serial..Serial3)
  * const char* fmt, the formatting string followed by the data to be formatted
  *
  *     int d = 65;
  *     float f = 123.4567;
  *     char* str = "Hello";
  *     serial_t::printf(Serial, "<fmt>", d);
  *
  * Example:
  *   serial_t::printf(Serial, "Sensor %d is %o and reads %1f\n", d, d, f) will
  *   output "Sensor 65 is on and reads 123.5" to the serial port.
  *
  * Formatting strings <fmt>
  * %B    - binary (d = 0b1000001)
  * %b    - binary (d = 1000001)
  * %c    - character (s = H)
  * %d/%i - integer (d = 65)\
  * %f    - float (f = 123.45)
  * %3f   - float (f = 123.346) three decimal places specified by %3.
  * %s    - char* string (s = Hello)
  * %X    - hexidecimal (d = 0x41)
  * %x    - hexidecimal (d = 41)
  * %%    - escaped percent ("%")
  **/
  void printf(const char* fmt, ...) {
    va_list argv;
    va_start(argv, fmt);

    for (int i = 0; fmt[i] != '\0'; i++) {
      if (fmt[i] == '%') {
        // Look for specification of number of decimal places
        int places = 2;
        if (fmt[i+1] >= '0' && fmt[i+1] <= '9') {
          places = fmt[i+1] - '0';
          i++;
        }

        switch (fmt[++i]) {
          case 'B': serial_out->print("0b");
          case 'b': serial_out->print(va_arg(argv, int), BIN); break;
          case 'c': serial_out->print((char) va_arg(argv, int)); break;
          case 'd':
          case 'i': serial_out->print(va_arg(argv, int), DEC); break;
          case 'f': serial_out->print(va_arg(argv, double), places); break;
          case 'l': serial_out->print(va_arg(argv, long), DEC); break;
          case 's': serial_out->print(va_arg(argv, const char*)); break;
          case 'X':
            serial_out->print("0x");
            serial_out->print(va_arg(argv, int), HEX);
            break;
          case '%': serial_out->print(fmt[i]); break;
          default: serial_out->print("?"); break;
        }
      } else {
        serial_out->print(fmt[i]);
      }
    }
    va_end(argv);
  }
};

/*******************************************************************************
 *                                  GPIO
 ******************************************************************************/

class gpio_t {
public:
  gpio_t() {}

  void set_pin(const int pin, const int mode) { pinMode(pin, mode); }

  void digital_write(const int pin, const int mode) { digitalWrite(pin, mode); }
  int digital_read(const int pin) { return digitalRead(pin); }

  void analog_write(const int pin, const int mode) { analogWrite(pin, mode); }
  int analog_read(const int pin) { return analogRead(pin); }

  void delay(const int ms) { delay(ms); }
  void delay_us(const int us) { delayMicroseconds(us); }

  int pulse_in(const int pin, const int val, const int timeout_us=1000000) {
    return pulseIn(pin, val, timeout_us);
  }

  unsigned long pulse_in_long(const int pin,
                              const int val,
                              const int timeout_us=1000000) {
    return pulseIn(pin, val, timeout_us);
  }
};

/*******************************************************************************
 *                                   I2C
 ******************************************************************************/

/* DEFINES */
#define I2C_BUF_MAX 1024

class i2c_t {
public:

  i2c_t() {
    Wire.begin();
  }

  uint8_t read_byte(const uint8_t dev_addr, const uint8_t reg_addr) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.endTransmission();

    uint8_t size = 1;
    uint8_t last = 1;
    Wire.requestFrom(dev_addr, size, last);
    const uint8_t data = Wire.read();

    return data;
  }

  void read_bytes(const uint8_t dev_addr,
                  const uint8_t reg_addr,
                  const size_t length,
                  uint8_t *data) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.endTransmission();

    Wire.requestFrom(dev_addr, length);
    for (size_t i = 0; i < length; i++) {
      data[i] = Wire.read();
    }
  }

	uint8_t read_u8(const uint8_t dev_addr, const uint8_t reg_addr) {
		return read_byte(dev_addr, reg_addr);
	}

	int8_t read_s8(const uint8_t dev_addr, const uint8_t reg_addr) {
		return read_byte(dev_addr, reg_addr);
	}

	uint16_t read_u16(const uint8_t dev_addr, const uint8_t reg_addr) {
		uint8_t bytes[2] = {0};
		read_bytes(dev_addr, reg_addr, 2, bytes);
    return uint16(bytes);
	}

	int16_t read_s16(const uint8_t dev_addr, const uint8_t reg_addr) {
		return read_u16(dev_addr, reg_addr);
	}

	uint32_t read_u32(const uint8_t dev_addr, const uint8_t reg_addr) {
		uint8_t bytes[4] = {0};
		read_bytes(dev_addr, reg_addr, 4, bytes);
    return uint32(bytes);
	}

	int32_t read_s32(const uint8_t dev_addr, const uint8_t reg_addr) {
		return read_u32(dev_addr, reg_addr);
	}

  void write_byte(const uint8_t dev_addr,
                  const uint8_t reg_addr,
                  const uint8_t value) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.write(value);
    Wire.endTransmission();
  }

  void write_bytes(const uint8_t dev_addr,
                   const uint8_t reg_addr,
                   const uint8_t *data,
                   const size_t length) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.write(data, length);
    Wire.endTransmission();
  }

	void scan_addrs(uint8_t *addrs, uint8_t *nb_addrs) {
		*nb_addrs = 0;

		for (uint8_t addr = 1; addr < 127; addr++) {
			Wire.beginTransmission(addr);
			if (Wire.endTransmission() == 0) {
				addrs[*nb_addrs] = addr;
				*nb_addrs += 1;
			}
		}
	}

	void scan_i2c_addrs(serial_t &serial) {
		uint8_t addrs[128] = {0};
		uint8_t nb_addrs;
		scan_addrs(addrs, &nb_addrs);

		serial.printf("scanning i2c devices:\n\r");
		serial.printf("nb_addrs: %d\n\r", nb_addrs);
		for (uint8_t i = 0; i < nb_addrs; i++) {
			serial.printf("%X\n\r", addrs[i]);
		}

		delay(5000);
	}
};

/*******************************************************************************
 *                                    PWM
 ******************************************************************************/

class pwm_t {
public:
  uint8_t pin = 0;
  uint8_t freq = 0;
  uint32_t channel = 0;
  /* HardwareTimer *timer = nullptr; */

  pwm_t() {}

  pwm_t(const uint8_t pin_, const uint8_t freq_)
    : pin{pin_}, freq{freq_} {}

  void setup() {
    /* channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM)); */
    /* TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM); */
    /* timer = new HardwareTimer(Instance); */
    /* timer->setPWM(channel, pin, freq, 50); */
  }

  void set(const uint8_t duty_cycle) {
    /* timer->pause(); */
    /* timer->setPWM(channel, pin, freq, duty_cycle); */
    /* timer->refresh(); */
    /* timer->resume(); */
  }
};

/*******************************************************************************
 *                                  HC-SR04
 ******************************************************************************/

class hcsr04_t {
public:
  gpio_t gpio;
  uint8_t trig_pin;
  uint8_t echo_pin;

  hcsr04_t(gpio_t &gpio_, const uint8_t trig_pin_, const uint8_t echo_pin_)
    : gpio{gpio_} {
    gpio.set_pin(trig_pin_, OUTPUT);
    gpio.set_pin(echo_pin_, INPUT);
  }

  float measure() {
    // Makesure pin is low
    gpio.digital_write(trig_pin, LOW);
    gpio.delay_us(2);

    // Hold trigger for 10 microseconds
    gpio.digital_write(trig_pin, HIGH);
    gpio.delay_us(10);
    gpio.digital_write(trig_pin, LOW);

    // Measure echo signal
    const float duration_ms = gpio.pulse_in(echo_pin, HIGH, 1000);
    const float temperature = 20.0;
    double sof_cm = 0.03313 + 0.0000606 * temperature;
    double dist_cm = duration_ms / 2.0 * sof_cm;
    if (dist_cm == 0 || dist_cm > 400) {
      return -1.0 ;
    } else {
      return dist_cm;
    }
  }
};

/*******************************************************************************
 *                                  MPU-6050
 ******************************************************************************/

class mpu6050_t {
public:
	// I2C ADDRESS
	enum i2c_addrs {
		MPU6050_ADDRESS = 0x68,
		MPU6050_ADDRESS_AD0_LOW = 0x68,  // addr pin low (GND) [default]
		MPU6050_ADDRESS_AD0_HIGH = 0x69  // addr pin high (VCC)
	};

	// REGISTERS
	enum registers {
		MPU6050_REG_XG_OFFS_TC = 0x00,
		MPU6050_REG_YG_OFFS_TC = 0x01,
		MPU6050_REG_ZG_OFFS_TC = 0x02,
		MPU6050_REG_X_FINE_GAIN = 0x03,
		MPU6050_REG_Y_FINE_GAIN = 0x04,
		MPU6050_REG_Z_FINE_GAIN = 0x05,
		MPU6050_REG_XA_OFFS_H = 0x06,
		MPU6050_REG_XA_OFFS_L_TC = 0x07,
		MPU6050_REG_YA_OFFS_H = 0x08,
		MPU6050_REG_YA_OFFS_L_TC = 0x09,
		MPU6050_REG_ZA_OFFS_H = 0x0A,
		MPU6050_REG_ZA_OFFS_L_TC = 0x0B,
		MPU6050_REG_XG_OFFS_USRH = 0x13,
		MPU6050_REG_XG_OFFS_USRL = 0x14,
		MPU6050_REG_YG_OFFS_USRH = 0x15,
		MPU6050_REG_YG_OFFS_USRL = 0x16,
		MPU6050_REG_ZG_OFFS_USRH = 0x17,
		MPU6050_REG_ZG_OFFS_USRL = 0x18,
		MPU6050_REG_SMPLRT_DIV = 0x19,
		MPU6050_REG_CONFIG = 0x1A,
		MPU6050_REG_GYRO_CONFIG = 0x1B,
		MPU6050_REG_ACCEL_CONFIG = 0x1C,
		MPU6050_REG_FF_THR = 0x1D,
		MPU6050_REG_FF_DUR = 0x1E,
		MPU6050_REG_MOT_THR = 0x1F,
		MPU6050_REG_MOT_DUR = 0x20,
		MPU6050_REG_ZRMOT_THR = 0x21,
		MPU6050_REG_ZRMOT_DUR = 0x22,
		MPU6050_REG_FIFO_EN = 0x23,
		MPU6050_REG_I2C_MST_CTRL = 0x24,
		MPU6050_REG_I2C_SLV0_ADDR = 0x25,
		MPU6050_REG_I2C_SLV0_REG = 0x26,
		MPU6050_REG_I2C_SLV0_CTRL = 0x27,
		MPU6050_REG_I2C_SLV1_ADDR = 0x28,
		MPU6050_REG_I2C_SLV1_REG = 0x29,
		MPU6050_REG_I2C_SLV1_CTRL = 0x2A,
		MPU6050_REG_I2C_SLV2_ADDR = 0x2B,
		MPU6050_REG_I2C_SLV2_REG = 0x2C,
		MPU6050_REG_I2C_SLV2_CTRL = 0x2D,
		MPU6050_REG_I2C_SLV3_ADDR = 0x2E,
		MPU6050_REG_I2C_SLV3_REG = 0x2F,
		MPU6050_REG_I2C_SLV3_CTRL = 0x30,
		MPU6050_REG_I2C_SLV4_ADDR = 0x31,
		MPU6050_REG_I2C_SLV4_REG = 0x32,
		MPU6050_REG_I2C_SLV4_DO = 0x33,
		MPU6050_REG_I2C_SLV4_CTRL = 0x34,
		MPU6050_REG_I2C_SLV4_DI = 0x35,
		MPU6050_REG_I2C_MST_STATUS = 0x36,
		MPU6050_REG_INT_PIN_CFG = 0x37,
		MPU6050_REG_INT_ENABLE = 0x38,
		MPU6050_REG_DMP_INT_STATUS = 0x39,
		MPU6050_REG_INT_STATUS = 0x3A,
		MPU6050_REG_ACCEL_XOUT_H = 0x3B,
		MPU6050_REG_ACCEL_XOUT_L = 0x3C,
		MPU6050_REG_ACCEL_YOUT_H = 0x3D,
		MPU6050_REG_ACCEL_YOUT_L = 0x3E,
		MPU6050_REG_ACCEL_ZOUT_H = 0x3F,
		MPU6050_REG_ACCEL_ZOUT_L = 0x40,
		MPU6050_REG_TEMP_OUT_H = 0x41,
		MPU6050_REG_TEMP_OUT_L = 0x42,
		MPU6050_REG_GYRO_XOUT_H = 0x43,
		MPU6050_REG_GYRO_XOUT_L = 0x44,
		MPU6050_REG_GYRO_YOUT_H = 0x45,
		MPU6050_REG_GYRO_YOUT_L = 0x46,
		MPU6050_REG_GYRO_ZOUT_H = 0x47,
		MPU6050_REG_GYRO_ZOUT_L = 0x48,
		MPU6050_REG_EXT_SENS_DATA_00 = 0x49,
		MPU6050_REG_EXT_SENS_DATA_01 = 0x4A,
		MPU6050_REG_EXT_SENS_DATA_02 = 0x4B,
		MPU6050_REG_EXT_SENS_DATA_03 = 0x4C,
		MPU6050_REG_EXT_SENS_DATA_04 = 0x4D,
		MPU6050_REG_EXT_SENS_DATA_05 = 0x4E,
		MPU6050_REG_EXT_SENS_DATA_06 = 0x4F,
		MPU6050_REG_EXT_SENS_DATA_07 = 0x50,
		MPU6050_REG_EXT_SENS_DATA_08 = 0x51,
		MPU6050_REG_EXT_SENS_DATA_09 = 0x52,
		MPU6050_REG_EXT_SENS_DATA_10 = 0x53,
		MPU6050_REG_EXT_SENS_DATA_11 = 0x54,
		MPU6050_REG_EXT_SENS_DATA_12 = 0x55,
		MPU6050_REG_EXT_SENS_DATA_13 = 0x56,
		MPU6050_REG_EXT_SENS_DATA_14 = 0x57,
		MPU6050_REG_EXT_SENS_DATA_15 = 0x58,
		MPU6050_REG_EXT_SENS_DATA_16 = 0x59,
		MPU6050_REG_EXT_SENS_DATA_17 = 0x5A,
		MPU6050_REG_EXT_SENS_DATA_18 = 0x5B,
		MPU6050_REG_EXT_SENS_DATA_19 = 0x5C,
		MPU6050_REG_EXT_SENS_DATA_20 = 0x5D,
		MPU6050_REG_EXT_SENS_DATA_21 = 0x5E,
		MPU6050_REG_EXT_SENS_DATA_22 = 0x5F,
		MPU6050_REG_EXT_SENS_DATA_23 = 0x60,
		MPU6050_REG_MOT_DETECT_STATUS = 0x61,
		MPU6050_REG_I2C_SLV0_DO = 0x63,
		MPU6050_REG_I2C_SLV1_DO = 0x64,
		MPU6050_REG_I2C_SLV2_DO = 0x65,
		MPU6050_REG_I2C_SLV3_DO = 0x66,
		MPU6050_REG_I2C_MST_DELAY_CTRL = 0x67,
		MPU6050_REG_SIGNAL_PATH_RESET = 0x68,
		MPU6050_REG_MOT_DETECT_CTRL = 0x69,
		MPU6050_REG_USER_CTRL = 0x6A,
		MPU6050_REG_PWR_MGMT_1 = 0x6B,
		MPU6050_REG_PWR_MGMT_2 = 0x6C,
		MPU6050_REG_BANK_SEL = 0x6D,
		MPU6050_REG_MEM_START_ADDR = 0x6E,
		MPU6050_REG_MEM_R_W = 0x6F,
		MPU6050_REG_DMP_CFG_1 = 0x70,
		MPU6050_REG_DMP_CFG_2 = 0x71,
		MPU6050_REG_FIFO_COUNTH = 0x72,
		MPU6050_REG_FIFO_COUNTL = 0x73,
		MPU6050_REG_FIFO_R_W = 0x74,
		MPU6050_REG_WHO_AM_I = 0x75
	};

	// Interface
  i2c_t &i2c;

	// State
  int8_t ok = 0;
  float temperature = 0.0f;

	// Settings
  float accel_sensitivity = 0.0f;
  float gyro_sensitivity = 0.0f;
  float accel[3] = {0.0f};
  float gyro[3] = {0.0f};
  float sample_rate = 0.0f;

  mpu6050_t(i2c_t &i2c_) : i2c{i2c_} {
    // Configure Digital low-pass filter
    uint8_t dlpf_cfg = 0;
    set_dplf(dlpf_cfg);

    // Set power management register
    i2c.write_byte(MPU6050_ADDRESS, MPU6050_REG_PWR_MGMT_1, 0x00);

    // Configure Gyroscope range
    const uint8_t gyro_range = 0;
    set_gyro_range(gyro_range);
    switch (gyro_range) {
    case 0: gyro_sensitivity = 131.0; break;
    case 1: gyro_sensitivity = 65.5; break;
    case 2: gyro_sensitivity = 32.8; break;
    case 3: gyro_sensitivity = 16.4; break;
    }

    // Configure accel range
    const uint8_t accel_range = 0;
    set_accel_range(accel_range);
    switch (accel_range) {
    case 0: accel_sensitivity = 16384.0; break;
    case 1: accel_sensitivity = 8192.0; break;
    case 2: accel_sensitivity = 4096.0; break;
    case 3: accel_sensitivity = 2048.0; break;
    }

    // Configure sample rate
    //set_sample_rate(sample_rate);
    sample_rate = get_sample_rate();
  }

  /** Get MPU6050 address */
  int8_t ping() {
    return i2c.read_byte(MPU6050_ADDRESS, MPU6050_REG_WHO_AM_I);
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
  uint8_t get_dplf() {
    uint8_t data = i2c.read_byte(MPU6050_ADDRESS, MPU6050_REG_CONFIG);
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
  void set_dplf(const uint8_t setting) {
    i2c.write_byte(MPU6050_ADDRESS, MPU6050_REG_CONFIG, setting);
  }

  /** Get sample rate division */
  uint8_t get_sample_rate_div() {
    return i2c.read_byte(MPU6050_ADDRESS, MPU6050_REG_SMPLRT_DIV);
  }

  /** Set sample rate */
  void set_sample_rate_div(const int8_t div) {
    i2c.write_byte(MPU6050_ADDRESS, MPU6050_REG_SMPLRT_DIV, div);
  }

  /** Set sample rate division */
  void set_sample_rate(float sample_rate) {
    // gyro_output_rate = 8kHz (iff DPLF_CFG: 0 or 7) or 1kHz
    uint8_t dlpf_cfg = get_dplf();
    float gyro_rate = ((dlpf_cfg == 0) || (dlpf_cfg == 7)) ? 8000.0f : 1000.0f;

    // Calculate and set sample rate divider needed to get desired sample rate.
    // The equation is given in MPU6050 register map documentation (page 12 of
    // 46). Under the sample rate divider register section.
    //
    // sample_rate = gyro_output_rate / (1 + smplrt_div)
    // smplrt_div = (gyro_output_rate / sample_rate) - 1
    float smplrt_div = (gyro_rate / sample_rate) - 1;
    set_sample_rate_div(smplrt_div);
  }

  /** Get sample rate */
  float get_sample_rate() {
    // Get sample rate divider
    uint8_t smplrt_div = get_sample_rate_div();

    // Get gyro sample rate
    uint8_t dlpf_cfg = get_dplf();
    float gyro_rate = ((dlpf_cfg == 0) || (dlpf_cfg == 7)) ? 8000.0f : 1000.0f;

    // Calculate sample rate. The equation is given in MPU6050 register map
    // documentation (page 12 of 46). Under the sample rate divider register
    // section.
    return gyro_rate / (1 + smplrt_div);
  }

  /** Set gyro range */
  void set_gyro_range(const int8_t range) {
    uint8_t data = range << 3;
    i2c.write_byte(MPU6050_ADDRESS, MPU6050_REG_GYRO_CONFIG, data);
  }

  /** Get gyro range */
  uint8_t get_gyro_range() {
    uint8_t data = i2c.read_byte(MPU6050_ADDRESS, MPU6050_REG_GYRO_CONFIG);
    data = (data >> 3) & 0b00000011;
    return data;
  }

  /** Set accelerometer range */
  void set_accel_range(const int8_t range) {
    // assert(range > 3 || range < 0);
    uint8_t data = range << 3;
    i2c.write_byte(MPU6050_ADDRESS, MPU6050_REG_ACCEL_CONFIG, data);
  }

  /** Get accelerometer range */
  uint8_t get_accel_range() {
    uint8_t data = i2c.read_byte(MPU6050_ADDRESS, MPU6050_REG_ACCEL_CONFIG);
    data  = (data >> 3) & 0b00000011;
    return data;
  }

  /** Get IMU data */
  void get_data() {
    // Read data
    uint8_t data[14] = {0};
    memset(data, '\0', 14);
    i2c.read_bytes(MPU6050_ADDRESS, MPU6050_REG_ACCEL_XOUT_H, 14, data);

    // Accelerometer
    const double g = 9.81; // Gravitational constant
    const int16_t raw_ax = (data[0] << 8) | data[1];
    const int16_t raw_ay = (data[2] << 8) | data[3];
    const int16_t raw_az = (data[4] << 8) | data[5];
    accel[0] = (raw_ax / accel_sensitivity) * g;
    accel[1] = (raw_ay / accel_sensitivity) * g;
    accel[2] = (raw_az / accel_sensitivity) * g;

    // Temperature
    const int16_t raw_temp = (data[6] << 8) | (data[7]);
    temperature = raw_temp / 340.0 + 36.53;

    // Gyroscope
    const int16_t raw_gx = (data[8] << 8) | (data[9]);
    const int16_t raw_gy = (data[10] << 8) | (data[11]);
    const int16_t raw_gz = (data[12] << 8) | (data[13]);
    gyro[0] = deg2rad(raw_gx / gyro_sensitivity);
    gyro[1] = deg2rad(raw_gy / gyro_sensitivity);
    gyro[2] = deg2rad(raw_gz / gyro_sensitivity);

    // Set last_updated
    // last_updated = clock();
  }

  void print_config(serial_t &serial) {
    serial.printf("Ping: %d\n\r", ping());
    serial.printf("DPLF: %d\n\r", get_dplf());
    serial.printf("Accel sensitivity: %f\n\r", accel_sensitivity);
    serial.printf("Gyro sensitivity: %f\n\r", gyro_sensitivity);
    serial.printf("Sample rate div: %f\n\r", get_sample_rate_div());
    serial.printf("Sample rate: %f\n\r", sample_rate);
  }
};

/*******************************************************************************
 *                                  BMP280
 ******************************************************************************/

class bmp280_t {
public:
	// BMP280 I2C Addresses
	enum i2c_addrs {
		BMP280_ADDR = 0x77,     // Primary I2C Address
		BMP280_ADDR_ALT = 0x76  // Alternate Address
	};

	// BMP280 Registers
	enum registers {
		BMP280_REG_DIG_T1 = 0x88,
		BMP280_REG_DIG_T2 = 0x8A,
		BMP280_REG_DIG_T3 = 0x8C,
		BMP280_REG_DIG_P1 = 0x8E,
		BMP280_REG_DIG_P2 = 0x90,
		BMP280_REG_DIG_P3 = 0x92,
		BMP280_REG_DIG_P4 = 0x94,
		BMP280_REG_DIG_P5 = 0x96,
		BMP280_REG_DIG_P6 = 0x98,
		BMP280_REG_DIG_P7 = 0x9A,
		BMP280_REG_DIG_P8 = 0x9C,
		BMP280_REG_DIG_P9 = 0x9E,
		BMP280_REG_DIG_H1 = 0xA1,
		BMP280_REG_DIG_H2 = 0xE1,
		BMP280_REG_DIG_H3 = 0xE3,
		BMP280_REG_DIG_H4 = 0xE4,
		BMP280_REG_DIG_H5 = 0xE5,
		BMP280_REG_DIG_H6 = 0xE7,
		BMP280_REG_CHIPID = 0xD0,
		BMP280_REG_VERSION = 0xD1,
		BMP280_REG_SOFTRESET = 0xE0,
		BMP280_REG_CAL26 = 0xE1,    // R calibration stored in 0xE1-0xF0
		BMP280_REG_CONTROLHUMID = 0xF2,
		BMP280_REG_STATUS = 0XF3,
		BMP280_REG_CONTROL = 0xF4,
		BMP280_REG_CONFIG = 0xF5,
		BMP280_REG_PRESSUREDATA = 0xF7,
		BMP280_REG_TEMPDATA = 0xFA,
		BMP280_REG_HUMIDDATA = 0xFD
	};

	// Sampling rates
	enum sampling_rates {
		BMP280_SAMPLING_NONE = 0b000,
		BMP280_SAMPLING_X1 = 0b001,
		BMP280_SAMPLING_X2 = 0b010,
		BMP280_SAMPLING_X4 = 0b011,
		BMP280_SAMPLING_X8 = 0b100,
		BMP280_SAMPLING_X16 = 0b101
	};

	// Power modes
	enum power_modes {
		BMP280_MODE_SLEEP = 0b00,
		BMP280_MODE_FORCED = 0b01,
		BMP280_MODE_NORMAL = 0b11
	};

	// Filter values
	enum filter_values {
		BMP280_FILTER_OFF = 0b000,
		BMP280_FILTER_X2 = 0b001,
		BMP280_FILTER_X4 = 0b010,
		BMP280_FILTER_X8 = 0b011,
		BMP280_FILTER_X16 = 0b10
	};

	// Standby duration in ms
	enum standby_duration{
		BMP280_STANDBY_MS_0_5 = 0b000,
		BMP280_STANDBY_MS_62_5 = 0b001,
		BMP280_STANDBY_MS_125 = 0b010,
		BMP280_STANDBY_MS_250 = 0b011,
		BMP280_STANDBY_MS_500 = 0b100,
		BMP280_STANDBY_MS_1000 = 0b101,
		BMP280_STANDBY_MS_2000 = 0b110,
		BMP280_STANDBY_MS_4000 = 0b111
	};

  // BMP280 Configuration
  struct bmp280_config {
    uint8_t t_sb = BMP280_STANDBY_MS_0_5;  // Standby time in normal mode
    uint8_t filter = BMP280_FILTER_X16;    // Filter settings
    uint8_t spi3w_en = 0; 							   // Enable 3-wire SPI

    uint8_t get() { return (t_sb << 5) | (filter << 2) | spi3w_en; }
  } config;

  // BMP280 Control Measurements
  struct bmp280_ctrl_meas {
    uint8_t mode = BMP280_MODE_NORMAL; 		 // Device mode
    uint8_t osrs_p = BMP280_SAMPLING_X16;  // Pressure oversampling
    uint8_t osrs_t = BMP280_SAMPLING_X2;   // Temperature oversampling

    uint8_t get() { return (osrs_t << 5) | (osrs_p << 2) | mode; }
  } ctrl_meas;

	// BMP280 Calibration Data
	struct bmp280_calib_data_t {
		// Temperature compensation value
		uint16_t dig_T1 = 0;
		int16_t dig_T2 = 0;
		int16_t dig_T3 = 0;

		// Pressure compenstation value
		uint16_t dig_P1 = 0;
		int16_t dig_P2 = 0;
		int16_t dig_P3 = 0;
		int16_t dig_P4 = 0;
		int16_t dig_P5 = 0;
		int16_t dig_P6 = 0;
		int16_t dig_P7 = 0;
		int16_t dig_P8 = 0;
		int16_t dig_P9 = 0;
	} calib_data;

	// Interfaces
	uint8_t sensor_addr = BMP280_ADDR_ALT;
	i2c_t &i2c;
	serial_t &serial;

	// State
	bool connected = false;
	uint8_t previous_measuring = 0;

  bmp280_t(i2c_t &i2c_, serial_t &serial_) : i2c{i2c_}, serial{serial_} {
		// Verify chip id
		uint8_t sensor_id = i2c.read_byte(sensor_addr, BMP280_REG_CHIPID);
		if (sensor_id == 0x58) {
			connected = true;
		}

		// Reset the device using soft-reset
		i2c.write_byte(sensor_addr, BMP280_REG_SOFTRESET, 0xB6);

		// Wait for chip to wake up.
		delay(10);

		// Get calibration data
		get_calib_data();

		// Configure sensor
		i2c.write_byte(sensor_addr, BMP280_REG_CONFIG, config.get());
		i2c.write_byte(sensor_addr, BMP280_REG_CONTROL, ctrl_meas.get());

		// Wait
		delay(100);
	}

	bool data_ready() {
		uint8_t reg = i2c.read_byte(sensor_addr, BMP280_REG_STATUS);

		uint8_t measuring = (reg & 0b00001000) >> 3;
		if (measuring ^ previous_measuring) {  // Has measuring bit flipped?
			previous_measuring = measuring;
			if (measuring == 0) {
				return true;
			}
		}

		return false;
	}

	void get_calib_data() {
		calib_data.dig_T1 = i2c.read_u16(sensor_addr, BMP280_REG_DIG_T1);
		calib_data.dig_T2 = i2c.read_s16(sensor_addr, BMP280_REG_DIG_T2);
		calib_data.dig_T3 = i2c.read_s16(sensor_addr, BMP280_REG_DIG_T3);

		calib_data.dig_P1 = i2c.read_u16(sensor_addr, BMP280_REG_DIG_P1);
		calib_data.dig_P2 = i2c.read_s16(sensor_addr, BMP280_REG_DIG_P2);
		calib_data.dig_P3 = i2c.read_s16(sensor_addr, BMP280_REG_DIG_P3);
		calib_data.dig_P4 = i2c.read_s16(sensor_addr, BMP280_REG_DIG_P4);
		calib_data.dig_P5 = i2c.read_s16(sensor_addr, BMP280_REG_DIG_P5);
		calib_data.dig_P6 = i2c.read_s16(sensor_addr, BMP280_REG_DIG_P6);
		calib_data.dig_P7 = i2c.read_s16(sensor_addr, BMP280_REG_DIG_P7);
		calib_data.dig_P8 = i2c.read_s16(sensor_addr, BMP280_REG_DIG_P8);
		calib_data.dig_P9 = i2c.read_s16(sensor_addr, BMP280_REG_DIG_P9);
	}

	/** Returns temperature in Celcius degrees and pressure in Pa **/
	void get_data(float &temperature, float &pressure) {
		// Get data
		uint8_t data[6] = {0};
		i2c.read_bytes(sensor_addr, BMP280_REG_PRESSUREDATA, 6, data);

		// Compensate for temperature
		int32_t t_fine = 0;
		{
			int32_t T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
			int32_t T1 = calib_data.dig_T1;
			int32_t T2 = calib_data.dig_T2;
			int32_t T3 = calib_data.dig_T3;
			int32_t var1 = ((((T >> 3) - (T1 << 1))) * T2) >> 11;
			int32_t var2 = (((((T >> 4) - T1) * ((T >> 4) - T1)) >> 12) * (T3)) >> 14;
			t_fine = var1 + var2;
			temperature = ((t_fine * 5 + 128) >> 8) / 100.0f;
		}

		// Compensate for pressure
		{
			int32_t P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
			int64_t P1 = calib_data.dig_P1;
			int64_t P2 = calib_data.dig_P2;
			int64_t P3 = calib_data.dig_P3;
			int64_t P4 = calib_data.dig_P4;
			int64_t P5 = calib_data.dig_P5;
			int64_t P6 = calib_data.dig_P6;
			int64_t P7 = calib_data.dig_P7;
			int64_t P8 = calib_data.dig_P8;
			int64_t P9 = calib_data.dig_P9;

			int64_t var1, var2 = 0;
			var1 = ((int64_t) t_fine) - 128000;
			var2 = var1 * var1 * P6;
			var2 = var2 + ((var1 * P5) << 17);
			var2 = var2 + (P4 << 35);
			var1 = ((var1 * var1 * P3) >> 8) + ((var1 * P2) << 12);
			var1 = (((((int64_t)1) << 47) + var1)) * P1 >> 33;
			if (var1 == 0) {
				pressure = -1; // avoid exception caused by division by zero
				return;
			}

			int64_t p = 1048576 - P;
			p = (((p << 31) - var2) * 3125) / var1;
			var1 = (P9 * (p >> 13) * (p >> 13)) >> 25;
			var2 = (P8 * p) >> 19;
			pressure = ((float) (((p + var1 + var2) >> 8) + (P7 << 4))) / 256.0f;
		}
	}
};

/*******************************************************************************
 *                                   SBUS
 ******************************************************************************/

class sbus_t {
public:
  sbus_t() {}
};

/*******************************************************************************
 *                                  UBLOX
 ******************************************************************************/

/**
 * UBX Class IDs
 * -------------
 * NAV 0x01 Navigation Results: Position, Speed, Time, Acceleration, Heading,
 * DOP, SVs used RXM 0x02 Receiver Manager: Satellite Status, RTC Status INF
 * 0x04 Information: Printf-Style Messages, with IDs such as Error, Warning,
 * Notice ACK 0x05 Ack/Nak: Acknowledge or Reject messages to UBX-CFG input
 * messages CFG 0x06 Configuration Input: Set Dynamic Model, Set DOP Mask, Set
 * Baud Rate, etc. UPD 0x09 Firmware Update: Memory/Flash erase/write, Reboot,
 * Flash identification, etc. MON 0x0A Monitoring: Communication Status, CPU
 * Load, Stack Usage, Task Status TIM 0x0D Timing: Time Pulse Output, Time Mark
 * Results MGA 0x13 Multiple GNSS Assistance: Assistance data for various GNSS
 * LOG 0x21 Logging: Log creation, deletion, info and retrieval
 * SEC 0x27 Security Feature
 */
#define UBX_NAV 0x01
#define UBX_RXM 0x02
#define UBX_INF 0x04
#define UBX_ACK 0x05
#define UBX_CFG 0x06
#define UBX_UPD 0x09
#define UBX_MON 0x0A
#define UBX_TIM 0x0D
#define UBX_MGA 0x13
#define UBX_LOG 0x21
#define UBX_SEC 0x27

/**
 * UBX Class CFG
 * -------------
 * ACK-ACK 0x05 0x01 2 Output Message Acknowledged
 * ACK-NAK 0x05 0x00 2 Output Message Not-Acknowledged
 */
#define UBX_ACK_ACK 0x01
#define UBX_ACK_NAK 0x00

/**
 * UBX Class CFG
 * -------------
 * CFG-VALDEL 0x06 0x8C 4 + 4*N Set Deletes values corresponding to...
 * CFG-VALGET 0x06 0x8B 4 + 4*N Poll Request Get Configuration Items
 * CFG-VALSET 0x06 0x8A 4 + 1*N Set Sets values corresponding to provided...
 */
#define UBX_CFG_VALDEL 0x8C
#define UBX_CFG_VALGET 0x8B
#define UBX_CFG_VALSET 0x8A

/**
 * UBX Class MON Monitoring Messages
 * ---------------------------------
 * MON-COMMS 0x0A 0x36 8 + 40*nPorts Periodic/Polled Comm port information
 * MON-GNSS 0x0A 0x28 8 Polled Information message major GNSS...
 * MON-HW2 0x0A 0x0B 28 Periodic/Polled Extended Hardware Status
 * MON-HW3 0x0A 0x37 22 + 6*nPins Periodic/Polled HW I/O pin information
 * MON-HW 0x0A 0x09 60 Periodic/Polled Hardware Status
 * MON-IO 0x0A 0x02 0 + 20*N Periodic/Polled I/O Subsystem Status
 * MON-MSGPP 0x0A 0x06 120 Periodic/Polled Message Parse and Process Status
 * MON-PATCH 0x0A 0x27 4 + 16*nEntries Polled Output information about installed...
 * MON-RF 0x0A 0x38 4 + 24*nBlocks Periodic/Polled RF information
 * MON-RXBUF 0x0A 0x07 24 Periodic/Polled Receiver Buffer Status
 * MON-RXR 0x0A 0x21 1 Output Receiver Status Information
 * MON-TXBUF 0x0A 0x08 28 Periodic/Polled Transmitter Buffer Status
 * MON-VER 0x0A 0x04 40 + 30*N Polled Receiver/Software Version
 */
#define UBX_MON_COMMS 0x36
#define UBX_MON_GNSS 0x28
#define UBX_MON_HW2 0x0B
#define UBX_MON_HW3 0x37
#define UBX_MON_HW 0x09
#define UBX_MON_IO 0x02
#define UBX_MON_MSGPP 0x06
#define UBX_MON_PATCH 0x27
#define UBX_MON_RF 0x38
#define UBX_MON_RXBUF 0x07
#define UBX_MON_RXR 0x21
#define UBX_MON_TXBUF 0x08
#define UBX_MON_VER 0x04

/**
 * UBX Class NAV Navigation Results Messages
 * -----------------------------------------
 * NAV-CLOCK 0x01 0x22 20 Periodic/Polled Clock Solution
 * NAV-DOP 0x01 0x04 18 Periodic/Polled Dilution of precision
 * NAV-EOE 0x01 0x61 4 Periodic End Of Epoch
 * NAV-GEOFENCE 0x01 0x39 8 + 2*numFe... Periodic/Polled Geofencing status
 * NAV-HPPOSECEF 0x01 0x13 28 Periodic/Polled High Precision Position Solution
 * in ECEF NAV-HPPOSLLH 0x01 0x14 36 Periodic/Polled High Precision Geodetic
 * Position Solution NAV-ODO 0x01 0x09 20 Periodic/Polled Odometer Solution
 * NAV-ORB 0x01 0x34 8 + 6*numSv Periodic/Polled GNSS Orbit Database Info
 * NAV-POSECEF 0x01 0x01 20 Periodic/Polled Position Solution in ECEF
 * NAV-POSLLH 0x01 0x02 28 Periodic/Polled Geodetic Position Solution
 * NAV-PVT 0x01 0x07 92 Periodic/Polled Navigation Position Velocity Time...
 * NAV-RELPOSNED 0x01 0x3C 64 Periodic/Polled Relative Positioning Information
 * in... NAV-RESETODO 0x01 0x10 0 Command Reset odometer NAV-SAT 0x01 0x35 8 +
 * 12*numSvs Periodic/Polled Satellite Information NAV-SIG 0x01 0x43 8 +
 * 16*numSi... Periodic/Polled Signal Information NAV-STATUS 0x01 0x03 16
 * Periodic/Polled Receiver Navigation Status NAV-SVIN 0x01 0x3B 40
 * Periodic/Polled Survey-in data NAV-TIMEBDS 0x01 0x24 20 Periodic/Polled BDS
 * Time Solution NAV-TIMEGAL 0x01 0x25 20 Periodic/Polled Galileo Time Solution
 * NAV-TIMEGLO 0x01 0x23 20 Periodic/Polled GLO Time Solution
 * NAV-TIMEGPS 0x01 0x20 16 Periodic/Polled GPS Time Solution
 * NAV-TIMELS 0x01 0x26 24 Periodic/Polled Leap second event information
 * NAV-TIMEUTC 0x01 0x21 20 Periodic/Polled UTC Time Solution
 * NAV-VELECEF 0x01 0x11 20 Periodic/Polled Velocity Solution in ECEF
 * NAV-VELNED 0x01 0x12 36 Periodic/Polled Velocity Solution in NED
 */
#define UBX_NAV_CLOCK 0x22
#define UBX_NAV_DOP 0x04
#define UBX_NAV_EOE 0x61
#define UBX_NAV_GEOFENCE 0x39
#define UBX_NAV_HPPOSECEF 0x13
#define UBX_NAV_HPPOSLLH 0x14
#define UBX_NAV_ODO 0x09
#define UBX_NAV_ORB 0x34
#define UBX_NAV_POSECEF 0x01
#define UBX_NAV_POSLLH 0x02
#define UBX_NAV_PVT 0x07
#define UBX_NAV_RELPOSNED 0x3C
#define UBX_NAV_RESETODO 0x10
#define UBX_NAV_SAT 0x35
#define UBX_NAV_SIG 0x43
#define UBX_NAV_STATUS 0x03
#define UBX_NAV_SVIN 0x3B
#define UBX_NAV_TIMEBDS 0x24
#define UBX_NAV_TIMEGAL 0x25
#define UBX_NAV_TIMEGLO 0x23
#define UBX_NAV_TIMEGPS 0x20
#define UBX_NAV_TIMELS 0x26
#define UBX_NAV_TIMEUTC 0x21
#define UBX_NAV_VELECEF 0x11
#define UBX_NAV_VELNED 0x12

/**
 * UBX Class RXM Receiver Manager Messages
 * ---------------------------------------
 * RXM-MEASX 0x02 0x14 44 + 24*num... Periodic/Polled Satellite Measurements for
 * RRLP RXM-PMREQ 0x02 0x41 8 Command Requests a Power Management task RXM-PMREQ
 * 0x02 0x41 16 Command Requests a Power Management task RXM-RAWX 0x02 0x15 16 +
 * 32*num... Periodic/Polled Multi-GNSS Raw Measurement Data RXM-RLM 0x02 0x59
 * 16 Output Galileo SAR Short-RLM report RXM-RLM 0x02 0x59 28 Output Galileo
 * SAR Long-RLM report RXM-RTCM 0x02 0x32 8 Output RTCM input status RXM-SFRBX
 * 0x02 0x13 8 + 4*numW... Output
 */
#define UBX_RXM_MEASX 0x14
#define UBX_RXM_PMREQ 0x41
#define UBX_RXM_RAWX 0x15
#define UBX_RXM_RLM 0x59
#define UBX_RXM_RTCM 0x32
#define UBX_RXM_SFRBX 0x13

#define CFG_SIGNAL_GPS_ENA 0x1031001f
#define CFG_SIGNAL_GPS_L1CA_ENA 0x10310001
#define CFG_SIGNAL_QZSS_ENA 0x10310024
#define CFG_SIGNAL_BDS_B2_ENA 0x1031000e

#define CFG_RATE_MEAS 0x30210001
#define CFG_UART1_BAUDRATE 0x40520001
#define CFG_USBOUTPROT_NMEA 0x10780002

#define CFG_MSGOUT_RTCM_3X_TYPE1005_USB 0x209102c0
#define CFG_MSGOUT_RTCM_3X_TYPE1077_USB 0x209102cf
#define CFG_MSGOUT_RTCM_3X_TYPE1087_USB 0x209102d4
#define CFG_MSGOUT_RTCM_3X_TYPE1097_USB 0x2091031b
#define CFG_MSGOUT_RTCM_3X_TYPE1127_USB 0x209102d9
#define CFG_MSGOUT_RTCM_3X_TYPE1230_USB 0x20910306

#define CFG_MSGOUT_UBX_NAV_CLOCK_USB 0x20910068
#define CFG_MSGOUT_UBX_NAV_DOP_USB 0x2091003b
#define CFG_MSGOUT_UBX_NAV_EOE_USB 0x20910162
#define CFG_MSGOUT_UBX_NAV_HPPOSEECF_USB 0x20910031
#define CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB 0x20910036
#define CFG_MSGOUT_UBX_NAV_RELPOSNED_USB 0x20910090
#define CFG_MSGOUT_UBX_NAV_STATUS_USB 0x2091001d
#define CFG_MSGOUT_UBX_NAV_SVIN_USB 0x2091008b
#define CFG_MSGOUT_UBX_NAV_PVT_USB 0x20910009
#define CFG_MSGOUT_UBX_NAV_VELNED_USB 0x20910045
#define CFG_MSGOUT_UBX_MON_RF_USB 0x2091035c
#define CFG_MSGOUT_UBX_RXM_RTCM_USB 0x2091026b

#define CFG_TMODE_MODE 0x20030001
#define CFG_TMODE_SVIN_MIN_DUR 0x40030010
#define CFG_TMODE_SVIN_ACC_LIMIT 0x40030011

#define CFG_NAVSPG_DYNMODEL 0x20110021

/****************************** UBX Message ***********************************/

#define UBX_MON_RF_MAX_BLOCKS 100

class ubx_msg_t {
public:
  uint8_t ok;

  uint8_t msg_class;
  uint8_t msg_id;
  uint16_t payload_length;
  uint8_t payload[1024];
  uint8_t ck_a;
  uint8_t ck_b;

  ubx_msg_t() {
    ok = 0;

    msg_class = 0;
    msg_id = 0;
    payload_length = 0;
    memset(payload, '\0', sizeof(uint8_t) * 1024);
    ck_a = 0;
    ck_b = 0;
  }

  static void checksum(const uint8_t msg_class_,
                       const uint8_t msg_id_,
                       const uint16_t payload_length_,
                       const uint8_t *payload_,
                       uint8_t *ck_a,
                       uint8_t *ck_b) {
    *ck_a = 0;
    *ck_b = 0;

    *ck_a = *ck_a + msg_class_;
    *ck_b = *ck_b + *ck_a;

    *ck_a = *ck_a + msg_id_;
    *ck_b = *ck_b + *ck_a;

    *ck_a = *ck_a + (payload_length_ & 0x00FF);
    *ck_b = *ck_b + *ck_a;

    *ck_a = *ck_a + ((payload_length_ & 0xFF00) >> 8);
    *ck_b = *ck_b + *ck_a;

    for (uint16_t i = 0; i < payload_length_; i++) {
      *ck_a = *ck_a + payload_[i];
      *ck_b = *ck_b + *ck_a;
    }
  }

  uint8_t is_valid() {
    uint8_t expected_ck_a = 0;
    uint8_t expected_ck_b = 0;
    checksum(msg_class,
             msg_id,
             payload_length,
             payload,
             &expected_ck_a,
             &expected_ck_b);

    if (expected_ck_a == ck_a && expected_ck_b == ck_b) {
      return 1;
    }

    return 0;
  }

  void build(const uint8_t msg_class_,
             const uint8_t msg_id_,
             const uint16_t length_,
             const uint8_t *payload_) {
    // OK
    ok = 1;

    // Header
    msg_class = msg_class_;
    msg_id = msg_id_;
    payload_length = length_;

    // Payload
    if (payload_) {
      for (size_t i = 0; i < length_; i++) {
        payload[i] = payload_[i];
      }
    }

    // Checksum
    checksum(msg_class, msg_id, payload_length, payload, &ck_a, &ck_b);
  }

  void parse(const uint8_t *data) {
    // Check SYNC_1 and SYNC_2
    if (data[0] != 0xB5 || data[1] != 0x62) {
      ok = 0;
      return;
    }

    // Message class and id
    msg_class = data[2];
    msg_id = data[3];

    // Payload
    payload_length = (data[5] << 8) | (data[4]);
    for (uint16_t i = 0; i < payload_length; i++) {
      payload[i] = data[6 + i];
    }

    // Checksum
    ck_a = data[payload_length + 6];
    ck_b = data[payload_length + 6 + 1];
    ok = is_valid();
  }

  void serialize(uint8_t *frame, size_t *frame_size) {
    // Form packet frame (header[6] + payload length + checksum[2])
    *frame_size = 6 + payload_length + 2;

    // -- Form header
    frame[0] = 0xB5;                              // Sync Char 1
    frame[1] = 0x62;                              // Sync Char 2
    frame[2] = msg_class;                    // Message class
    frame[3] = msg_id;                       // Message id
    frame[4] = payload_length & 0xFF;        // Length
    frame[5] = (payload_length >> 8) & 0xFF; // Length

    // -- Form payload
    for (size_t i = 0; i < payload_length; i++) {
      frame[6 + i] = payload[i];
    }

    // -- Form checksum
    frame[*frame_size - 2] = ck_a;
    frame[*frame_size - 1] = ck_b;
  }

  void print() {
    /* printf("msg_class: 0x%02x\n", msg->msg_class); */
    /* printf("msg_id: 0x%02x\n", msg->msg_id); */
    /* printf("payload_length: 0x%02x\n", msg->payload_length); */
    /* for (size_t i = 0; i < msg->payload_length; i++) { */
    /*   printf("payload[%zu]: 0x%02x\n", i, msg->payload[i]); */
    /* } */
    /* printf("ck_a: 0x%02x\n", msg->ck_a); */
    /* printf("ck_b: 0x%02x\n", msg->ck_b); */
  }
};

class ubx_nav_dop_t : public ubx_msg_t {
public:
  uint32_t itow = 0;
  uint16_t gdop = 0;
  uint16_t pdop = 0;
  uint16_t tdop = 0;
  uint16_t vdop = 0;
  uint16_t hdop = 0;
  uint16_t ndop = 0;
  uint16_t edop = 0;

  ubx_nav_dop_t(const ubx_msg_t *msg) {
    itow = uint32(msg->payload, 0);
    gdop = int16(msg->payload, 4);
    pdop = int16(msg->payload, 6);
    tdop = int16(msg->payload, 8);
    vdop = int16(msg->payload, 10);
    hdop = int16(msg->payload, 12);
    ndop = int16(msg->payload, 14);
    edop = int16(msg->payload, 16);
  }
};

class ubx_nav_eoe_t : public ubx_msg_t {
public:
  uint32_t itow = 0;

  ubx_nav_eoe_t(const ubx_msg_t *msg) {
    itow = uint32(msg->payload, 0);
  }
};

class ubx_nav_hpposllh_t : public ubx_msg_t {
public:
  uint8_t version = 0;
  uint32_t itow = 0;
  int32_t lon = 0;
  int32_t lat = 0;
  int32_t height = 0;
  int32_t hmsl = 0;
  int8_t lon_hp = 0;
  int8_t lat_hp = 0;
  int8_t height_hp = 0;
  int8_t hmsl_hp = 0;
  uint32_t hacc = 0;
  uint32_t vacc = 0;

  ubx_nav_hpposllh_t(const ubx_msg_t *msg) {
    version = uint8(msg->payload, 0);
    itow = uint32(msg->payload, 4);
    lon = int32(msg->payload, 8);
    lat = int32(msg->payload, 12);
    height = int32(msg->payload, 16);
    hmsl = int32(msg->payload, 20);
    lon_hp = int8(msg->payload, 24);
    lat_hp = int8(msg->payload, 25);
    height_hp = int8(msg->payload, 26);
    hmsl_hp = int8(msg->payload, 27);
    hacc = uint32(msg->payload, 28);
    vacc = uint32(msg->payload, 32);
  }
};

class ubx_nav_pvt_t : public ubx_msg_t {
public:
  uint32_t itow = 0;
  uint16_t year = 0;
  uint8_t month = 0;
  uint8_t day = 0;
  uint8_t hour = 0;
  uint8_t min = 0;
  uint8_t sec = 0;
  uint8_t valid = 0;
  uint32_t tacc = 0;
  int32_t nano = 0;
  uint8_t fix_type = 0;
  uint8_t flags = 0;
  uint8_t flags2 = 0;
  uint8_t num_sv = 0;
  uint32_t lon = 0;
  uint32_t lat = 0;
  uint32_t height = 0;
  uint32_t hmsl = 0;
  int32_t hacc = 0;
  int32_t vacc = 0;
  int32_t veln = 0;
  int32_t vele = 0;
  int32_t veld = 0;
  int32_t gspeed = 0;
  int32_t headmot = 0;
  uint32_t sacc = 0;
  uint32_t headacc = 0;
  uint16_t pdop = 0;
  int32_t headveh = 0;
  int16_t magdec = 0;
  uint16_t magacc = 0;

  ubx_nav_pvt_t(const ubx_msg_t *msg) {
    itow = uint32(msg->payload, 0);
    year = uint16(msg->payload, 4);
    month = uint8(msg->payload, 6);
    day = uint8(msg->payload, 7);
    hour = uint8(msg->payload, 8);
    min = uint8(msg->payload, 9);
    sec = uint8(msg->payload, 10);
    valid = uint8(msg->payload, 11);
    tacc = uint32(msg->payload, 12);
    nano = int32(msg->payload, 16);
    fix_type = uint8(msg->payload, 20);
    flags = uint8(msg->payload, 21);
    flags2 = uint8(msg->payload, 22);
    num_sv = uint8(msg->payload, 23);
    lon = int32(msg->payload, 24);
    lat = int32(msg->payload, 28);
    height = int32(msg->payload, 32);
    hmsl = int32(msg->payload, 36);
    hacc = uint32(msg->payload, 40);
    vacc = uint32(msg->payload, 44);
    veln = int32(msg->payload, 48);
    vele = int32(msg->payload, 52);
    veld = int32(msg->payload, 56);
    gspeed = int32(msg->payload, 60);
    headmot = int32(msg->payload, 64);
    sacc = uint32(msg->payload, 68);
    headacc = uint32(msg->payload, 72);
    pdop = uint16(msg->payload, 76);
    headveh = int32(msg->payload, 84);
    magdec = int16(msg->payload, 88);
    magacc = uint16(msg->payload, 90);
  }
};

class ubx_nav_status_t : public ubx_msg_t {
public:
  uint32_t itow = 0;
  uint8_t fix = 0;
  uint8_t flags = 0;
  uint8_t fix_status = 0;
  uint8_t flags2 = 0;
  uint32_t ttff = 0;
  uint32_t msss = 0;

  ubx_nav_status_t(const ubx_msg_t *msg) {
    itow = uint32(msg->payload, 0);
    fix = uint8(msg->payload, 4);
    flags = uint8(msg->payload, 5);
    fix_status = uint8(msg->payload, 6);
    flags2 = uint8(msg->payload, 7);
    ttff = uint32(msg->payload, 8);
    msss = uint32(msg->payload, 12);
  }
};

class ubx_nav_svin_t : public ubx_msg_t {
public:
  uint32_t itow = 0;
  uint32_t dur = 0;
  int32_t mean_x = 0;
  int32_t mean_y = 0;
  int32_t mean_z = 0;
  int8_t mean_xhp = 0;
  int8_t mean_yhp = 0;
  int8_t mean_zhp = 0;
  uint32_t mean_acc = 0;
  uint32_t obs = 0;
  uint8_t valid = 0;
  uint8_t active = 0;

  ubx_nav_svin_t(const ubx_msg_t *msg) {
    itow = uint32(msg->payload, 4);
    dur = uint32(msg->payload, 8);
    mean_x = int32(msg->payload, 12);
    mean_y = int32(msg->payload, 16);
    mean_z = int32(msg->payload, 20);
    mean_xhp = int8(msg->payload, 24);
    mean_yhp = int8(msg->payload, 25);
    mean_zhp = int8(msg->payload, 26);
    mean_acc = uint32(msg->payload, 28);
    obs = uint32(msg->payload, 32);
    valid = uint8(msg->payload, 36);
    active = uint8(msg->payload, 37);
  }
};

class ubx_nav_velned_t : public ubx_msg_t {
public:
  uint32_t itow = 0;
  int32_t veln = 0;
  int32_t vele = 0;
  int32_t veld = 0;
  uint32_t speed = 0;
  uint32_t gspeed = 0;
  int32_t heading = 0;
  uint32_t sacc = 0;
  uint32_t cacc = 0;

  ubx_nav_velned_t(const ubx_msg_t *msg) {
    itow = uint32(msg->payload, 0);
    veln = int32(msg->payload, 4);
    vele = int32(msg->payload, 8);
    veld = int32(msg->payload, 12);
    speed = uint32(msg->payload, 16);
    gspeed = uint32(msg->payload, 20);
    heading = int32(msg->payload, 24);
    sacc = uint32(msg->payload, 28);
    cacc = uint32(msg->payload, 32);
  }
};

class ubx_rxm_rtcm_t : public ubx_msg_t {
public:
  uint8_t flags = 0;
  uint16_t sub_type = 0;
  uint16_t ref_station = 0;
  uint16_t msg_type = 0;

  ubx_rxm_rtcm_t(const ubx_msg_t *msg) {
    flags = msg->payload[1];
    sub_type = uint16(msg->payload, 2);
    ref_station = uint16(msg->payload, 4);
    msg_type = uint16(msg->payload, 6);
  }
};

class ubx_mon_rf_t : public ubx_msg_t {
public:
  uint8_t version = 0;
  uint32_t nblocks = 0;

  uint8_t block_id[UBX_MON_RF_MAX_BLOCKS] = {0};
  uint8_t flags[UBX_MON_RF_MAX_BLOCKS] = {0};
  uint8_t ant_status[UBX_MON_RF_MAX_BLOCKS] = {0};
  uint8_t ant_power[UBX_MON_RF_MAX_BLOCKS] = {0};
  uint32_t post_status[UBX_MON_RF_MAX_BLOCKS] = {0};
  uint16_t noise_per_ms[UBX_MON_RF_MAX_BLOCKS] = {0};
  uint16_t agc_cnt[UBX_MON_RF_MAX_BLOCKS] = {0};
  uint8_t jam_ind[UBX_MON_RF_MAX_BLOCKS] = {0};
  int8_t ofs_i[UBX_MON_RF_MAX_BLOCKS] = {0};
  uint8_t mag_i[UBX_MON_RF_MAX_BLOCKS] = {0};
  int8_t ofs_q[UBX_MON_RF_MAX_BLOCKS] = {0};
  uint8_t mag_q[UBX_MON_RF_MAX_BLOCKS] = {0};

  ubx_mon_rf_t(const ubx_msg_t *msg) {
    version = uint8(msg->payload, 0);
    nblocks = uint8(msg->payload, 1);

    for (uint32_t i = 0; i < nblocks; i++) {
      const uint32_t offset = 24 * i;
      block_id[i] = uint8(msg->payload, 4 + offset);
      flags[i] = uint8(msg->payload, 5 + offset);
      ant_status[i] = uint8(msg->payload, 6 + offset);
      ant_power[i] = uint8(msg->payload, 7 + offset);
      post_status[i] = uint8(msg->payload, 8 + offset);
      noise_per_ms[i] = uint8(msg->payload, 16 + offset);
      agc_cnt[i] = uint8(msg->payload, 18 + offset);
      jam_ind[i] = uint8(msg->payload, 20 + offset);
      ofs_i[i] = uint8(msg->payload, 21 + offset);
      mag_i[i] = uint8(msg->payload, 22 + offset);
      ofs_q[i] = uint8(msg->payload, 23 + offset);
      mag_q[i] = uint8(msg->payload, 24 + offset);
    }
  }
};

/**************************** UBX Stream Parser *******************************/

/**
 * UBX Stream Parser States
 */
#define SYNC_1 0
#define SYNC_2 1
#define MSG_CLASS 2
#define MSG_ID 3
#define PAYLOAD_LENGTH_LOW 4
#define PAYLOAD_LENGTH_HI 5
#define PAYLOAD_DATA 6
#define CK_A 7
#define CK_B 8

/**
 * UBX Stream Parser
 */
class ubx_parser_t {
public:
  uint8_t state;
  uint8_t buf_data[9046];
  uint16_t buf_pos;
  ubx_msg_t msg;

  ubx_parser_t() {
    state = SYNC_1;
    memset(buf_data, '\0', 9046);
    buf_pos = 0;
  }

  void reset() {
    state = SYNC_1;
    for (size_t i = 0; i < 1024; i++) {
      buf_data[i] = 0;
    }
    buf_pos = 0;
  }

  int update(uint8_t data) {
    // Add byte to buffer
    buf_data[buf_pos++] = data;

    // Parse byte
    switch (state) {
    case SYNC_1:
      if (data == 0xB5) {
        state = SYNC_2;
      } else {
        reset();
      }
      break;
    case SYNC_2:
      if (data == 0x62) {
        state = MSG_CLASS;
      } else {
        reset();
      }
      break;
    case MSG_CLASS: state = MSG_ID; break;
    case MSG_ID: state = PAYLOAD_LENGTH_LOW; break;
    case PAYLOAD_LENGTH_LOW: state = PAYLOAD_LENGTH_HI; break;
    case PAYLOAD_LENGTH_HI: state = PAYLOAD_DATA; break;
    case PAYLOAD_DATA: {
      uint8_t length_low = buf_data[4];
      uint8_t length_hi = buf_data[5];
      uint16_t payload_length = (length_hi << 8) | (length_low);
      if (buf_pos == 6 + payload_length) {
        state = CK_A;
      }
      if (buf_pos >= 1022) {
        reset();
        return -2;
      }
      break;
    }
    case CK_A: state = CK_B; break;
    case CK_B:
      msg.parse(buf_data);
      reset();
      return 1;
    /* default: FATAL("Invalid Parser State!"); break; */
    default: return -2;
    }

    return 0;
  }
};

/*************************** RTCM3 Stream Parser ******************************/

/**
 * RTCM3 Stream Parser
 */
class rtcm3_parser_t {
public:
  uint8_t buf_data[9046] = {0};
  size_t buf_pos = 0;
  size_t msg_len = 0;
  size_t msg_type = 0;

  rtcm3_parser_t() {}

  void reset() {
    for (size_t i = 0; i < 9046; i++) {
      buf_data[i] = 0;
    }
    buf_pos = 0;
    msg_len = 0;
  }

  /**
   * RTCM 3.2 Frame
   * --------------
   * Byte 0: Always 0xD3
   * Byte 1: 6-bits of zero
   * Byte 2: 10-bits of length of this packet including the first two-ish header
   *         bytes, + 6.
   * byte 3 + 4: Msg type 12 bits
   *
   * Example [Msg type 1087]:
   *
   *   D3 00 7C 43 F0 ...
   *
   * Where 0x7C is the payload size = 124
   * = 124 + 6 [header]
   * = 130 total bytes in this packet
   */
  int update(uint8_t data) {
    // Add byte to buffer
    buf_data[buf_pos] = data;

    // Parse message
    if (buf_data[0] != 0xD3) {
      reset();

    } else if (buf_pos == 1) {
      // Get the last two bits of this byte. Bits 8 and 9 of 10-bit length
      msg_len = (data & 0x03) << 8;

    } else if (buf_pos == 2) {
      msg_len |= data; // Bits 0-7 of packet length
      msg_len += 6;
      // There are 6 additional bytes of what we presume is
      // header, msgType, CRC, and stuff
      //
    } else if (buf_pos == 3) {
      msg_type = data << 4; // Message Type, most significant 4 bits

    } else if (buf_pos == 4) {
      msg_type |= (data >> 4); // Message Type, bits 0-7

    }
    buf_pos++;

    // Check if end of message
    if (buf_pos == msg_len) {
      return 1;
    }

    return 0;
  }
};

/*********************************** UBlox ************************************/

#define UBLOX_MAX_CONNS 10
#define UBLOX_READY 0
#define UBLOX_PARSING_UBX 1
#define UBLOX_PARSING_RTCM3 2

typedef class ublox_t ublox_t;
typedef void (*ubx_msg_callback)(ublox_t *ublox);
typedef void (*rtcm3_msg_callback)(ublox_t *ublox);

class ublox_t {
public:
  int state;
  uint8_t ok;
  /* serial_t serial; */

  int sockfd;
  int conns[UBLOX_MAX_CONNS];
  size_t nb_conns;

  ubx_parser_t ubx_parser;
  ubx_msg_callback ubx_cb = nullptr;

  rtcm3_parser_t rtcm3_parser;
  rtcm3_msg_callback rtcm3_cb = nullptr;

  ublox_t() {}

  void reset() {
    state = UBLOX_READY;
    ok = 0;

    /* // Serial */
    /* if (serial.connected) { */
    /*   serial_disconnect(&serial); */
    /* } */
    /*  */
    /* // Connections */
    /* for (size_t i = 0; i < nb_conns; i++) { */
    /*   close(conns[i]); */
    /*   conns[i] = -1; */
    /* } */
    /* nb_conns = 0; */

    /* // Socket */
    /* if (sockfd != -1) { */
    /*   close(sockfd); */
    /* } */
    /* sockfd = -1; */

    // Parsers
    ubx_parser.reset();
    rtcm3_parser.reset();
  }

  /* int connect(ublox_t *ublox) { */
  /*   if (serial_connect(&ublox->serial) != 0) { */
  /*     return -1; */
  /*   } */
  /*   ok = 1; */
  /*   return 0; */
  /* } */
  /*  */
  /* void disconnect(ublox_t *ublox) { */
  /*   return reset(ublox); */
  /* } */

  int transmit(uint8_t msg_class,
               uint8_t msg_id,
               uint16_t length,
               uint8_t *payload) {
    // Build UBX message
    ubx_msg_t msg;
    msg.build(msg_class, msg_id, length, payload);

    // Serialize the message
    uint8_t frame[1024] = {0};
    size_t frame_size = 0;
    msg.serialize(frame, &frame_size);

    // Transmit msg
    /* size_t retval = write(ublox->serial.connfd, frame, frame_size); */
    /* if (retval != frame_size) { */
    /*   LOG_ERROR("Failed to send data to UART!"); */
    /*   return -1; */
    /* } */

    return 0;
  }

  int poll(const uint8_t msg_class,
           const uint8_t msg_id,
           uint16_t *payload_length,
           uint8_t *payload,
           const uint8_t expect_ack,
           const int retry) {
    int attempts = 0;
    ubx_parser_t parser;

request:
    // Request
    attempts++;
    if (attempts > retry) {
      payload_length = 0;
      return -1;
    }
    transmit(msg_class, msg_id, *payload_length, payload);

    // Arbitrary counter for response timeout
    int counter = 0;
response:
    // Response
    while (counter < 1024) {
      /* uint8_t data = 0; */
      /* if (serial_read(&ublox->serial, &data, 1) == 0) { */
      /*   if (parser.update(data) == 1) { */
      /*     break; */
      /*   } */
      /* } */

      counter++;
    }

    // Check parsed message
    if (parser.msg.ok == 0) {
      /* LOG_WARN("Checksum failed, retrying ..."); */
      goto request;
    }

    // Try sending the request again?
    if (counter == 1024) {
      goto request;
    }

    // Check message
    const uint8_t msg_is_ack = (parser.msg.msg_class == UBX_ACK);
    const uint8_t match_class = (parser.msg.msg_class == msg_class);
    const uint8_t match_id = (parser.msg.msg_id == msg_id);
    if (!msg_is_ack && match_class && match_id) {
      // Copy payload length and data
      for (uint16_t i = 0; i < parser.msg.payload_length; i++) {
        payload[i] = parser.msg.payload[i];
      }
      *payload_length = parser.msg.payload_length;

      // Get another message (hopefully an ACK)
      if (expect_ack) {
        counter = 0;
        goto response;
      } else {
        return 0;
      }
    }
    if (!msg_is_ack && !match_class && !match_id) {
      // Get another message
      goto response;

    } else if (expect_ack && msg_is_ack) {
      // Check the ACK message
      const uint8_t match_class = (msg_class == parser.msg.payload[0]);
      const uint8_t match_id = (msg_id == parser.msg.payload[1]);
      const uint8_t is_ack = (parser.msg.msg_id == UBX_ACK_ACK);
      if (match_class && match_id && is_ack) {
        return 0;
      } else {
        return -2;
      }
    }

    return 0;
  }

  int read_ack(const uint8_t msg_class, const uint8_t msg_id) {
    ubx_parser_t parser;

    // Get Ack
    int counter = 0; // Arbitrary counter for timeout
    while (counter != 1024) {
      uint8_t data = 0;
      /* if (serial_read(&ublox->serial, &data, 1) != 0) { */
      /*   continue; */
      /* } */

      if (parser.update(data) == 1) {
        const uint8_t is_ack_msg = (parser.msg.msg_class == UBX_ACK);
        const uint8_t ack_msg_class = parser.msg.payload[0];
        const uint8_t ack_msg_id = parser.msg.payload[1];
        const uint8_t ack_msg_class_match = (ack_msg_class == msg_class);
        const uint8_t ack_msg_id_match = (ack_msg_id == msg_id);

        if (is_ack_msg && ack_msg_class_match && ack_msg_id_match) {
          break;
        }
      }

      counter++;
    }

    // Try again?
    if (counter == 1024) {
      return 1;
    }

    return (parser.msg.msg_id == UBX_ACK_ACK) ? 0 : -1;
  }

  int get(const uint8_t layer, const uint32_t key, uint32_t *val) {
    // Build message
    uint16_t payload_len = 4 + 4;
    uint8_t payload[1024] = {0};
    payload[0] = 0; // Version
    payload[1] = layer;
    payload[4 + 0] = key >> 0;
    payload[4 + 1] = key >> 8;
    payload[4 + 2] = key >> 16;
    payload[4 + 3] = key >> 24;

    // Poll
    if (poll(UBX_CFG, UBX_CFG_VALGET, &payload_len, payload, 1, 1) != 0) {
      return -1;
    }

    *val = uint32(payload, 8);
    return 0;
  }

  int set(const uint8_t layer,
          const uint32_t key,
          const uint32_t val,
          const uint8_t val_size) {
    const uint32_t bit_masks[4] = {0x000000FF,
                                   0x0000FF00,
                                   0x00FF0000,
                                   0xFF000000};

    // Build message
    uint16_t payload_length = 4 + 4 + val_size;
    uint8_t payload[1024] = {0};
    payload[0] = 0; // Version
    payload[1] = layer;
    payload[2] = 0;

    payload[4 + 0] = (key & bit_masks[0]);
    payload[4 + 1] = (key & bit_masks[1]) >> 8;
    payload[4 + 2] = (key & bit_masks[2]) >> 16;
    payload[4 + 3] = (key & bit_masks[3]) >> 24;

    for (uint8_t i = 0; i < val_size; i++) {
      payload[4 + 4 + i] = ((val & bit_masks[i]) >> (8 * i));
    }

    // Set value
    int attempts = 0;
retry:
    attempts++;
    if (attempts >= 5) {
      /* LOG_ERROR("Failed to set configuration!"); */
      return -1;
    }

    transmit(UBX_CFG, UBX_CFG_VALSET, payload_length, payload);
    switch (read_ack(UBX_CFG, UBX_CFG_VALSET)) {
    case 0: return 0;
    case 1: goto retry;
    case -1:
    /* default: LOG_ERROR("Failed to set configuration!"); return -1; */
    default: return -1;
    }
  }

  void version() {
    uint16_t length = 0;
    uint8_t payload[1024] = {0};
    if (poll(UBX_MON, 0x04, &length, payload, 0, 5) == 0) {
      /* printf("SW VERSION: %s\n", payload); */
      /* printf("HW VERSION: %s\n", payload + 30); */
      /* printf("%s\n", payload + 40); */
    } else {
      /* LOG_ERROR("Failed to obtain UBlox version!"); */
    }
  }

  int parse_ubx(uint8_t data) {
    if (ubx_parser.update(data) == 1) {
      // DEBUG("[UBX]\tmsg_class: %d\tmsg_id: %d",
      //       ubx_parser.msg.msg_class,
      //       ubx_parser.msg.msg_id);

      // UBX message callback
      if (ubx_cb) {
        ubx_cb(this);
      }
      state = UBLOX_READY;
      return 1;
    }

    return 0;
  }

  void broadcast_rtcm3() {
    // const uint8_t *msg_data = rtcm3_parser.buf_data;
    // const size_t msg_len = rtcm3_parser.msg_len;
    // const int msg_flags = MSG_DONTWAIT | MSG_NOSIGNAL;

    // Broad cast RTCM3 to clients and check client connection
    int good_conns[UBLOX_MAX_CONNS] = {0};
    size_t nb_conns = 0;

    for (size_t i = 0; i < nb_conns; i++) {
      /* const int conn_fd = conns[i]; */
      // if (send(conn_fd, msg_data, msg_len, msg_flags) == -1) {
      //   /* LOG_ERROR("Rover diconnected!"); */
      // } else {
      //   good_conns[nb_conns] = conn_fd;
      //   nb_conns++;
      // }
    }

    // Clear connections
    for (size_t i = 0; i < UBLOX_MAX_CONNS; i++) {
      conns[i] = 0;
    }
    nb_conns = 0;

    // Copy good connections back to conns
    for (size_t i = 0; i < nb_conns; i++) {
      conns[i] = good_conns[i];
    }
    nb_conns = nb_conns;
  }

  int parse_rtcm3(uint8_t data) {
    if (rtcm3_parser.update(data)) {
      // Debug
      // DEBUG("[RTCM3]\tmsg type: %zu\tmsg length: %zu",
      //       ublox->rtcm3_parser.msg_type,
      //       ublox->rtcm3_parser.msg_len);

      // RTCM3 message callback
      if (rtcm3_cb) {
        rtcm3_cb(this);
      }

      // Reset parser and msg type
      rtcm3_parser.reset();
      state = UBLOX_READY;

      return 1;
    }

    return 0;
  }
};

/*******************************************************************************
 *                                    PID
 ******************************************************************************/

class pid_ctrl_t {
public:
  float error_prev = 0.0f;
  float error_sum = 0.0f;

  float error_p = 0.0f;
  float error_i = 0.0f;
  float error_d = 0.0f;

  float k_p = 0.0f;
  float k_i = 0.0f;
  float k_d = 0.0f;

  pid_ctrl_t() {}

  pid_ctrl_t(const float k_p_, const float k_i_, const float k_d_)
    : k_p{k_p_}, k_i{k_i_}, k_d{k_d_} {}

  float update(const float setpoint, const float actual, const float dt) {
    // Calculate errors
    const float error = setpoint - actual;
    error_sum += error * dt;

    // Calculate output
    error_p = k_p * error;
    error_i = k_i * error_sum;
    error_d = k_d * (error - error_prev) / dt;
    const float output = error_p + error_i + error_d;

    error_prev = error;
    return output;
  }

  void reset(pid_ctrl_t *pid) {
    error_prev = 0.0;
    error_sum = 0.0;

    error_p = 0.0;
    error_i = 0.0;
    error_d = 0.0;
  }
};

/*******************************************************************************
 *                            ATTITUDE CONTROLLER
 ******************************************************************************/

class att_ctrl_t {
public:
  pid_ctrl_t roll_ctrl;
  pid_ctrl_t pitch_ctrl;
  pid_ctrl_t yaw_ctrl;

  float roll_limits[2] = {0.0f, 0.0f};
  float pitch_limits[2] = {0.0f, 0.0f};
  float max_thrust = 0.0f;

  float outputs[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float dt = 0.0f;

  att_ctrl_t() {
    roll_ctrl = pid_ctrl_t{0.0f, 0.0f, 0.0f};
    pitch_ctrl = pid_ctrl_t{0.0f, 0.0f, 0.0f};
    yaw_ctrl = pid_ctrl_t{0.0f, 0.0f, 0.0f};
  }

  void update(const float setpoints[4], const float T_WB[16], const float dt_) {
    // Check rate
    dt += dt_;
    if (this->dt < 0.001f) {
      return;
    }

    // Form actual
    float r_WB[3] = {0};
    float q_WB[4] = {0};
    float rpy[3] = {0.0f, 0.0f, 0.0f};
    tf_trans(T_WB, r_WB);
    tf_quat(T_WB, q_WB);
    quat2euler(q_WB, rpy);
    const float actual[4] = {rpy[0], rpy[1], rpy[2], r_WB[2]};

    // Calculate yaw error
    float actual_yaw = rad2deg(actual[2]);
    float setpoint_yaw = rad2deg(setpoints[2]);
    float error_yaw = setpoint_yaw - actual_yaw;

    // Wrap yaw
    if (error_yaw > 180.0) {
      error_yaw -= 360.0;
    } else if (error_yaw < -180.0) {
      error_yaw += 360.0;
    }
    error_yaw = deg2rad(error_yaw);

    // Roll, pitch and yaw
    float r = roll_ctrl.update(setpoints[0], actual[0], dt);
    float p = pitch_ctrl.update(setpoints[1], actual[1], dt);
    float y = yaw_ctrl.update(error_yaw, 0.0, dt);
    r = (r < roll_limits[0]) ? roll_limits[0] : r;
    r = (r > roll_limits[1]) ? roll_limits[1] : r;
    p = (p < pitch_limits[0]) ? pitch_limits[0] : p;
    p = (p > pitch_limits[1]) ? pitch_limits[1] : p;

    // Thrust
    float t = max_thrust * setpoints[3];
    t = (t > max_thrust) ? max_thrust : t;
    t = (t < 0.0) ? 0.0 : t;

    // Map roll, pitch, yaw and thrust to motor outputs
    outputs[0] = -p - y + t;
    outputs[1] = -r + y + t;
    outputs[2] = p - y + t;
    outputs[3] = r + y + t;

    // Limit outputs
    for (int i = 0; i < 4; i++) {
      outputs[i] = (outputs[i] > max_thrust) ? max_thrust : outputs[i];
      outputs[i] = (outputs[i] < 0.0) ? 0.0 : outputs[i];
    }

    // Reset dt
    dt = 0.0;
  }
};

/*******************************************************************************
 *                                    FCU
 ******************************************************************************/

class fcu_t {
public:
  // Interface
  i2c_t &i2c;
  pwm_t &pwm;
  gpio_t &gpio;
  serial_t &serial;

  // Ultrasound distance sensor
  /* const int trig_pin = PA5; */
  /* const int echo_pin = PA6; */
  /* hcsr04_t uds{gpio, trig_pin, echo_pin}; */

  // IMU
  mpu6050_t imu{i2c};

	// Humidity sensor

  // Control
  att_ctrl_t att_ctrl;

  fcu_t(i2c_t &i2c_, pwm_t &pwm_, gpio_t &gpio_, serial_t &serial_)
    : i2c{i2c_}, pwm{pwm_}, gpio{gpio_}, serial{serial_} {}

  void update() {
    imu.get_data();
  }
};


// GLOBAL VARIABLES
auto t_prev = millis();
i2c_t i2c;
pwm_t pwm;
gpio_t gpio;
serial_t serial;
fcu_t fcu{i2c, pwm, gpio, serial};
bmp280_t hs{i2c, serial};

void setup() {}

/* void test_uds() { */
/*   float distance = fcu.hcsr04.measure(); */
/*   serial.printf("distance: %f [cm]\n\r", 0.0); */
/*   gpio.delay(1 * 1e3); */
/* } */

void test_imu() {
  fcu.imu.get_data();
  serial.printf("x: %f  ", fcu.imu.accel[0]);
  serial.printf("y: %f  ", fcu.imu.accel[1]);
  serial.printf("z: %f  ", fcu.imu.accel[2]);
  serial.printf("\n\r");
}

void test_tps() {
	if (hs.data_ready()) {
		float temperature = 0.0f;
		float pressure = 0.0f;
		hs.get_data(temperature, pressure);

		/* serial.printf("%f  ", temperature); */
		/* serial.printf("%f\n\r", pressure); */

		serial.printf("temperature: %f [deg]  ", temperature);
		serial.printf("pressure: %f [Pa]", pressure);

		auto t_now = millis();
		float t_elapsed = ((float) t_now - t_prev) * 1e-3;
		serial.printf("time elapsed: %f [s]\n\r", t_elapsed);
		t_prev = t_now;
	}
}

void loop() {
	test_tps();
}
