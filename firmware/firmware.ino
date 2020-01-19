#include <Wire.h>

#include "i2c.hpp"
#include "mpu6050.hpp"
#include "pwm.hpp"
#include "hcsr04.hpp"

// GLOBAL VARIABLES
mpu6050_t imu;
hcsr04_t uds;
// pwm_t pwm;

uint8_t trig_pin = PB8;
uint8_t echo_pin = PB9;

void print_mpu6050_config() {
  Serial.print("Accel sensitivity: ");
  Serial.print(imu.accel_sensitivity);
  Serial.print("\n\r");

  Serial.print("Gyro sensitivity: ");
  Serial.print(imu.gyro_sensitivity);
  Serial.print("\n\r");

  Serial.print("DPLF: ");
  Serial.print(mpu6050_get_dplf());
  Serial.print("\n\r");

  Serial.print("Ping: ");
  Serial.print(mpu6050_ping());
  Serial.print("\n\r");

  Serial.print("Sample rate div: ");
  Serial.print(mpu6050_get_sample_rate_div());
  Serial.print("\n\r");

  Serial.print("Sample rate: ");
  Serial.print(imu.sample_rate);
  Serial.print("\n\r");
}

void setup() {

  i2c_setup();
	// mpu6050_setup(&imu);
	hcsr04_setup(&uds, trig_pin, echo_pin);
	/* pwm_setup(&pwm, trig_pin, 1000); */

  pinMode(trig_pin, OUTPUT);
  digitalWrite(trig_pin, LOW);
  pinMode(echo_pin, INPUT);


  Serial.begin(115200);
  Serial.print("------------");
  Serial.print("\n\r");
  Serial.print("setup done\n\r");

  float distance = hcsr04_measure(&uds);
  Serial.print("distance: ");
  Serial.print(distance);
  Serial.print("\n\r");

  // print_mpu6050_config();
}

void loop() {
  float distance = hcsr04_measure(&uds);
  Serial.print("distance: ");
  Serial.print(distance);
  Serial.print("\n\r");

	// mpu6050_get_data(&imu);
  // Serial.print("x: "); Serial.print(imu.accel[0]); Serial.print(" ");
  // Serial.print("y: "); Serial.print(imu.accel[1]); Serial.print(" ");
  // Serial.print("z: "); Serial.print(imu.accel[2]); Serial.print(" ");
  // Serial.print("\n\r");
}

void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {
  // transmit diagnostic informations through serial link.
  Serial.println(__func);
  Serial.println(__file);
  Serial.println(__lineno, DEC);
  Serial.println(__sexp);
  Serial.flush();
  // abort program execution.
  abort();
}
