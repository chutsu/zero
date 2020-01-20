#ifndef HC_SR04_HPP
#define HC_SR04_HPP

#include <stdint.h>

#include <Arduino.h>

struct hcsr04_t {
  uint8_t trig_pin;
  uint8_t echo_pin;
};

void hcsr04_setup(hcsr04_t *sensor,
                  const uint8_t trig_pin,
                  const uint8_t echo_pin) {
  // Configure trigger and echo pins
  sensor->trig_pin = trig_pin;
  sensor->echo_pin = echo_pin;
  pinMode(sensor->trig_pin, OUTPUT);
  pinMode(sensor->echo_pin, INPUT);
}

float hcsr04_measure(hcsr04_t *sensor) {
  // Makesure pin is low
  digitalWrite(sensor->trig_pin, LOW);
  delayMicroseconds(2);

  // Hold trigger for 10 microseconds
  digitalWrite(sensor->trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor->trig_pin, LOW);

  // Measure echo signal
  const float duration_ms = pulseIn(sensor->echo_pin, HIGH, 1000);
  const float temperature = 20.0;
  double sof_cm = 0.03313 + 0.0000606 * temperature;
  double dist_cm = duration_ms / 2.0 * sof_cm;
  if (dist_cm == 0 || dist_cm > 400) {
    return -1.0 ;
  } else {
    return dist_cm;
  }
}

#endif  // HC_SR04_HPP
