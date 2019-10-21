#ifndef HC_SR04_HPP
#define HC_SR04_HPP

#include <stdint.h>

#include "pwm.hpp"

struct hcsr04_t {
  uint8_t trig_pin;
  uint8_t echo_pin;
  pwm_t pwm;
};

void hcsr04_setup(hcsr04_t *sensor,
                   const uint8_t trig_pin,
                   const uint8_t echo_pin) {
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  sensor->trig_pin = trig_pin;
  sensor->echo_pin = echo_pin;
	pwm_setup(&sensor->pwm, trig_pin, 20);
	pwm_set(&sensor->pwm, 50);
}

float hcsr04_measure(hcsr04_t *sensor) {
  digitalWrite(sensor->trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor->trig_pin, LOW);
  const float duration = pulseIn(sensor->echo_pin, HIGH, 1000);
  digitalWrite(sensor->trig_pin, LOW);
  return duration * 0.00343 / 2.0
}

#endif  // HC_SR04_HPP
