#ifndef PWM_HPP
#define PWM_HPP

#include <stdint.h>

#include <Arduino.h>

struct pwm_t {
	HardwareTimer *timer = nullptr;
	uint8_t pin = 0;
	uint8_t freq = 0;
	uint32_t channel = 0;
};

void pwm_setup(pwm_t *pwm, const uint8_t pin, const uint8_t freq);
void pwm_set(pwm_t *pwm, const uint8_t dutycycle);

#endif  // PWM_HPP
