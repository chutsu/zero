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

// void pwm_setup(pwm_t *pwm, const uint8_t pin, const uint8_t freq) {
// 	PinName pin_name = digitalPinToPinName(pin);
// 	TIM_TypeDef *instance = (TIM_TypeDef *) pinmap_peripheral(pin_name, PinMap_PWM);
//
// 	pwm->timer = new HardwareTimer(instance);
// 	pwm->pin = pin;
// 	pwm->freq = freq;
// 	pwm->channel = STM_PIN_CHANNEL(pinmap_function(pin_name, PinMap_PWM));
// }
//
// void pwm_set(pwm_t *pwm, const uint8_t dutycycle) {
// 	pwm->timer->setPWM(pwm->channel, pwm->pin, pwm->freq, dutycycle);
// }

#endif  // PWM_HPP
