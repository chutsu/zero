#include "pwm.hpp"

void pwm_setup(pwm_t *pwm, const uint8_t pin, const uint8_t freq) {
  pinMode(pin, OUTPUT);
	PinName pin_name = digitalPinToPinName(pin);
	TIM_TypeDef *instance = (TIM_TypeDef *) pinmap_peripheral(pin_name, PinMap_PWM);

	pwm->timer = new HardwareTimer(instance);
	pwm->pin = pin;
	pwm->freq = freq;
	pwm->channel = STM_PIN_CHANNEL(pinmap_function(pin_name, PinMap_PWM));
}

void pwm_set(pwm_t *pwm, const uint8_t dutycycle) {
	pwm->timer->setPWM(pwm->channel, pwm->pin, pwm->freq, dutycycle);
}
