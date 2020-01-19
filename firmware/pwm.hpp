#ifndef PWM_HPP
#define PWM_HPP

#include <stdint.h>

#include <Arduino.h>

struct pwm_t {
  uint8_t pin = 0;
  uint8_t freq = 0;
  uint32_t channel = 0;
  HardwareTimer *timer = nullptr;
};

void pwm_setup(pwm_t *pwm, const uint8_t pin, const uint8_t freq) {
  pwm->pin = pin;
  pwm->freq = freq;
  pwm->channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));

  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
  pwm->timer = new HardwareTimer(Instance);
  pwm->timer->setPWM(pwm->channel, pin, freq, 50);
}

void pwm_set(const pwm_t *pwm, const uint8_t dutycycle) {
  pwm->timer->pause();
  pwm->timer->setPWM(pwm->channel, pwm->pin, pwm->freq, dutycycle);
  pwm->timer->refresh();
  pwm->timer->resume();
}

#endif  // PWM_HPP
