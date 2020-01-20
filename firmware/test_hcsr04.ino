#include <Wire.h>

#include "pwm.hpp"
#include "hcsr04.hpp"

// GLOBAL VARIABLES
pwm_t pwm;
hcsr04_t uds;
uint8_t trig_pin = PA5;
uint8_t echo_pin = PA6;

void setup() {
  Serial.begin(115200);
  Serial.print("\n\r");
  Serial.print("------------");
  Serial.print("\n\r");

  pwm_setup(&pwm, trig_pin, 10);
  hcsr04_setup(&uds, trig_pin, echo_pin);
}

void loop() {
  Serial.print("distance: ");
  const float distance = hcsr04_measure(&uds);
  Serial.print(distance, 4);
  Serial.print(" [cm]");
  Serial.print("\n\r");
  delay(1 * 1e3);
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
