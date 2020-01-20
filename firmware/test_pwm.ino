/* #include <Wire.h> */
/*  */
/* #include "pwm.hpp" */
/*  */
/* // GLOBAL VARIABLES */
/* pwm_t pwm; */
/* uint8_t freq = 10; */
/* uint8_t dutycycle = 100; */
/*  */
/* uint8_t trig_pin = PA6; */
/* uint8_t echo_pin = PA5; */
/*  */
/* void setup() { */
/*   Serial.begin(115200); */
/*   Serial.print("------------"); */
/*   Serial.print("\n\r"); */
/*  */
/*   pwm_setup(&pwm, trig_pin, freq); */
/*   pinMode(echo_pin, INPUT); */
/* } */
/*  */
/* void loop() { */
/*   // Change PWM dutycycle */
/*   dutycycle -= 1; */
/*   pwm_set(&pwm, dutycycle); */
/*  */
/*   // Print dutycycle */
/*   Serial.print("dutycycle: "); */
/*   Serial.print(dutycycle); */
/*   Serial.print("\t"); */
/*  */
/*   // Print duration */
/*   const float duration = pulseIn(echo_pin, LOW) * 1e-6; */
/*   Serial.print("duration: "); */
/*   Serial.print(duration, 4); */
/*   Serial.print(" [s]"); */
/*   Serial.print("\n\r"); */
/*  */
/*   // Reset dutycycle */
/*   if (dutycycle == 1) { */
/*     dutycycle = 100; */
/*     delay(5 * 1e3); */
/*   } */
/* } */
/*  */
/* void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) { */
/*   // transmit diagnostic informations through serial link. */
/*   Serial.println(__func); */
/*   Serial.println(__file); */
/*   Serial.println(__lineno, DEC); */
/*   Serial.println(__sexp); */
/*   Serial.flush(); */
/*   // abort program execution. */
/*   abort(); */
/* } */
