/* #include <Wire.h> */
/*  */
/* #include "i2c.hpp" */
/* #include "mpu6050.hpp" */
/*  */
/* // GLOBAL VARIABLES */
/* mpu6050_t imu; */
/*  */
/* void setup() { */
/*   i2c_setup(); */
/*   mpu6050_setup(&imu); */
/*  */
/*   Serial.begin(115200); */
/*   Serial.print("------------"); */
/*   Serial.print("\n\r"); */
/* } */
/*  */
/* void loop() { */
/* 	mpu6050_get_data(&imu); */
/*   Serial.print("x: "); Serial.print(imu.accel[0]); Serial.print(" "); */
/*   Serial.print("y: "); Serial.print(imu.accel[1]); Serial.print(" "); */
/*   Serial.print("z: "); Serial.print(imu.accel[2]); Serial.print(" "); */
/*   Serial.print("\n\r"); */
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
