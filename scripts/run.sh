#!/bin/sh
set -e

# make format_code
# make clean
# make
# ./build/bin/test_math
# ./build/bin/test_template

cd firmware
cd MPU6050
arduino --upload MPU6050.ino --port /dev/ttyUSB0
# make clean
# make flash
