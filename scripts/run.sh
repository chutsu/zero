#!/bin/sh
set -e

# make format_code
# make clean
# make
# ./build/bin/test_math
# ./build/bin/test_template

cd firmware
arduino --upload firmware6050.ino --port /dev/ttyUSB0
# make clean
# make flash
