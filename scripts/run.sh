#!/bin/sh
set -e

# make format_code
make clean
make
./build/bin/test_math --target test_tf_set_rot
./build/bin/test_math --target test_tf_set_trans
./build/bin/test_math --target test_tf_quat
# ./build/bin/test_math --target test_tf_point
# ./build/bin/test_math --target test_tf_hpoint
./build/bin/test_math --target test_quat2rot
# ./build/bin/test_template

# cd firmware
# arduino --upload firmware6050.ino --port /dev/ttyUSB0
# make clean
# make flash
