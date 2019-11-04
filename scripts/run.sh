#!/bin/sh
set -e

# make format_code
# make clean
make
# ./build/bin/test_math --target test_eye
# ./build/bin/test_math --target test_ones
# ./build/bin/test_math --target test_zeros
# ./build/bin/test_math --target test_mat_set
# ./build/bin/test_math --target test_mat_val
# ./build/bin/test_math --target test_mat_block
# ./build/bin/test_math --target test_mat_transpose
# ./build/bin/test_math --target test_mat_add
# ./build/bin/test_math --target test_mat_sub
# ./build/bin/test_math --target test_vec_add
# ./build/bin/test_math --target test_vec_sub
# ./build/bin/test_math --target test_dot
# ./build/bin/test_math --target test_tf_set_rot
# ./build/bin/test_math --target test_tf_set_trans
# ./build/bin/test_math --target test_tf_trans
# ./build/bin/test_math --target test_tf_rot
# ./build/bin/test_math --target test_tf_quat
# ./build/bin/test_math --target test_tf_inv
# ./build/bin/test_math --target test_tf_point
# ./build/bin/test_math --target test_tf_hpoint
# ./build/bin/test_math --target test_quat2rot
# ./build/bin/test_template

# cd firmware
# arduino --upload firmware6050.ino --port /dev/ttyUSB0
# make clean
# make flash
