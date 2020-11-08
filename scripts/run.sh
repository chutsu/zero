#!/bin/sh
set -e
# ARDUINO=~/Downloads/arduino-1.8.12/arduino

debug() {
  gdb \
    -ex=run \
    -ex=bt \
    -ex="set confirm off" \
    -ex=quit \
    --args "$1" "$2" "$3"
}

# $ARDUINO --upload firmware/firmware.ino --port /dev/ttyUSB0
# $ARDUINO --upload firmware/firmware.ino

# node js/zero.js

# python3 scripts/tf_point.py

# make format_code
# make clean
# time make

cd ./build/bin
# ZERO-DATA
# ./test_zero --target test_malloc_string
# ./test_zero --target test_csv_rows
# ./test_zero --target test_csv_cols
# ./test_zero --target test_csv_fields
# ./test_zero --target test_csv_data
# ZERO-LINEAR_ALGEBRA
# ./test_zero --target test_eye
# ./test_zero --target test_ones
# ./test_zero --target test_zeros
# ./test_zero --target test_mat_set
# ./test_zero --target test_mat_val
# ./test_zero --target test_mat_block_get
# ./test_zero --target test_mat_block_set
# ./test_zero --target test_mat_diag_get
# ./test_zero --target test_mat_diag_set
# ./test_zero --target test_mat_triu
# ./test_zero --target test_mat_tril
# ./test_zero --target test_mat_trace
# ./test_zero --target test_mat_transpose
# ./test_zero --target test_mat_add
# ./test_zero --target test_mat_sub
# ./test_zero --target test_vec_add
# ./test_zero --target test_vec_sub
# ./test_zero --target test_dot
# ./test_zero --target test_skew
# ./test_zero --target test_check_jacobian
# ZERO-SVD
# ./test_zero --target test_svd
# ./test_zero --target test_svdcomp
# ./test_zero --target test_pinv
# ZERO-CHOL
# ./test_zero --target test_chol
# ./test_zero --target test_chol_lls_solve
# ./test_zero --target test_chol_lls_solve2
# ./test_zero --target test_chol_Axb
# ZERO-TRANSFORMS
# ./test_zero --target test_tf_set_rot
# ./test_zero --target test_tf_set_trans
# ./test_zero --target test_tf_trans
# ./test_zero --target test_tf_rot
# ./test_zero --target test_tf_quat
# ./test_zero --target test_tf_inv
# ./test_zero --target test_tf_point
# ./test_zero --target test_tf_hpoint
# ./test_zero --target test_tf_perturb_rot
# ./test_zero --target test_tf_perturb_trans
# ./test_zero --target test_quat2rot
# ZERO-POSE
# ./test_zero --target test_pose_init
# ./test_zero --target test_pose_set_get_quat
# ./test_zero --target test_pose_set_get_trans
# ./test_zero --target test_pose2tf
# ./test_zero --target test_load_poses

# ./test_ba
# ./test_ba --target test_load_camera
# ./test_ba --target test_parse_keypoints_line
# ./test_ba --target test_load_keypoints
# ./test_ba --target test_ba_residuals
# ./test_ba --target test_J_cam_pose
# ./test_ba --target test_J_landmark
# ./test_ba --target test_ba_jacobian
# ./test_ba --target test_ba_update
# ./test_ba --target test_ba_cost
time ./test_ba --target test_ba_solve

# ./test_gui

# ./test_template
