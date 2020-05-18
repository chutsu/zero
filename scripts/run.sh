#!/bin/sh
set -e

debug() {
  gdb \
    -ex=run \
    -ex=bt \
    -ex="set confirm off" \
    -ex=quit \
    --args "$1" "$2" "$3"
}

# node js/zero.js

# make format_code
# make clean
time make

# ./build/bin/test_zero
# ZERO-DATA
# ./build/bin/test_zero --target test_malloc_string
# ./build/bin/test_zero --target test_csv_rows
# ./build/bin/test_zero --target test_csv_cols
# ./build/bin/test_zero --target test_csv_fields
# ./build/bin/test_zero --target test_csv_data
# ZERO-LINEAR_ALGEBRA
# ./build/bin/test_zero --target test_eye
# ./build/bin/test_zero --target test_ones
# ./build/bin/test_zero --target test_zeros
# ./build/bin/test_zero --target test_mat_set
# ./build/bin/test_zero --target test_mat_val
# ./build/bin/test_zero --target test_mat_block_get
# ./build/bin/test_zero --target test_mat_block_set
# ./build/bin/test_zero --target test_mat_diag_get
# ./build/bin/test_zero --target test_mat_diag_set
# ./build/bin/test_zero --target test_mat_triu
# ./build/bin/test_zero --target test_mat_tril
# ./build/bin/test_zero --target test_mat_trace
# ./build/bin/test_zero --target test_mat_transpose
# ./build/bin/test_zero --target test_mat_add
# ./build/bin/test_zero --target test_mat_sub
# ./build/bin/test_zero --target test_vec_add
# ./build/bin/test_zero --target test_vec_sub
# ./build/bin/test_zero --target test_dot
# ./build/bin/test_zero --target test_skew
# ./build/bin/test_zero --target test_check_jacobian
# ZERO-SVD
# ./build/bin/test_zero --target test_svd
# ./build/bin/test_zero --target test_svdcomp
# ./build/bin/test_zero --target test_pinv
# ZERO-CHOL
# ./build/bin/test_zero --target test_cholesky
# ./build/bin/test_zero --target test_chol_lls_solve
# ./build/bin/test_zero --target test_chol_lls_solve2
# ./build/bin/test_zero --target test_chol_Axb
# ZERO-TRANSFORMS
# ./build/bin/test_zero --target test_tf_set_rot
# ./build/bin/test_zero --target test_tf_set_trans
# ./build/bin/test_zero --target test_tf_trans
# ./build/bin/test_zero --target test_tf_rot
# ./build/bin/test_zero --target test_tf_quat
# ./build/bin/test_zero --target test_tf_inv
# ./build/bin/test_zero --target test_tf_point
# ./build/bin/test_zero --target test_tf_hpoint
# ./build/bin/test_zero --target test_tf_perturb_rot
# ./build/bin/test_zero --target test_tf_perturb_trans
# ./build/bin/test_zero --target test_quat2rot
# ZERO-POSE
# ./build/bin/test_zero --target test_pose_init
# ./build/bin/test_zero --target test_pose_set_get_quat
# ./build/bin/test_zero --target test_pose_set_get_trans
# ./build/bin/test_zero --target test_pose2tf
# ./build/bin/test_zero --target test_load_poses

# ./build/bin/test_ba
# ./build/bin/test_ba --target test_load_camera
# ./build/bin/test_ba --target test_load_keypoints
# ./build/bin/test_ba --target test_ba_residuals
# ./build/bin/test_ba --target test_J_cam_pose
# ./build/bin/test_ba --target test_J_landmark
# ./build/bin/test_ba --target test_ba_jacobian
# ./build/bin/test_ba --target test_ba_update
# ./build/bin/test_ba --target test_ba_cost
./build/bin/test_ba --target test_ba_solve

# ./build/bin/test_template

# cd firmware
# arduino --upload firmware.ino --port /dev/ttyUSB0
# make clean
# make flash
