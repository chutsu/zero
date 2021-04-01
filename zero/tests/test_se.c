#include "munit.h"
#include "zero/se.h"

int test_pose_setup() {
  uint64_t param_id = 0;
  timestamp_t ts = 1;
  pose_t pose;

  real_t data[7] = {1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3};
  pose_setup(&pose, &param_id, ts, data);

  MU_CHECK(param_id == 1);
  MU_CHECK(pose.param_id == 0);
  MU_CHECK(pose.ts == 1);

  MU_CHECK(fltcmp(pose.data[0], 1.0) == 0.0);
  MU_CHECK(fltcmp(pose.data[1], 0.0) == 0.0);
  MU_CHECK(fltcmp(pose.data[2], 0.0) == 0.0);
  MU_CHECK(fltcmp(pose.data[3], 0.0) == 0.0);

  MU_CHECK(fltcmp(pose.data[4], 0.1) == 0.0);
  MU_CHECK(fltcmp(pose.data[5], 0.2) == 0.0);
  MU_CHECK(fltcmp(pose.data[6], 0.3) == 0.0);

  return 0;
}

int test_speed_biases_setup() {
  uint64_t param_id = 0;
  timestamp_t ts = 1;
  speed_biases_t sb;

  real_t data[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  speed_biases_setup(&sb, &param_id, ts, data);

  MU_CHECK(param_id == 1);
  MU_CHECK(sb.param_id == 0);
  MU_CHECK(sb.ts == 1);

  MU_CHECK(fltcmp(sb.data[0], 1.0) == 0.0);
  MU_CHECK(fltcmp(sb.data[1], 2.0) == 0.0);
  MU_CHECK(fltcmp(sb.data[2], 3.0) == 0.0);

  MU_CHECK(fltcmp(sb.data[3], 4.0) == 0.0);
  MU_CHECK(fltcmp(sb.data[4], 5.0) == 0.0);
  MU_CHECK(fltcmp(sb.data[5], 6.0) == 0.0);

  MU_CHECK(fltcmp(sb.data[6], 7.0) == 0.0);
  MU_CHECK(fltcmp(sb.data[7], 8.0) == 0.0);
  MU_CHECK(fltcmp(sb.data[8], 9.0) == 0.0);

  return 0;
}

int test_feature_setup() {
  uint64_t param_id = 0;
  feature_t feature;

  real_t data[3] = {0.1, 0.2, 0.3};
  feature_setup(&feature, &param_id, data);

  MU_CHECK(param_id == 1);
  MU_CHECK(feature.param_id == 0);

  MU_CHECK(fltcmp(feature.data[0], 0.1) == 0.0);
  MU_CHECK(fltcmp(feature.data[1], 0.2) == 0.0);
  MU_CHECK(fltcmp(feature.data[2], 0.3) == 0.0);

  return 0;
}

int test_extrinsics_setup() {
  uint64_t param_id = 0;
  extrinsics_t extrinsics;

  real_t data[7] = {1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3};
  extrinsics_setup(&extrinsics, &param_id, data);

  MU_CHECK(param_id == 1);
  MU_CHECK(extrinsics.param_id == 0);

  MU_CHECK(fltcmp(extrinsics.data[0], 1.0) == 0.0);
  MU_CHECK(fltcmp(extrinsics.data[1], 0.0) == 0.0);
  MU_CHECK(fltcmp(extrinsics.data[2], 0.0) == 0.0);
  MU_CHECK(fltcmp(extrinsics.data[3], 0.0) == 0.0);

  MU_CHECK(fltcmp(extrinsics.data[4], 0.1) == 0.0);
  MU_CHECK(fltcmp(extrinsics.data[5], 0.2) == 0.0);
  MU_CHECK(fltcmp(extrinsics.data[6], 0.3) == 0.0);

  return 0;
}

int test_camera_setup() {
  uint64_t param_id = 0;

  camera_t camera;
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t data[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_setup(&camera,
               &param_id,
               cam_idx, cam_res,
               proj_model, dist_model,
               data);

  camera_print(&camera);

  return 0;
}

int test_pose_factor_setup() {
  uint64_t param_id = 0;

  timestamp_t ts = 1;
  pose_t pose;
  real_t data[7] = {1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3};
  pose_setup(&pose, &param_id, ts, data);

  pose_factor_t pose_factor;
  real_t var[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  pose_factor_setup(&pose_factor, &pose, var);

  print_matrix("pose_factor.covar", pose_factor.covar, 6, 6);
  print_matrix("pose_factor.r", pose_factor.r, 6, 1);
  print_matrix("pose_factor.J0", pose_factor.J0, 6, 6);

  return 0;
}

int test_pose_factor_eval() {
  /* pose_t pose; */

  /* pose_factor_t pose_factor; */
  /* pose_factor->ts = 0; */
  /* pose_factor->measured[0]; */
  /* pose_factor->measured[1]; */
  /* pose_factor->measured[2]; */
  /* pose_factor->measured[3]; */
  /* pose_factor->measured[4]; */
  /* pose_factor->measured[5]; */
  /* pose_factor->measured[6]; */
  /* double covar[6 * 6]; */

  return 0;
}

int test_cam_factor_setup() {
  uint64_t param_id = 0;

  timestamp_t ts = 0;
  pose_t pose;
  {
    real_t data[7] = {1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3};
    pose_setup(&pose, &param_id, ts, data);
  }

  extrinsics_t extrinsics;
  {
    real_t data[7] = {1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3};
    extrinsics_setup(&extrinsics, &param_id, data);
  }

  camera_t camera;
  {
    const int cam_idx = 0;
    const int cam_res[2] = {752, 480};
    const char *proj_model = "pinhole";
    const char *dist_model = "radtan4";
    const real_t data[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
    camera_setup(&camera,
                &param_id,
                cam_idx, cam_res,
                proj_model, dist_model,
                data);
  }

  cam_factor_t cam_factor;
  real_t var[2] = {1.0, 1.0};
  cam_factor_setup(&cam_factor, &pose, &extrinsics, &camera, var);

  print_matrix("cam_factor.covar", cam_factor.covar, 2, 2);
  print_matrix("cam_factor.r", cam_factor.r, 2, 1);
  print_matrix("cam_factor.J0", cam_factor.J0, 2, 6);
  print_matrix("cam_factor.J1", cam_factor.J1, 2, 6);
  print_matrix("cam_factor.J2", cam_factor.J2, 2, 8);
  print_matrix("cam_factor.J3", cam_factor.J3, 2, 3);

  return 0;
}

int test_cam_factor_eval() {
  uint64_t param_id = 0;

  timestamp_t ts = 0;
  pose_t pose;
  {
    real_t data[7] = {1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3};
    pose_setup(&pose, &param_id, ts, data);
  }

  extrinsics_t extrinsics;
  {
    real_t data[7] = {1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3};
    extrinsics_setup(&extrinsics, &param_id, data);
  }

  camera_t camera;
  {
    const int cam_idx = 0;
    const int cam_res[2] = {752, 480};
    const char *proj_model = "pinhole";
    const char *dist_model = "radtan4";
    const real_t data[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
    camera_setup(&camera,
                &param_id,
                cam_idx, cam_res,
                proj_model, dist_model,
                data);
  }

  cam_factor_t cam_factor;
  real_t var[2] = {1.0, 1.0};
  cam_factor_setup(&cam_factor, &pose, &extrinsics, &camera, var);
	cam_factor_eval(&cam_factor);

	return 0;
}

int test_imu_buf_setup() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  return 0;
}

int test_imu_buf_add() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buf_add(&imu_buf, ts, acc, gyr);
  imu_buf_print(&imu_buf);

	MU_CHECK(imu_buf.size == 1);
	MU_CHECK(imu_buf.ts[0] == ts);
	MU_CHECK(fltcmp(imu_buf.acc[0][0], 1.0) == 0);
	MU_CHECK(fltcmp(imu_buf.acc[0][1], 2.0) == 0);
	MU_CHECK(fltcmp(imu_buf.acc[0][2], 3.0) == 0);
	MU_CHECK(fltcmp(imu_buf.gyr[0][0], 1.0) == 0);
	MU_CHECK(fltcmp(imu_buf.gyr[0][1], 2.0) == 0);
	MU_CHECK(fltcmp(imu_buf.gyr[0][2], 3.0) == 0);

	return 0;
}

int test_imu_buf_clear() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buf_add(&imu_buf, ts, acc, gyr);
	imu_buf_clear(&imu_buf);

	MU_CHECK(imu_buf.size == 0);
	MU_CHECK(imu_buf.ts[0] == 0);
	MU_CHECK(fltcmp(imu_buf.acc[0][0], 0.0) == 0);
	MU_CHECK(fltcmp(imu_buf.acc[0][1], 0.0) == 0);
	MU_CHECK(fltcmp(imu_buf.acc[0][2], 0.0) == 0);
	MU_CHECK(fltcmp(imu_buf.gyr[0][0], 0.0) == 0);
	MU_CHECK(fltcmp(imu_buf.gyr[0][1], 0.0) == 0);
	MU_CHECK(fltcmp(imu_buf.gyr[0][2], 0.0) == 0);

	return 0;
}

int test_imu_buf_copy() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buf_add(&imu_buf, ts, acc, gyr);

  imu_buf_t imu_buf2;
	imu_buf_setup(&imu_buf2);
	imu_buf_copy(&imu_buf, &imu_buf2);

	MU_CHECK(imu_buf2.size == 1);
	MU_CHECK(imu_buf2.ts[0] == ts);
	MU_CHECK(fltcmp(imu_buf2.acc[0][0], 1.0) == 0);
	MU_CHECK(fltcmp(imu_buf2.acc[0][1], 2.0) == 0);
	MU_CHECK(fltcmp(imu_buf2.acc[0][2], 3.0) == 0);
	MU_CHECK(fltcmp(imu_buf2.gyr[0][0], 1.0) == 0);
	MU_CHECK(fltcmp(imu_buf2.gyr[0][1], 2.0) == 0);
	MU_CHECK(fltcmp(imu_buf2.gyr[0][2], 3.0) == 0);

	return 0;
}

int test_imu_buf_print() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buf_add(&imu_buf, ts, acc, gyr);

  imu_buf_print(&imu_buf);
  return 0;
}

int test_solver_setup() {
  solver_t solver;
  solver_setup(&solver);
  return 0;
}

/* int test_solver_print() { */
/*   solver_t solver; */
/*   solver_setup(&solver); */
/*   solver_print(&solver); */
/*   return 0; */
/* } */

void test_suite() {
  MU_ADD_TEST(test_pose_setup);
  MU_ADD_TEST(test_speed_biases_setup);
  MU_ADD_TEST(test_feature_setup);
  MU_ADD_TEST(test_extrinsics_setup);
  MU_ADD_TEST(test_camera_setup);

  MU_ADD_TEST(test_pose_factor_setup);
  MU_ADD_TEST(test_pose_factor_eval);

  MU_ADD_TEST(test_cam_factor_setup);
  MU_ADD_TEST(test_cam_factor_eval);

  MU_ADD_TEST(test_imu_buf_setup);
  MU_ADD_TEST(test_imu_buf_add);
  MU_ADD_TEST(test_imu_buf_clear);
  MU_ADD_TEST(test_imu_buf_copy);
  MU_ADD_TEST(test_imu_buf_print);
  /* MU_ADD_TEST(test_imu_factor_setup); */
  /* MU_ADD_TEST(test_imu_factor_eval); */

  /* MU_ADD_TEST(test_solver_setup); */
  /* MU_ADD_TEST(test_solver_print); */
  /* MU_ADD_TEST(test_solver_eval); */
  /* MU_ADD_TEST(test_solver_solve); */
}

MU_RUN_TESTS(test_suite)
