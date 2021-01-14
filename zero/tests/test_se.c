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

int test_speed_bias_setup() {
  uint64_t param_id = 0;
  timestamp_t ts = 1;
  speed_bias_t sb;

  real_t data[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  speed_bias_setup(&sb, &param_id, ts, data);

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

int test_landmark_setup() {
  uint64_t param_id = 0;
  landmark_t landmark;

  real_t data[3] = {0.1, 0.2, 0.3};
  landmark_setup(&landmark, &param_id, data);

  MU_CHECK(param_id == 1);
  MU_CHECK(landmark.param_id == 0);

  MU_CHECK(fltcmp(landmark.data[0], 0.1) == 0.0);
  MU_CHECK(fltcmp(landmark.data[1], 0.2) == 0.0);
  MU_CHECK(fltcmp(landmark.data[2], 0.3) == 0.0);

  return 0;
}

int test_extrinsics_setup() {
  uint64_t param_id = 0;
  timestamp_t ts = 1;
  extrinsics_t extrinsics;

  real_t data[7] = {1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3};
  extrinsics_setup(&extrinsics, &param_id, ts, data);

  MU_CHECK(param_id == 1);
  MU_CHECK(extrinsics.param_id == 0);
  MU_CHECK(extrinsics.ts == 1);

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

  return 0;
}

int test_pose_factor_eval() {
  pose_t pose;

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

int test_fgraph_setup() {
  fgraph_t graph;
  fgraph_setup(&graph);
  return 0;
}

int test_fgraph_print() {
  fgraph_t graph;
  fgraph_setup(&graph);
  fgraph_print(&graph);
  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_pose_setup);
  MU_ADD_TEST(test_speed_bias_setup);
  MU_ADD_TEST(test_landmark_setup);
  MU_ADD_TEST(test_extrinsics_setup);
  MU_ADD_TEST(test_camera_setup);

  MU_ADD_TEST(test_pose_factor_eval);
  MU_ADD_TEST(test_fgraph_setup);
  MU_ADD_TEST(test_fgraph_print);
}

MU_RUN_TESTS(test_suite)
