#include "zero/munit.h"
#include "zero/ba.h"

/* #define TEST_DATA "zero/tests/test_data/ba_data" */
#define TEST_DATA "/tmp/ba_data"

int test_parse_keypoints_line() {
  keypoints_t *keypoints = parse_keypoints_line("4,1,2,3,4\n");

  /* keypoints_print(keypoints); */
  MU_CHECK(keypoints->size == 2);
  MU_CHECK(fltcmp(keypoints->data[0][0], 1.0) == 0);
  MU_CHECK(fltcmp(keypoints->data[0][1], 2.0) == 0);
  MU_CHECK(fltcmp(keypoints->data[1][0], 3.0) == 0);
  MU_CHECK(fltcmp(keypoints->data[1][1], 4.0) == 0);

  keypoints_delete(keypoints);

  return 0;
}

int test_load_keypoints() {
  int nb_frames = 0;
  keypoints_t **keypoints = load_keypoints(TEST_DATA, &nb_frames);

  for (int i = 0; i < nb_frames; i++) {
    MU_CHECK(keypoints[i] != NULL);
    MU_CHECK(keypoints[i]->size > 0);
    /* printf("frame[%d]\n", i); */
    /* keypoints_print(keypoints[i]); */
    keypoints_delete(keypoints[i]);
  }
  free(keypoints);

  return 0;
}

int test_ba_load_data() {
  ba_data_t *data = ba_load_data(TEST_DATA);
  ba_data_free(data);
  return 0;
}

int test_ba_residuals() {
  ba_data_t *data = ba_load_data(TEST_DATA);

  int r_size = 0;
  double *r = ba_residuals(data, &r_size);
  for (int i = 0; i < r_size; i++) {
    MU_CHECK(r[i] < 0.01);
  }

  const double cost = ba_cost(r, r_size);
  printf("Cost: %f\n", cost);

  ba_data_free(data);
  free(r);
  return 0;
}

int test_ba_jacobians() {
  /* Test */
  int J_rows = 0;
  int J_cols = 0;
  ba_data_t *data = ba_load_data(TEST_DATA);
  double *J = ba_jacobian(data, &J_rows, &J_cols);

  /* Write Jacobians to csv file */
  int idx = 0;
  FILE *csv_file = fopen("/tmp/J.csv", "w");
  for (int i = 0; i < J_rows; i++) {
    for (int j = 0; j < J_cols; j++) {
      fprintf(csv_file, "%f", J[idx]);
      idx++;
      if ((j + 1) != J_cols) {
        fprintf(csv_file, ",");
      }
    }
    fprintf(csv_file, "\n");
  }
  fclose(csv_file);

  /* Clean up */
  free(J);
  ba_data_free(data);

  return 0;
}

int test_ba_update() {
  ba_data_t *data = ba_load_data(TEST_DATA);

  int e_size = 0;
  double *e = ba_residuals(data, &e_size);

  int E_rows = 0;
  int E_cols = 0;
  double *E = ba_jacobian(data, &E_rows, &E_cols);

  /* H = (E' * W * E); */
  double *E_t = mat_new(E_cols, E_rows);
  double *H = mat_new(E_cols, E_cols);
  mat_transpose(E, E_rows, E_cols, E_t);
  dot(E_t, E_cols, E_rows, E, E_rows, E_cols, H);

  /* g = -E' * W * e; */
  double *g = vec_new(E_cols);
  mat_scale(E_t, E_cols, E_rows, -1.0);
  dot(E_t, E_cols, E_rows, e, e_size, 1, g);

  /* dx = pinv(H) * g; */
  double *H_inv = mat_new(E_cols, E_cols);
  double *dx = vec_new(E_cols);
  pinv(H, E_cols, E_cols, H_inv);
  dot(H_inv, E_cols, E_cols, g, E_cols, 1, dx);

  mat_save("/tmp/E.csv", E, E_rows, E_cols);
  mat_save("/tmp/E_t.csv", E_t, E_cols, E_rows);
  mat_save("/tmp/H.csv", H, E_cols, E_cols);
  mat_save("/tmp/g.csv", g, E_cols, 1);
  mat_save("/tmp/dx.csv", dx, E_cols, 1);

  ba_update(data, e, e_size, E, E_rows, E_cols);

  free(e);
  free(E);
  free(E_t);
  free(H);
  free(H_inv);
  free(g);
  ba_data_free(data);

  return 0;
}

int test_ba_cost() {
  const double e[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
  const double cost = ba_cost(e, 5);

  printf("Cost: %f\n", cost);
  MU_CHECK(fltcmp(cost, 27.50) == 0);

  return 0;
}

int test_ba_solve() {
  ba_data_t *data = ba_load_data(TEST_DATA);
  ba_solve(data);
  ba_data_free(data);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_parse_keypoints_line);
  MU_ADD_TEST(test_load_keypoints);
  MU_ADD_TEST(test_ba_load_data);
  MU_ADD_TEST(test_ba_residuals);
  MU_ADD_TEST(test_ba_jacobians);
  MU_ADD_TEST(test_ba_update);
  MU_ADD_TEST(test_ba_cost);
  MU_ADD_TEST(test_ba_solve);
}

MU_RUN_TESTS(test_suite);
