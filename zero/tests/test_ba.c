#include "zero/munit.h"
#include "zero/ba.h"

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

  ba_data_free(data);
	free(r);
	return 0;
}

void test_suite() {
  /* MU_ADD_TEST(test_parse_keypoints_line); */
  /* MU_ADD_TEST(test_load_keypoints); */
  /* MU_ADD_TEST(test_ba_load_data); */
  MU_ADD_TEST(test_ba_residuals);
}

MU_RUN_TESTS(test_suite);
