#include "zero/munit.h"
#include "zero/core.h"
#include "zero/svd.h"

int test_dsvd() {
  double **A = malloc(sizeof(double *) * 3);
  A[0] = malloc(sizeof(double) * 2);
  A[1] = malloc(sizeof(double) * 2);
  A[2] = malloc(sizeof(double) * 2);
  /* clang-format off */
  A[0][0] = 0.41676; A[0][1] = 0.72068;
  A[1][0] = 0.28617; A[1][1] = 0.34818;
  A[2][0] = 0.49262; A[2][1] = 0.68185;
  /* clang-format on */

  int m = 3;
  int n = 2;

  double w[2] = {0};
  double **v = malloc(sizeof(double *) * 2);
  v[0] = malloc(sizeof(double) * 2);
  v[1] = malloc(sizeof(double) * 2);

  /* struct timespec t = tic(); */
  svd(A, m, n, w, v);
  /* printf("%fs\n", toc(&t)); */

  printf("A:\n");
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      printf("%f ", A[i][j]);
    }
    printf("\n");
  }

  printf("V:\n");
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      printf("%f ", v[i][j]);
    }
    printf("\n");
  }

  printf("w:\n");
  for (int i = 0; i < 2; i++) {
    printf("%f ", w[i]);
  }
  printf("\n");

  free(A[0]);
  free(A[1]);
  free(A[2]);
  free(A);

  free(v[0]);
  free(v[1]);
  free(v);

  return 0;
}

void test_suite() { MU_ADD_TEST(test_dsvd); }

MU_RUN_TESTS(test_suite);
