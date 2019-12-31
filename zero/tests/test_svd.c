#include "zero/munit.h"
#include "zero/core.h"
#include "zero/svd.h"

int test_dsvd() {
  double **a = malloc(sizeof(double *) * 3);
  a[0] = malloc(sizeof(double) * 3);
  a[1] = malloc(sizeof(double) * 3);
  a[2] = malloc(sizeof(double) * 3);
  /* a[0][0] = 2.0; */
  /* a[0][1] = 0.0; */
  /* a[1][0] = 0.0; */
  /* a[1][1] = -3.0; */
  /* a[2][0] = 0.0; */
  /* a[2][1] = 0.0; */

  a[0][0] = 0.41676;
  a[0][1] = 0.72068;
  a[1][0] = 0.28617;
  a[1][1] = 0.34818;
  a[2][0] = 0.49262;
  a[2][1] = 0.68185;

  printf("a:\n");
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      printf("%f ", a[i][j]);
    }
    printf("\n");
  }

  int m = 3;
  int n = 2;

  double w[2] = {0};
  double **v = malloc(sizeof(double *) * 2);
  v[0] = malloc(sizeof(double) * 2);
  v[1] = malloc(sizeof(double) * 2);

  /* struct timespec t = tic(); */
  dsvd(a, m, n, w, v);
  /* printf("%fs\n", toc(&t)); */

  printf("a:\n");
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      printf("%f ", a[i][j]);
    }
    printf("\n");
  }

  printf("v:\n");
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

  free(a[0]);
  free(a[1]);
  free(a[2]);
  free(a);

  free(v[0]);
  free(v[1]);
  free(v);

  return 0;
}

void test_suite() { MU_ADD_TEST(test_dsvd); }

MU_RUN_TESTS(test_suite);
