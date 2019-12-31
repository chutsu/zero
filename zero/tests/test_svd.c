#include "zero/munit.h"
#include "zero/core.h"
#include "zero/svd.h"

int test_dsvd() {
  /* clang-format off */
  double A[6] = {
		0.41676, 0.72068,
		0.28617, 0.34818,
		0.49262, 0.68185
	};
  /* clang-format on */

  int m = 3;
  int n = 2;

  double w[2] = {0};
  double V[4] = {0};

  struct timespec t = tic();
  svd(A, m, n, w, V);
  printf("%fs\n", toc(&t));

  printf("A:\n");
  int index = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      printf("%f ", A[index]);
      index++;
    }
    printf("\n");
  }
  printf("\n");

  printf("V:\n");
  index = 0;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      printf("%f ", V[index]);
      index++;
    }
    printf("\n");
  }
  printf("\n");

  printf("w:\n");
  for (int i = 0; i < 2; i++) {
    printf("%f ", w[i]);
  }
  printf("\n");

  return 0;
}

void test_suite() { MU_ADD_TEST(test_dsvd); }

MU_RUN_TESTS(test_suite);
