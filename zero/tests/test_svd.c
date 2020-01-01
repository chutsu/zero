#include "zero/munit.h"
#include "zero/core.h"
#include "zero/svd.h"

/* Parameters */
#define M 6
#define N 5
#define LDA M
#define LDU M
#define LDVT N

int test_svd() {
  int m = M;
  int n = N;
  double s[n];
  double U[m * m];
  double V_t[n * n];

  /* clang-format off */
  double A[LDA * N] = {
		0.914396, 0.363861, 0.776292, 0.189401, 0.444104,
		0.593250, 0.616198, 0.437685, 0.722506, 0.444178,
		0.121810, 0.365621, 0.385872, 0.152660, 0.977475,
		0.939700, 0.457225, 0.863963, 0.040427, 0.540278,
		0.216404, 0.414613, 0.327000, 0.915100, 0.568208,
		0.102671, 0.427741, 0.143185, 0.538898, 0.163511
	};
  /* clang-format on */

  struct timespec t = tic();
  int retval = svd(A, m, n, U, s, V_t);
  printf("time taken: %fs\n", toc(&t));
  if (retval > 0) {
    printf("The algorithm computing SVD failed to converge.\n");
    exit(1);
  }

  printf("\n");
  print_matrix("s", s, 1, n);
  printf("\n");

  print_matrix("U", U, m, n);
  printf("\n");

  print_matrix("V_t", V_t, n, n);
  printf("\n");

  return 0;
}

int test_pinv() { return 0; }

void test_suite() {
  MU_ADD_TEST(test_svd);
  MU_ADD_TEST(test_pinv);
}

MU_RUN_TESTS(test_suite);
