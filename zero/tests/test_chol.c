#include "zero/munit.h"
#include "zero/core.h"
#include "zero/chol.h"

int test_cholesky() {
  /* clang-format off */
  const int n = 3;
  double A[9] = {
    4.0, 12.0, -16.0,
    12.0, 37.0, -43.0,
    -16.0, -43.0, 98.0
  };
  /* clang-format on */

  struct timespec t = tic();
  double *L = cholesky(A, n);
  printf("time taken: [%fs]\n", toc(&t));

  double Lt[9] = {0};
  double LLt[9] = {0};
  mat_transpose(L, n, n, Lt);
  dot(L, n, n, Lt, n, n, LLt);

  int debug = 0;
  if (debug) {
    print_matrix("L", L, n, n);
    printf("\n");
    print_matrix("Lt", Lt, n, n);
    printf("\n");
    print_matrix("LLt", LLt, n, n);
    printf("\n");
    print_matrix("A", A, n, n);
  }

  int retval = mat_equals(A, LLt, n, n);
  MU_CHECK(retval == 0);
  free(L);

  return 0;
}

int test_chol_lls_solve() {
  /* clang-format off */
  const int n = 3;
  double A[9] = {
    2.0, -1.0, 0.0,
    -1.0, 2.0, -1.0,
    0.0, -1.0, 1.0
  };
  double b[3] = {1.0, 0.0, 0.0};
  double x[3] = {0.0, 0.0, 0.0};
  /* clang-format on */

  struct timespec t = tic();
  chol_lls_solve(A, b, x, n);
  printf("time taken: [%fs]\n", toc(&t));
  print_vector("x", x, n);

  MU_CHECK(fltcmp(x[0], 1.0) == 0);
  MU_CHECK(fltcmp(x[1], 1.0) == 0);
  MU_CHECK(fltcmp(x[2], 1.0) == 0);

  return 0;
}

int test_chol_lls_solve2() {
  /* clang-format off */
  /* const int n = 3; */
  /* double A[9] = { */
  /*   2.0, -1.0, 0.0, */
  /*   -1.0, 2.0, -1.0, */
  /*   0.0, -1.0, 1.0 */
  /* }; */
  /* double b[3] = {1.0, 0.0, 0.0}; */
  /* double x[3] = {0.0, 0.0, 0.0}; */
  /* clang-format on */

  /* clang-format off */
  int n = 4;
  double A[16] = {
    4.16, 0.0, 0.0, 0.0,
    -3.12, 5.03, 0.0, 0.0,
    0.56, -0.83, 0.76, 0.0,
    -0.10, 1.18,  0.34, 1.18
    /* 4.16, -3.12, 0.56, -0.10, */
    /* -3.12, 5.03, -0.83, 1.18, */
    /* 0.56, -0.83, 0.76, 0.34, */
    /* -0.10, 1.18,  0.34, 1.18 */
  };
  double b[4] = {1.0, 0.0, 0.0, 0.0};
  double x[4] = {0.0, 0.0, 0.0, 0.0};
  /* clang-format on */

  /* struct timespec t = tic(); */
  chol_lls_solve2(A, b, x, n);
  /* printf("time taken: [%fs]\n", toc(&t)); */
  /* print_vector("x", x, n); */

  /* MU_CHECK(fltcmp(x[0], 1.0) == 0); */
  /* MU_CHECK(fltcmp(x[1], 1.0) == 0); */
  /* MU_CHECK(fltcmp(x[2], 1.0) == 0); */

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_cholesky);
  MU_ADD_TEST(test_chol_lls_solve);
  MU_ADD_TEST(test_chol_lls_solve2);
}

MU_RUN_TESTS(test_suite);
