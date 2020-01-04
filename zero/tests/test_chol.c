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

void test_suite() { MU_ADD_TEST(test_cholesky); }

MU_RUN_TESTS(test_suite);
