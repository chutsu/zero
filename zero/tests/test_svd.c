#include "zero/munit.h"
#include "zero/core.h"
#include "zero/svd.h"

/* Parameters */
#define M 2
#define N 2
#define LDA M
#define LDU M
#define LDVT N

int test_svd() {
  double A[M * N];
  double A_orig[M * N];
  for (int i = 0; i < (M * N); i++) {
    A[i] = randf(0.0, 1.0);
    A_orig[i] = A[i];
  }

  struct timespec t = tic();
  double U[M * M];
  double d[N];
  double V_t[N * N];
  int retval = svd(A, M, N, U, d, V_t);
  printf("time taken: %fs\n", toc(&t));
  if (retval != 0) {
    printf("The algorithm computing SVD failed to converge.\n");
    exit(1);
  }

  /* A = U * S * V_t */
  double S[N * N];
  mat_diag_set(S, N, N, d);

  double US[M * N];
  double USV[M * M];
  dot(U, M, N, S, N, N, US);
  dot(US, M, N, V_t, N, N, USV);

  print_matrix("A", A_orig, M, N);
  printf("\n");
  print_matrix("USV'", USV, M, N);
  MU_CHECK(mat_equal(A_orig, USV, M, N) == 0);

  return 0;
}

int test_svdcmp() {
  double A[M * N];
  double A_orig[M * N];
  for (int i = 0; i < (M * N); i++) {
    A[i] = randf(0.0, 1.0);
    A_orig[i] = A[i];
  }

  double d[N];
  double V[N * N];
  struct timespec t = tic();
  int retval = svdcmp(A, M, N, d, V);
  printf("time taken: %fs\n", toc(&t));
  if (retval != 0) {
    printf("The algorithm computing SVD failed to converge.\n");
    exit(1);
  }

  /* A = U * S * V_t */
  double S[N * N];
  mat_diag_set(S, N, N, d);

  double US[M * N];
  double USV[M * M];
  double V_t[N * N];
  mat_transpose(V, N, N, V_t);
  dot(A, M, N, S, N, N, US);
  dot(US, M, N, V_t, N, N, USV);

  print_matrix("A", A_orig, M, N);
  printf("\n");
  print_matrix("USV", USV, M, N);
  MU_CHECK(mat_equal(A_orig, USV, M, N) == 0);

  return 0;
}

int test_pinv() { return 0; }

void test_suite() {
  MU_ADD_TEST(test_svd);
  MU_ADD_TEST(test_svdcmp);
  /* MU_ADD_TEST(test_pinv); */
}

MU_RUN_TESTS(test_suite);
