#include "munit.h"
#include "zero/zero.h"

/* TEST PARAMS */
#define M 10
#define N 10
#define TEST_CSV "zero/tests/test_data/test_csv.csv"
#define TEST_POSES_CSV "zero/tests/test_data/poses.csv"

int test_debug() {
  DEBUG("Hello World!");
  return 0;
}

int test_log_error() {
  LOG_ERROR("Hello World!");
  return 0;
}

int test_log_warn() {
  LOG_WARN("Hello World!");
  return 0;
}

int test_malloc_string() {
  char *s = malloc_string("hello world!");
  MU_CHECK(strcmp(s, "hello world!") == 0);
  return 0;
}

int test_dsv_rows() {
  int nb_rows = dsv_rows(TEST_CSV);
  MU_CHECK(nb_rows == 10);
  return 0;
}

int test_dsv_cols() {
  int nb_cols = dsv_cols(TEST_CSV, ',');
  MU_CHECK(nb_cols == 10);
  return 0;
}

int test_dsv_fields() {
  int nb_fields = 0;
  char **fields = dsv_fields(TEST_CSV, ',', &nb_fields);
  const char *expected[10] = {"a", "b", "c", "d", "e", "f", "g", "h", "i", "j"};

  MU_CHECK(nb_fields == 10);
  for (int i = 0; i < nb_fields; i++) {
    /* printf("field[%d]: %s\n", i, fields[i]); */
    MU_CHECK(strcmp(fields[i], expected[i]) == 0);
    free(fields[i]);
  }
  free(fields);

  return 0;
}

int test_dsv_data() {
  int nb_rows = 0;
  int nb_cols = 0;
  real_t **data = dsv_data(TEST_CSV, ',', &nb_rows, &nb_cols);

  int index = 0;
  for (int i = 0; i < nb_rows; i++) {
    for (int j = 0; j < nb_rows; j++) {
      MU_CHECK(fltcmp(data[i][j], index + 1) == 0);
      index++;

      /* printf("%f ", data[i][j]); */
    }
    /* printf("\n"); */
  }

  return 0;
}

int test_eye() {
  real_t A[25] = {0.0};
  eye(A, 5, 5);

  /* print_matrix("I", A, 5, 5); */
  size_t idx = 0;
  size_t rows = 5;
  size_t cols = 5;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      real_t expected = (i == j) ? 1.0 : 0.0;
      MU_CHECK(fltcmp(A[idx], expected) == 0);
      idx++;
    }
  }

  return 0;
}

int test_ones() {
  real_t A[25] = {0.0};
  ones(A, 5, 5);

  /* print_matrix("A", A, 5, 5); */
  size_t idx = 0;
  size_t rows = 5;
  size_t cols = 5;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      MU_CHECK((fabs(A[idx] - 1.0) < 1e-5));
      idx++;
    }
  }

  return 0;
}

int test_zeros() {
  real_t A[25] = {0.0};
  zeros(A, 5, 5);

  /* print_matrix("A", A, 5, 5); */
  size_t idx = 0;
  size_t rows = 5;
  size_t cols = 5;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      MU_CHECK((fabs(A[idx] - 0.0) < 1e-5));
      idx++;
    }
  }

  return 0;
}

int test_mat_set() {
  real_t A[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  mat_set(A, 3, 0, 0, 1.0);
  mat_set(A, 3, 1, 1, 1.0);
  mat_set(A, 3, 2, 2, 1.0);

  /* print_matrix("A", A, 3, 3); */
  MU_CHECK(fltcmp(mat_val(A, 3, 0, 0), 1.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 1, 1), 1.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 2, 2), 1.0) == 0);

  return 0;
}

int test_mat_val() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};

  /* print_matrix("A", A, 3, 3); */
  MU_CHECK(fltcmp(mat_val(A, 3, 0, 0), 1.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 0, 1), 2.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 0, 2), 3.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 1, 0), 4.0) == 0);

  return 0;
}

int test_mat_block_get() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[4] = {0.0};
  mat_block_get(A, 3, 1, 1, 2, 2, B);

  print_matrix("A", A, 3, 3);
  print_matrix("B", B, 2, 2);
  MU_CHECK(fltcmp(mat_val(B, 2, 0, 0), 5.0) == 0);
  MU_CHECK(fltcmp(mat_val(B, 2, 0, 1), 6.0) == 0);
  MU_CHECK(fltcmp(mat_val(B, 2, 1, 0), 8.0) == 0);
  MU_CHECK(fltcmp(mat_val(B, 2, 1, 1), 9.0) == 0);

  return 0;
}

int test_mat_block_set() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[4] = {0.0, 0.0, 0.0, 0.0};
  mat_block_set(A, 3, 1, 1, 2, 2, B);

  print_matrix("B", B, 2, 2);
  print_matrix("A", A, 3, 3);
  MU_CHECK(fltcmp(mat_val(A, 3, 1, 1), 0.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 1, 2), 0.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 2, 1), 0.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 2, 2), 0.0) == 0);

  return 0;
}

int test_mat_diag_get() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t d[3] = {0.0, 0.0, 0.0};
  mat_diag_get(A, 3, 3, d);

  print_matrix("A", A, 3, 3);
  print_vector("d", d, 3);
  MU_CHECK(fltcmp(d[0], 1.0) == 0);
  MU_CHECK(fltcmp(d[1], 5.0) == 0);
  MU_CHECK(fltcmp(d[2], 9.0) == 0);

  return 0;
}

int test_mat_diag_set() {
  real_t A[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  real_t d[4] = {1.0, 2.0, 3.0};
  mat_diag_set(A, 3, 3, d);

  print_matrix("A", A, 3, 3);
  MU_CHECK(fltcmp(mat_val(A, 3, 0, 0), 1.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 1, 1), 2.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 2, 2), 3.0) == 0);

  return 0;
}

int test_mat_triu() {
  /* clang-format off */
  real_t A[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  real_t U[16] = {0};
  /* clang-format on */
  mat_triu(A, 4, U);
  print_matrix("U", U, 4, 4);

  return 0;
}

int test_mat_tril() {
  /* clang-format off */
  real_t A[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  real_t L[16] = {0};
  /* clang-format on */
  mat_tril(A, 4, L);
  print_matrix("L", L, 4, 4);

  return 0;
}

int test_mat_trace() {
  /* clang-format off */
  real_t A[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  /* clang-format on */
  const real_t tr = mat_trace(A, 4, 4);
  MU_CHECK(fltcmp(tr, 1.0 + 6.0 + 11.0 + 16.0) == 0.0);

  return 0;
}

int test_mat_transpose() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t C[9] = {0.0};
  mat_transpose(A, 3, 3, C);
  print_matrix("C", C, 3, 3);

  real_t B[3*4] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0};
  real_t D[9] = {0.0};
  print_matrix("B", B, 3, 4);
  mat_transpose(B, 3, 4, D);
  print_matrix("D", D, 4, 3);

  return 0;
}

int test_mat_add() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0};
  real_t C[9] = {0.0};
  mat_add(A, B, C, 3, 3);
  print_matrix("C", C, 3, 3);

  return 0;
}

int test_mat_sub() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t C[9] = {0.0};
  mat_sub(A, B, C, 3, 3);
  print_matrix("C", C, 3, 3);

  return 0;
}

int test_mat_scale() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  mat_scale(A, 3, 3, 2.0);
  print_matrix("A", A, 3, 3);

  return 0;
}

int test_vec_add() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0};
  real_t C[9] = {0.0};
  vec_add(A, B, C, 9);
  print_vector("C", C, 9);

  return 0;
}

int test_vec_sub() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t C[9] = {0.0};
  vec_sub(A, B, C, 9);
  print_vector("C", C, 9);

  return 0;
}

/* void dot(const real_t *A, const size_t A_m, const size_t A_n, */
/*          const real_t *B, const size_t B_m, const size_t B_n, */
/*          real_t *C) { */
/*   assert(A_n == B_m); */
/*  */
/*   cblas_dgemm( */
/*     CblasRowMajor, #<{(| Matrix data arrangement |)}># */
/*     CblasNoTrans,  #<{(| Transpose A |)}># */
/*     CblasNoTrans,  #<{(| Transpose B |)}># */
/*     A_m,           #<{(| Number of rows in A and C |)}># */
/*     B_n,           #<{(| Number of cols in B and C |)}># */
/*     A_n,           #<{(| Number of cols in A |)}># */
/*     1.0,           #<{(| Scaling factor for the product of A and B |)}># */
/*     A,             #<{(| Matrix A |)}># */
/*     A_n,           #<{(| First dimension of A |)}># */
/*     B,             #<{(| Matrix B |)}># */
/*     B_n,           #<{(| First dimension of B |)}># */
/*     1.0,           #<{(| Scale factor for C |)}># */
/*     C,             #<{(| Output |)}># */
/*     A_m            #<{(| First dimension of C |)}># */
/*   ); */
/* } */

int test_dot() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[3] = {1.0, 2.0, 3.0};
  real_t C[3] = {0.0};

  /* Multiply matrix A and B */
  dot(A, 3, 3, B, 3, 1, C);
  print_vector("C", C, 3);

  MU_CHECK(fltcmp(C[0], 14.0) == 0);
  MU_CHECK(fltcmp(C[1], 32.0) == 0);
  MU_CHECK(fltcmp(C[2], 50.0) == 0);

  return 0;
}

int test_skew() {
  real_t x[3] = {1.0, 2.0, 3.0};
  real_t S[3 * 3] = {0};

  skew(x, S);
  print_matrix("S", S, 3, 3);

  MU_CHECK(fltcmp(S[0], 0.0) == 0);
  MU_CHECK(fltcmp(S[1], -3.0) == 0);
  MU_CHECK(fltcmp(S[2], 2.0) == 0);

  MU_CHECK(fltcmp(S[3], 3.0) == 0);
  MU_CHECK(fltcmp(S[4], 0.0) == 0);
  MU_CHECK(fltcmp(S[5], -1.0) == 0);

  MU_CHECK(fltcmp(S[6], -2.0) == 0);
  MU_CHECK(fltcmp(S[7], 1.0) == 0);
  MU_CHECK(fltcmp(S[8], 0.0) == 0);

  return 0;
}

int test_check_jacobian() {
  const size_t m = 2;
  const size_t n = 3;
  const real_t threshold = 1e-6;
  const int print = 1;

  // Positive test
  {
    const real_t fdiff[6] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    const real_t jac[6] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    int retval = check_jacobian("test_check_jacobian",
                                fdiff,
                                jac,
                                m,
                                n,
                                threshold,
                                print);
    MU_CHECK(retval == 0);
  }

  // Negative test
  {
    const real_t fdiff[6] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    const real_t jac[6] = {0.0, 1.0, 2.0, 3.1, 4.0, 5.0};
    int retval = check_jacobian("test_check_jacobian",
                                fdiff,
                                jac,
                                m,
                                n,
                                threshold,
                                print);
    MU_CHECK(retval == -1);
  }

  return 0;
}

int test_svd() {
  real_t A[M * N];
  real_t A_orig[M * N];
  for (int i = 0; i < (M * N); i++) {
    A[i] = randf(0.0, 1.0);
    A_orig[i] = A[i];
  }

  real_t d[N];
  real_t V[N * N];
  struct timespec t = tic();
  int retval = svd(A, M, N, d, V);
  printf("time taken: %fs\n", toc(&t));
  if (retval != 0) {
    printf("The algorithm computing SVD failed to converge.\n");
    exit(1);
  }

  /* A = U * S * V_t */
  real_t S[N * N];
  mat_diag_set(S, N, N, d);

  real_t US[M * N];
  real_t USV[M * M];
  real_t V_t[N * N];
  mat_transpose(V, N, N, V_t);
  dot(A, M, N, S, N, N, US);
  dot(US, M, N, V_t, N, N, USV);

  print_matrix("A", A_orig, M, N);
  printf("\n");
  print_matrix("USV", USV, M, N);
  MU_CHECK(mat_equals(A_orig, USV, M, N, 1e-5) == 0);

  return 0;
}

int test_chol() {
  /* clang-format off */
  const int n = 3;
  real_t A[9] = {
    4.0, 12.0, -16.0,
    12.0, 37.0, -43.0,
    -16.0, -43.0, 98.0
  };
  /* clang-format on */

  struct timespec t = tic();
  real_t L[9] = {0};
  chol(A, n, L);
  printf("time taken: [%fs]\n", toc(&t));

  real_t Lt[9] = {0};
  real_t LLt[9] = {0};
  mat_transpose(L, n, n, Lt);
  dot(L, n, n, Lt, n, n, LLt);

  int debug = 1;
  /* int debug = 0; */
  if (debug) {
    print_matrix("L", L, n, n);
    printf("\n");
    print_matrix("Lt", Lt, n, n);
    printf("\n");
    print_matrix("LLt", LLt, n, n);
    printf("\n");
    print_matrix("A", A, n, n);
  }

  int retval = mat_equals(A, LLt, n, n, 1e-5);
  MU_CHECK(retval == 0);

  return 0;
}

int test_chol_solve() {
  /* clang-format off */
  const int n = 3;
  real_t A[9] = {
    2.0, -1.0, 0.0,
    -1.0, 2.0, -1.0,
    0.0, -1.0, 1.0
  };
  real_t b[3] = {1.0, 0.0, 0.0};
  real_t x[3] = {0.0, 0.0, 0.0};
  /* clang-format on */

  struct timespec t = tic();
  chol_solve(A, b, x, n);
  printf("time taken: [%fs]\n", toc(&t));
  print_vector("x", x, n);

  MU_CHECK(fltcmp(x[0], 1.0) == 0);
  MU_CHECK(fltcmp(x[1], 1.0) == 0);
  MU_CHECK(fltcmp(x[2], 1.0) == 0);

  return 0;
}

#ifdef USE_LAPACK
int test_chol_solve2() {
  /* #<{(| clang-format off |)}># */
  /* const int m = 3; */
  /* const real_t A[9] = { */
  /*   2.0, -1.0, 0.0, */
  /*   -1.0, 2.0, -1.0, */
  /*   0.0, -1.0, 1.0 */
  /* }; */
  /* const real_t b[3] = {1.0, 0.0, 0.0}; */
  /* real_t x[3] = {0.0, 0.0, 0.0}; */
  /* #<{(| clang-format on |)}># */

  /* real_t a[9] = { 1.0, .6, .3, .6, 1., .5, .3, .5, 1 }; */
  /* print_matrix("a", a, 3, 3); */
  /* int retval = LAPACKE_dpotrf(LAPACK_ROW_MAJOR, 'L', 3, a, 3); */
  /* if (retval != 0) { */
  /*   fprintf(stderr, "Failed to decompose A using Cholesky Decomposition!\n");
   */
  /* } */
  /* print_matrix("a", a, 3, 3); */
  /* mat_save("/tmp/A.csv", A, m, m); */

  /* clang-format off */
  int m = 4;
  real_t A[16] = {
    4.16, -3.12, 0.56, -0.10,
    -3.12, 5.03, -0.83, 1.18,
    0.56, -0.83, 0.76, 0.34,
    -0.10, 1.18,  0.34, 1.18
  };
  real_t b[4] = {1.0, 0.0, 0.0, 0.0};
  real_t x[4] = {0.0, 0.0, 0.0, 0.0};
  /* clang-format on */

  struct timespec t = tic();
  lapack_chol_solve(A, b, x, m);
  /* OCTAVE_SCRIPT("scripts/plot_matrix.m /tmp/A.csv"); */
  printf("time taken: [%fs]\n", toc(&t));
  print_vector("x", x, m);

  /* MU_CHECK(fltcmp(x[0], 1.0) == 0); */
  /* MU_CHECK(fltcmp(x[1], 1.0) == 0); */
  /* MU_CHECK(fltcmp(x[2], 1.0) == 0); */

  return 0;
}
#endif

int test_tf_rot_set() {
  real_t C[9];
  for (int i = 0; i < 9; i++) {
    C[i] = 1.0;
  }

  real_t T[16] = {0.0};
  tf_rot_set(T, C);
  /* print_matrix("T", T, 4, 4); */

  MU_CHECK(fltcmp(T[0], 1.0) == 0);
  MU_CHECK(fltcmp(T[1], 1.0) == 0);
  MU_CHECK(fltcmp(T[2], 1.0) == 0);
  MU_CHECK(fltcmp(T[3], 0.0) == 0);

  MU_CHECK(fltcmp(T[4], 1.0) == 0);
  MU_CHECK(fltcmp(T[5], 1.0) == 0);
  MU_CHECK(fltcmp(T[6], 1.0) == 0);
  MU_CHECK(fltcmp(T[7], 0.0) == 0);

  MU_CHECK(fltcmp(T[8], 1.0) == 0);
  MU_CHECK(fltcmp(T[9], 1.0) == 0);
  MU_CHECK(fltcmp(T[10], 1.0) == 0);
  MU_CHECK(fltcmp(T[11], 0.0) == 0);

  MU_CHECK(fltcmp(T[12], 0.0) == 0);
  MU_CHECK(fltcmp(T[13], 0.0) == 0);
  MU_CHECK(fltcmp(T[14], 0.0) == 0);
  MU_CHECK(fltcmp(T[15], 0.0) == 0);

  return 0;
}

int test_tf_trans_set() {
  real_t r[3] = {1.0, 2.0, 3.0};

  real_t T[16] = {0.0};
  tf_trans_set(T, r);
  /* print_matrix("T", T, 4, 4); */

  MU_CHECK(fltcmp(T[0], 0.0) == 0);
  MU_CHECK(fltcmp(T[1], 0.0) == 0);
  MU_CHECK(fltcmp(T[2], 0.0) == 0);
  MU_CHECK(fltcmp(T[3], 1.0) == 0);

  MU_CHECK(fltcmp(T[4], 0.0) == 0);
  MU_CHECK(fltcmp(T[5], 0.0) == 0);
  MU_CHECK(fltcmp(T[6], 0.0) == 0);
  MU_CHECK(fltcmp(T[7], 2.0) == 0);

  MU_CHECK(fltcmp(T[8], 0.0) == 0);
  MU_CHECK(fltcmp(T[9], 0.0) == 0);
  MU_CHECK(fltcmp(T[10], 0.0) == 0);
  MU_CHECK(fltcmp(T[11], 3.0) == 0);

  MU_CHECK(fltcmp(T[12], 0.0) == 0);
  MU_CHECK(fltcmp(T[13], 0.0) == 0);
  MU_CHECK(fltcmp(T[14], 0.0) == 0);
  MU_CHECK(fltcmp(T[15], 0.0) == 0);

  return 0;
}

int test_tf_trans_get() {
  /* clang-format off */
  real_t T[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  /* clang-format on */
  print_matrix("T", T, 4, 4);

  /* Get translation vector */
  real_t r[3];
  tf_trans_get(T, r);
  print_vector("r", r, 3);

  MU_CHECK(fltcmp(r[0], 4.0) == 0);
  MU_CHECK(fltcmp(r[1], 8.0) == 0);
  MU_CHECK(fltcmp(r[2], 12.0) == 0);

  return 0;
}

int test_tf_rot_get() {
  /* Transform */
  /* clang-format off */
  real_t T[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  /* clang-format on */
  print_matrix("T", T, 4, 4);

  /* Get rotation matrix */
  real_t C[9];
  tf_rot_get(T, C);
  print_matrix("C", C, 3, 3);

  MU_CHECK(fltcmp(C[0], 1.0) == 0);
  MU_CHECK(fltcmp(C[1], 2.0) == 0);
  MU_CHECK(fltcmp(C[2], 3.0) == 0);

  MU_CHECK(fltcmp(C[3], 5.0) == 0);
  MU_CHECK(fltcmp(C[4], 6.0) == 0);
  MU_CHECK(fltcmp(C[5], 7.0) == 0);

  MU_CHECK(fltcmp(C[6], 9.0) == 0);
  MU_CHECK(fltcmp(C[7], 10.0) == 0);
  MU_CHECK(fltcmp(C[8], 11.0) == 0);

  return 0;
}

int test_tf_quat_get() {
  /* Transform */
  /* clang-format off */
  real_t T[16] = {1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 1.0};
  /* clang-format on */

  /* Create rotation matrix */
  const real_t euler[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
  real_t C[9] = {0};
  euler321(euler, C);
  tf_rot_set(T, C);

  /* Extract quaternion from transform */
  real_t q[4] = {0};
  tf_quat_get(T, q);

  /* Convert quaternion back to euler angles */
  real_t rpy[3] = {0};
  quat2euler(q, rpy);

  MU_CHECK(fltcmp(rad2deg(rpy[0]), 10.0) == 0);
  MU_CHECK(fltcmp(rad2deg(rpy[1]), 20.0) == 0);
  MU_CHECK(fltcmp(rad2deg(rpy[2]), 30.0) == 0);

  return 0;
}

int test_tf_inv() {
  /* Create Transform */
  /* clang-format off */
  real_t T[16] = {1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 1.0};
  /* clang-format on */
  /* -- Set rotation component */
  const real_t euler[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
  real_t C[9] = {0};
  euler321(euler, C);
  tf_rot_set(T, C);
  /* -- Set translation component */
  real_t r[3] = {1.0, 2.0, 3.0};
  tf_trans_set(T, r);
  print_matrix("T", T, 4, 4);
  printf("\n");

  /* Invert transform */
  real_t T_inv[16] = {0};
  tf_inv(T, T_inv);
  print_matrix("T_inv", T_inv, 4, 4);
  printf("\n");

  /* real_t Invert transform */
  real_t T_inv_inv[16] = {0};
  tf_inv(T_inv, T_inv_inv);
  print_matrix("T_inv_inv", T_inv_inv, 4, 4);

  /* Assert */
  int idx = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      MU_CHECK(fltcmp(T_inv_inv[idx], T[idx]) == 0);
    }
  }

  return 0;
}

int test_tf_point() {
  /* Transform */
  /* clang-format off */
  real_t T[16] = {1.0, 0.0, 0.0, 1.0,
                  0.0, 1.0, 0.0, 2.0,
                  0.0, 0.0, 1.0, 3.0,
                  0.0, 0.0, 0.0, 1.0};
  /* clang-format on */
  print_matrix("T", T, 4, 4);

  /* Point */
  real_t p[3] = {1.0, 2.0, 3.0};
  print_vector("p", p, 3);

  /* Transform point */
  real_t result[3] = {0};
  tf_point(T, p, result);
  print_vector("result", result, 3);

  return 0;
}

int test_tf_hpoint() {
  /* Transform */
  /* clang-format off */
  real_t T[16] = {1.0, 0.0, 0.0, 1.0,
                  0.0, 1.0, 0.0, 2.0,
                  0.0, 0.0, 1.0, 3.0,
                  0.0, 0.0, 0.0, 1.0};
  /* clang-format on */
  print_matrix("T", T, 4, 4);

  /* Homogeneous point */
  real_t hp[4] = {1.0, 2.0, 3.0, 1.0};
  print_vector("hp", hp, 4);

  /* Transform homogeneous point */
  real_t result[4] = {0};
  tf_hpoint(T, hp, result);
  print_vector("result", result, 4);

  return 0;
}

int test_tf_perturb_rot() {
  /* Transform */
  /* clang-format off */
  real_t T[4 * 4] = {1.0, 0.0, 0.0, 1.0,
                     0.0, 1.0, 0.0, 2.0,
                     0.0, 0.0, 1.0, 3.0,
                     0.0, 0.0, 0.0, 1.0};
  /* clang-format on */

  /* Perturb rotation */
  const real_t step_size = 1e-2;
  tf_perturb_rot(T, step_size, 0);
  print_matrix("T", T, 4, 4);

  /* Assert */
  MU_CHECK(fltcmp(T[0], 1.0) == 0);
  MU_CHECK(fltcmp(T[5], 1.0) != 0);
  MU_CHECK(fltcmp(T[10], 1.0) != 0);

  return 0;
}

int test_tf_perturb_trans() {
  /* Transform */
  /* clang-format off */
  real_t T[4 * 4] = {1.0, 0.0, 0.0, 1.0,
                     0.0, 1.0, 0.0, 2.0,
                     0.0, 0.0, 1.0, 3.0,
                     0.0, 0.0, 0.0, 1.0};
  /* clang-format on */

  /* Perturb translation */
  const real_t step_size = 1e-2;
  tf_perturb_trans(T, step_size, 0);
  print_matrix("T", T, 4, 4);

  /* Assert */
  MU_CHECK(fltcmp(T[3], 1.01) == 0);
  MU_CHECK(fltcmp(T[7], 2.0) == 0);
  MU_CHECK(fltcmp(T[11], 3.0) == 0);

  return 0;
}

int test_euler321() {
  /* Euler to rotation matrix */
  const real_t euler[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
  real_t C[9] = {0};
  euler321(euler, C);

  /* Rotation matrix to quaternion */
  real_t q[4] = {0};
  rot2quat(C, q);

  /* Quaternion to Euler angles*/
  real_t euler2[3] = {0};
  quat2euler(q, euler2);

  print_vector("euler", euler, 3);
  print_vector("euler2", euler2, 3);

  return 0;
}

int test_rot2quat() {
  /* Rotation matrix to quaternion */
  const real_t C[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  real_t q[4] = {0.0};
  rot2quat(C, q);
  print_vector("q", q, 4);

  return 0;
}

int test_quat2euler() {
  const real_t C[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  /* Rotation matrix to quaternion */
  real_t q[4] = {0.0};
  rot2quat(C, q);
  print_vector("q", q, 4);

  /* Quaternion to Euler angles */
  real_t rpy[3] = {0.0};
  quat2euler(q, rpy);
  print_vector("euler", rpy, 3);

  return 0;
}

int test_quat2rot() {
  /* Euler to rotation matrix */
  const real_t euler[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
  real_t C[9] = {0};
  euler321(euler, C);

  /* Rotation matrix to quaternion */
  real_t q[4] = {0.0};
  rot2quat(C, q);
  /* print_vector("q", q, 4); */

  /* Quaternion to rotation matrix */
  real_t rot[9] = {0.0};
  quat2rot(q, rot);

  for (int i = 0; i < 9; i++) {
    MU_CHECK(fltcmp(C[i], rot[i]) == 0);
  }

  return 0;
}

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

int test_speed_biases_setup() {
  uint64_t param_id = 0;
  timestamp_t ts = 1;
  speed_biases_t sb;

  real_t data[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  speed_biases_setup(&sb, &param_id, ts, data);

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

int test_feature_setup() {
  uint64_t param_id = 0;
  feature_t feature;

  real_t data[3] = {0.1, 0.2, 0.3};
  feature_setup(&feature, &param_id, data);

  MU_CHECK(param_id == 1);
  MU_CHECK(feature.param_id == 0);

  MU_CHECK(fltcmp(feature.data[0], 0.1) == 0.0);
  MU_CHECK(fltcmp(feature.data[1], 0.2) == 0.0);
  MU_CHECK(fltcmp(feature.data[2], 0.3) == 0.0);

  return 0;
}

int test_extrinsics_setup() {
  uint64_t param_id = 0;
  extrinsics_t extrinsics;

  real_t data[7] = {1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3};
  extrinsics_setup(&extrinsics, &param_id, data);

  MU_CHECK(param_id == 1);
  MU_CHECK(extrinsics.param_id == 0);

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
  uint64_t param_id = 0;

  camera_t camera;
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t data[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_setup(&camera,
               &param_id,
               cam_idx, cam_res,
               proj_model, dist_model,
               data);

  camera_print(&camera);

  return 0;
}

int test_pose_factor_setup() {
  uint64_t param_id = 0;

  timestamp_t ts = 1;
  pose_t pose;
  real_t data[7] = {1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3};
  pose_setup(&pose, &param_id, ts, data);

  pose_factor_t pose_factor;
  real_t var[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  pose_factor_setup(&pose_factor, &pose, var);

  print_matrix("pose_factor.covar", pose_factor.covar, 6, 6);
  print_matrix("pose_factor.r", pose_factor.r, 6, 1);
  print_matrix("pose_factor.J0", pose_factor.J0, 6, 6);

  return 0;
}

int test_pose_factor_eval() {
  /* pose_t pose; */

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

int test_cam_factor_setup() {
  uint64_t param_id = 0;

  timestamp_t ts = 0;
  pose_t pose;
  {
    real_t data[7] = {1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3};
    pose_setup(&pose, &param_id, ts, data);
  }

  extrinsics_t extrinsics;
  {
    real_t data[7] = {1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3};
    extrinsics_setup(&extrinsics, &param_id, data);
  }

  camera_t camera;
  {
    const int cam_idx = 0;
    const int cam_res[2] = {752, 480};
    const char *proj_model = "pinhole";
    const char *dist_model = "radtan4";
    const real_t data[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
    camera_setup(&camera,
                &param_id,
                cam_idx, cam_res,
                proj_model, dist_model,
                data);
  }

  cam_factor_t cam_factor;
  real_t var[2] = {1.0, 1.0};
  cam_factor_setup(&cam_factor, &pose, &extrinsics, &camera, var);

  print_matrix("cam_factor.covar", cam_factor.covar, 2, 2);
  print_matrix("cam_factor.r", cam_factor.r, 2, 1);
  print_matrix("cam_factor.J0", cam_factor.J0, 2, 6);
  print_matrix("cam_factor.J1", cam_factor.J1, 2, 6);
  print_matrix("cam_factor.J2", cam_factor.J2, 2, 8);
  print_matrix("cam_factor.J3", cam_factor.J3, 2, 3);

  return 0;
}

int test_cam_factor_eval() {
  uint64_t param_id = 0;

  timestamp_t ts = 0;
  pose_t pose;
  {
    real_t data[7] = {1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3};
    pose_setup(&pose, &param_id, ts, data);
  }

  extrinsics_t extrinsics;
  {
    real_t data[7] = {1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3};
    extrinsics_setup(&extrinsics, &param_id, data);
  }

  camera_t camera;
  {
    const int cam_idx = 0;
    const int cam_res[2] = {752, 480};
    const char *proj_model = "pinhole";
    const char *dist_model = "radtan4";
    const real_t data[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
    camera_setup(&camera,
                &param_id,
                cam_idx, cam_res,
                proj_model, dist_model,
                data);
  }

  cam_factor_t cam_factor;
  real_t var[2] = {1.0, 1.0};
  cam_factor_setup(&cam_factor, &pose, &extrinsics, &camera, var);
  cam_factor_eval(&cam_factor);

  return 0;
}

int test_imu_buf_setup() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  return 0;
}

int test_imu_buf_add() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buf_add(&imu_buf, ts, acc, gyr);
  imu_buf_print(&imu_buf);

  MU_CHECK(imu_buf.size == 1);
  MU_CHECK(imu_buf.ts[0] == ts);
  MU_CHECK(fltcmp(imu_buf.acc[0][0], 1.0) == 0);
  MU_CHECK(fltcmp(imu_buf.acc[0][1], 2.0) == 0);
  MU_CHECK(fltcmp(imu_buf.acc[0][2], 3.0) == 0);
  MU_CHECK(fltcmp(imu_buf.gyr[0][0], 1.0) == 0);
  MU_CHECK(fltcmp(imu_buf.gyr[0][1], 2.0) == 0);
  MU_CHECK(fltcmp(imu_buf.gyr[0][2], 3.0) == 0);

  return 0;
}

int test_imu_buf_clear() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buf_add(&imu_buf, ts, acc, gyr);
  imu_buf_clear(&imu_buf);

  MU_CHECK(imu_buf.size == 0);
  MU_CHECK(imu_buf.ts[0] == 0);
  MU_CHECK(fltcmp(imu_buf.acc[0][0], 0.0) == 0);
  MU_CHECK(fltcmp(imu_buf.acc[0][1], 0.0) == 0);
  MU_CHECK(fltcmp(imu_buf.acc[0][2], 0.0) == 0);
  MU_CHECK(fltcmp(imu_buf.gyr[0][0], 0.0) == 0);
  MU_CHECK(fltcmp(imu_buf.gyr[0][1], 0.0) == 0);
  MU_CHECK(fltcmp(imu_buf.gyr[0][2], 0.0) == 0);

  return 0;
}

int test_imu_buf_copy() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buf_add(&imu_buf, ts, acc, gyr);

  imu_buf_t imu_buf2;
  imu_buf_setup(&imu_buf2);
  imu_buf_copy(&imu_buf, &imu_buf2);

  MU_CHECK(imu_buf2.size == 1);
  MU_CHECK(imu_buf2.ts[0] == ts);
  MU_CHECK(fltcmp(imu_buf2.acc[0][0], 1.0) == 0);
  MU_CHECK(fltcmp(imu_buf2.acc[0][1], 2.0) == 0);
  MU_CHECK(fltcmp(imu_buf2.acc[0][2], 3.0) == 0);
  MU_CHECK(fltcmp(imu_buf2.gyr[0][0], 1.0) == 0);
  MU_CHECK(fltcmp(imu_buf2.gyr[0][1], 2.0) == 0);
  MU_CHECK(fltcmp(imu_buf2.gyr[0][2], 3.0) == 0);

  return 0;
}

int test_imu_buf_print() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buf_add(&imu_buf, ts, acc, gyr);

  imu_buf_print(&imu_buf);
  return 0;
}

int test_solver_setup() {
  solver_t solver;
  solver_setup(&solver);
  return 0;
}

/* int test_solver_print() { */
/*   solver_t solver; */
/*   solver_setup(&solver); */
/*   solver_print(&solver); */
/*   return 0; */
/* } */

void test_suite() {
  /* LOGGING */
  MU_ADD_TEST(test_debug);
  MU_ADD_TEST(test_log_error);
  MU_ADD_TEST(test_log_warn);

  /* DATA */
  MU_ADD_TEST(test_malloc_string);
  MU_ADD_TEST(test_dsv_rows);
  MU_ADD_TEST(test_dsv_cols);
  MU_ADD_TEST(test_dsv_fields);
  MU_ADD_TEST(test_dsv_data);

  /* LINEAR ALGEBRA */
  MU_ADD_TEST(test_eye);
  MU_ADD_TEST(test_ones);
  MU_ADD_TEST(test_zeros);
  MU_ADD_TEST(test_mat_set);
  MU_ADD_TEST(test_mat_val);
  MU_ADD_TEST(test_mat_block_get);
  MU_ADD_TEST(test_mat_block_set);
  MU_ADD_TEST(test_mat_diag_get);
  MU_ADD_TEST(test_mat_diag_set);
  MU_ADD_TEST(test_mat_triu);
  MU_ADD_TEST(test_mat_tril);
  MU_ADD_TEST(test_mat_trace);
  MU_ADD_TEST(test_mat_transpose);
  MU_ADD_TEST(test_mat_add);
  MU_ADD_TEST(test_mat_sub);
  MU_ADD_TEST(test_mat_scale);
  MU_ADD_TEST(test_vec_add);
  MU_ADD_TEST(test_vec_sub);
  MU_ADD_TEST(test_dot);
  MU_ADD_TEST(test_skew);
  MU_ADD_TEST(test_check_jacobian);

  /* SVD */
  MU_ADD_TEST(test_svd);

  /* CHOL */
  MU_ADD_TEST(test_chol);
  MU_ADD_TEST(test_chol_solve);
#ifdef USE_LAPACK
  MU_ADD_TEST(test_chol_solve2);
#endif

  /* TRANSFORMS */
  MU_ADD_TEST(test_tf_rot_set);
  MU_ADD_TEST(test_tf_trans_set);
  MU_ADD_TEST(test_tf_trans_get);
  MU_ADD_TEST(test_tf_rot_get);
  MU_ADD_TEST(test_tf_quat_get);
  MU_ADD_TEST(test_tf_inv);
  MU_ADD_TEST(test_tf_point);
  MU_ADD_TEST(test_tf_hpoint);
  MU_ADD_TEST(test_tf_perturb_rot);
  MU_ADD_TEST(test_tf_perturb_trans);
  MU_ADD_TEST(test_euler321);
  MU_ADD_TEST(test_rot2quat);
  MU_ADD_TEST(test_quat2euler);
  MU_ADD_TEST(test_quat2rot);

  /* SENSOR FUSION */
  MU_ADD_TEST(test_pose_setup);
  MU_ADD_TEST(test_speed_biases_setup);
  MU_ADD_TEST(test_feature_setup);
  MU_ADD_TEST(test_extrinsics_setup);
  MU_ADD_TEST(test_camera_setup);
  MU_ADD_TEST(test_pose_factor_setup);
  MU_ADD_TEST(test_pose_factor_eval);
  MU_ADD_TEST(test_cam_factor_setup);
  MU_ADD_TEST(test_cam_factor_eval);
  MU_ADD_TEST(test_imu_buf_setup);
  MU_ADD_TEST(test_imu_buf_add);
  MU_ADD_TEST(test_imu_buf_clear);
  MU_ADD_TEST(test_imu_buf_copy);
  MU_ADD_TEST(test_imu_buf_print);
  /* MU_ADD_TEST(test_imu_factor_setup); */
  /* MU_ADD_TEST(test_imu_factor_eval); */
  /* MU_ADD_TEST(test_solver_setup); */
  /* MU_ADD_TEST(test_solver_print); */
  /* MU_ADD_TEST(test_solver_eval); */
  /* MU_ADD_TEST(test_solver_solve); */
}

MU_RUN_TESTS(test_suite)
