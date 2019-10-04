#include "zero/munit.h"
#include "zero/math.h"

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
  real_t A[9] = {0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0};

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
  real_t A[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};

  /* print_matrix("A", A, 3, 3); */
  MU_CHECK(fltcmp(mat_val(A, 3, 0, 0), 1.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 0, 1), 2.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 0, 2), 3.0) == 0);
  MU_CHECK(fltcmp(mat_val(A, 3, 1, 0), 4.0) == 0);

  return 0;
}

int test_mat_block() {
  real_t A[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  real_t B[4] = {0.0};
  mat_block(A, 3, 1, 1, 2, 2, B);

  /* print_matrix("B", B, 2, 2); */
  MU_CHECK(fltcmp(mat_val(B, 2, 0, 0), 5.0) == 0);
  MU_CHECK(fltcmp(mat_val(B, 2, 0, 1), 6.0) == 0);
  MU_CHECK(fltcmp(mat_val(B, 2, 1, 0), 8.0) == 0);
  MU_CHECK(fltcmp(mat_val(B, 2, 1, 1), 9.0) == 0);

  return 0;
}

int test_mat_transpose() {
  real_t A[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  real_t C[9] = {0.0};
  mat_transpose(A, 3, 3, C);
  print_matrix("C", C, 3, 3);

  return 0;
}

int test_mat_add() {
  real_t A[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  real_t B[9] = {9.0, 8.0, 7.0,
                 6.0, 5.0, 4.0,
                 3.0, 2.0, 1.0};
  real_t C[9] = {0.0};
  mat_add(A, B, C, 3, 3);
  print_matrix("C", C, 3, 3);

  return 0;
}

int test_mat_sub() {
  real_t A[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  real_t B[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  real_t C[9] = {0.0};
  mat_sub(A, B, C, 3, 3);
  print_matrix("C", C, 3, 3);

  return 0;
}

int test_vec_add() {
  real_t A[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  real_t B[9] = {9.0, 8.0, 7.0,
                 6.0, 5.0, 4.0,
                 3.0, 2.0, 1.0};
  real_t C[9] = {0.0};
  vec_add(A, B, C, 9);
  print_vector("C", C, 9);

  return 0;
}

int test_vec_sub() {
  real_t A[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  real_t B[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  real_t C[9] = {0.0};
  vec_sub(A, B, C, 9);
  print_vector("C", C, 9);

  return 0;
}

/* void mat_dot(const real_t *A, const size_t A_m, const size_t A_n, */
/*              const real_t *B, const size_t B_m, const size_t B_n, */
/*              real_t *C) { */
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
  real_t A[9] = {1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0};
  /* real_t B[3] = {1.0, 2.0, 3.0}; */
  real_t C[9] = {0.0};

  dot(A, 3, 3, A, 3, 3, C);
  print_matrix("C", C, 3, 3);
  /* print_vector("C", C, 3); */

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_eye);
  MU_ADD_TEST(test_ones);
  MU_ADD_TEST(test_zeros);
  MU_ADD_TEST(test_mat_set);
  MU_ADD_TEST(test_mat_val);
  MU_ADD_TEST(test_mat_block);
  MU_ADD_TEST(test_mat_transpose);
  MU_ADD_TEST(test_mat_add);
  MU_ADD_TEST(test_mat_sub);
  MU_ADD_TEST(test_vec_add);
  MU_ADD_TEST(test_vec_sub);
  MU_ADD_TEST(test_dot);
}

MU_RUN_TESTS(test_suite);
