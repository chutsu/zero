#include "zero/munit.h"
#include "zero/core.h"

int test_eye() {
  real_t A[25] = {0.0};
  eye(A, 5, 5);

  size_t idx = 0;
  size_t rows = 5;
  size_t cols = 5;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      MU_CHECK(A[idx] == ((i == j) ? 1.0 : 0.0));
      idx++;
    }
  }
  /* print_matrix("I", A, 5, 5); */

  return 0;
}

int test_ones() {
  real_t A[25] = {0.0};
  ones(A, 5, 5);

  size_t idx = 0;
  size_t rows = 5;
  size_t cols = 5;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      MU_CHECK((fabs(A[idx] - 1.0) < 1e-5));
      idx++;
    }
  }
  /* print_matrix("A", A, 5, 5); */

  return 0;
}

int test_zeros() {
  real_t A[25] = {0.0};
  zeros(A, 5, 5);

  size_t idx = 0;
  size_t rows = 5;
  size_t cols = 5;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      MU_CHECK((fabs(A[idx] - 0.0) < 1e-5));
      idx++;
    }
  }
  /* print_matrix("A", A, 5, 5); */

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_eye);
  MU_ADD_TEST(test_ones);
  MU_ADD_TEST(test_zeros);
}

MU_RUN_TESTS(test_suite);
