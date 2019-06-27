#include "zero/core.h"

real_t deg2rad(const real_t d) { return d * (M_PI / 180.0); }

real_t rad2deg(const real_t r) { return r * (180.0 / M_PI); }

real_t *mat_new(const size_t rows, const size_t cols, const real_t *data) {
  real_t *m = calloc(rows * cols, sizeof(real_t));

  if (m != NULL) {
    size_t idx = 0;
    for (size_t i = 0; i < rows; i++) {
      for (size_t j = 0; j < cols; j++) {
        m[idx] = data[idx];
        idx++;
      }
    }
  }

  return m;
}

void mat_free(real_t *m) {
  free(m);
}

real_t *vec_new(const size_t length, const real_t *data) {
  real_t *v = calloc(length, sizeof(real_t));

  if (v != NULL) {
    size_t idx = 0;
    for (size_t i = 0; i < length; i++) {
      v[idx] = data[idx];
      idx++;
    }
  }

  return v;
}

void vec_free(real_t *v) {
  free(v);
}

void print_matrix(const char *prefix, const real_t *data,
                  const size_t rows, const size_t cols) {
  assert(prefix != NULL);
  assert(data != NULL);
  assert(rows != 0);
  assert(cols != 0);

  size_t idx = 0;
  printf("%s:\n", prefix);
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      printf("%f\t", data[idx]);
      idx++;
    }
    printf("\n");
  }
}

void print_vector(const char *prefix, const real_t *data, const size_t length) {
  assert(prefix != NULL);
  assert(data != NULL);
  assert(length != 0);

  size_t idx = 0;
  printf("%s: ", prefix);
  for (size_t i = 0; i < length; i++) {
    printf("%f\t", data[idx]);
    idx++;
  }
  printf("\n");
}

void eye(real_t *A, const size_t rows, const size_t cols) {
  assert(A != NULL);
  assert(rows != 0);
  assert(cols != 0);

  size_t idx = 0.0;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      A[idx] = (i == j) ? 1.0 : 0.0;
      idx++;
    }
  }
}

void ones(real_t *A, const size_t rows, const size_t cols) {
  assert(A != NULL);
  assert(rows != 0);
  assert(cols != 0);

  size_t idx = 0.0;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      A[idx] = 1.0;
      idx++;
    }
  }
}

void zeros(real_t *A, const size_t rows, const size_t cols) {
  assert(A != NULL);
  assert(rows != 0);
  assert(cols != 0);

  size_t idx = 0.0;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      A[idx] = 0.0;
      idx++;
    }
  }
}

/* void dot(const real_t *A, const size_t A_m, const size_t A_n, */
/*          const real_t *B, const size_t B_m, const size_t B_n, */
/*          real_t *C, const size_t C_m, const size_t C_n) { */
/*   assert(A_n == B_m); */
/*   assert(C_m == B_n); */
/*   assert(C_n == A_m); */
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
