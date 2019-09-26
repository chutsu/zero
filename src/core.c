#include "zero/core.h"

int8_t int8(const uint8_t *data, const size_t offset) {
  return (int8_t)(data[offset]);
}

uint8_t uint8(const uint8_t *data, const size_t offset) {
  return (uint8_t)(data[offset]);
}

int16_t int16(const uint8_t *data, const size_t offset) {
  return (int16_t)((data[offset + 1] << 8) | (data[offset]));
}

uint16_t uint16(const uint8_t *data, const size_t offset) {
  return (uint16_t)((data[offset + 1] << 8) | (data[offset]));
}

int32_t int32(const uint8_t *data, const size_t offset) {
  return (int32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
      (data[offset + 1] << 8) | (data[offset]));
}

uint32_t uint32(const uint8_t *data, const size_t offset) {
  return (uint32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
      (data[offset + 1] << 8) | (data[offset]));
}

real_t deg2rad(const real_t d) { return d * (M_PI / 180.0); }

real_t rad2deg(const real_t r) { return r * (180.0 / M_PI); }

int fltcmp(const double x, const double y) {
  if (fabs(x -  y) < 1e-6) {
    return 0;
  } else if (x > y) {
    return 1;
  }

  return -1;
}

double lerpd(const double a, const double b, const double t) {
  return a * (1.0 - t) + b * t;
}

float lerpf(const float a, const float b, const float t) {
  return a * (1.0 - t) + b * t;
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

void mat_set(real_t *A,
             const size_t stride,
             const size_t i,
             const size_t j,
             const real_t val) {
  A[(i * stride) + j] = val;
}

real_t mat_val(const real_t *A,
               const size_t stride,
               const size_t i,
               const size_t j) {
  return A[(i * stride) + j];
}

void mat_block(const real_t *A,
               const size_t stride,
               const size_t row_start,
               const size_t col_start,
               const size_t row_end,
               const size_t col_end,
               real_t *block) {
  assert(A != block);
  size_t idx = 0;
  for (size_t i = row_start; i <= row_end; i++) {
    for (size_t j = col_start; j <= col_end; j++) {
      block[idx] = mat_val(A, stride, i, j);
      idx++;
    }
  }
}

void mat_transpose(real_t *A, size_t m, size_t n, real_t *A_t) {
  assert(A != A_t);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      mat_set(A_t, m, j, i, mat_val(A, n, i, j));
    }
  }
}

void mat_add(real_t *A, real_t *B, real_t *C, size_t m, size_t n) {
  assert(A != C);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      mat_set(C, n, i, j, mat_val(A, n, i, j) + mat_val(B, n, i, j));
    }
  }
}

void mat_sub(real_t *A, real_t *B, real_t *C, size_t m, size_t n) {
  assert(A != C);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      mat_set(C, n, i, j, mat_val(A, n, i, j) - mat_val(B, n, i, j));
    }
  }
}

void vec_add(real_t *x, real_t *y, real_t *z, size_t length) {
  assert(x != z);

  for (size_t i = 0; i < length; i++) {
    z[i] = x[i] + y[i];
  }
}

void vec_sub(real_t *x, real_t *y, real_t *z, size_t length) {
  assert(x != z);

  for (size_t i = 0; i < length; i++) {
    z[i] = x[i] + y[i];
  }
}

void dot(const real_t *A, const size_t A_m, const size_t A_n,
         const real_t *B, const size_t B_m, const size_t B_n,
         real_t *C) {
  assert(A_n == B_m);

  size_t m = A_m;
  size_t n = B_n;

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      real_t sum = 0.0;
      for (size_t k = 0; k < A_n; k++) {
        sum += mat_val(A, A_n, i, k) * mat_val(B, B_n, k, j);
      }
      mat_set(C, n, i, j, sum);
    }
  }
}

void mat_scale(real_t *A, const size_t m, const size_t n, const real_t scale) {
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      mat_set(A, n, i, j, mat_val(A, n, i, j) * scale);
    }
  }
}

void vec_scale(real_t *x, const size_t length, const real_t scale) {
  for (size_t i = 0; i < length; i++) {
    x[i] = x[i] * scale;
  }
}
