#include "zero/core.h"

vec_t *vec_new(const int length) {
  vec_t *v = malloc(sizeof(vec_t));
  v->data = malloc(sizeof(real_t) * length);
  v->length = length;
  return v;
}

void vec_free(vec_t *v) { free(v->data); }

mat_t *mat_new(const int rows, const int cols) {
  mat_t *m = malloc(sizeof(mat_t));
  m->data = malloc(sizeof(real_t) * rows * cols);
  m->rows = rows;
  m->cols = cols;
  m->transpose = 0;
  return m;
}

void mat_free(mat_t *m) { free(m->data); }

inline real_t deg2rad(const real_t d) { return d * (M_PI / 180.0); }

inline real_t rad2deg(const real_t r) { return r * (180.0 / M_PI); }

void vec2_ones(vec2_t *v) {
  *v[0] = 1.0;
  *v[1] = 1.0;
}

void vec3_ones(vec3_t *v) {
  *v[0] = 1.0;
  *v[1] = 1.0;
  *v[2] = 1.0;
}

void vec4_ones(vec4_t *v) {
  *v[0] = 1.0;
  *v[1] = 1.0;
  *v[2] = 1.0;
  *v[3] = 1.0;
}

void vec5_ones(vec5_t *v) {
  *v[0] = 1.0;
  *v[1] = 1.0;
  *v[2] = 1.0;
  *v[3] = 1.0;
  *v[4] = 1.0;
}

void vec2_zeros(vec2_t *v) {
  *v[0] = 0.0;
  *v[1] = 0.0;
}

void vec3_zeros(vec3_t *v) {
  *v[0] = 0.0;
  *v[1] = 0.0;
  *v[2] = 0.0;
}

void vec4_zeros(vec4_t *v) {
  *v[0] = 0.0;
  *v[1] = 0.0;
  *v[2] = 0.0;
  *v[3] = 0.0;
}

void vec5_zeros(vec5_t *v) {
  *v[0] = 0.0;
  *v[1] = 0.0;
  *v[2] = 0.0;
  *v[3] = 0.0;
  *v[4] = 0.0;
}

void vec_ones(vec_t *v, const int length) {
  for (int i = 0; i < length; i++) {
    v->data[i] = 1.0;
  }
}

void vec_zeros(vec_t *v, const int length) {
  for (int i = 0; i < length; i++) {
    v->data[i] = 0.0;
  }
}

void mat2_transpose(mat2_t *m) {
  /* clang-format off */
  mat2_t m_copy;
  m_copy[0] = *m[0];
  m_copy[1] = *m[2];
  m_copy[2] = *m[1];
  m_copy[3] = *m[3];
  /* clang-format on */

  /* clang-format off */
  *m[0] = m_copy[0]; *m[1] = m_copy[1];
  *m[2] = m_copy[2]; *m[3] = m_copy[3];
  /* clang-format on */
}

void mat2_eye(mat2_t *m) {
  /* clang-format off */
  *m[0] = 1.0; *m[1] = 0.0;
  *m[2] = 0.0; *m[3] = 1.0;
  /* clang-format on */
}

void mat2_ones(mat2_t *m) {
  /* clang-format off */
  *m[0] = 1.0; *m[1] = 1.0;
  *m[2] = 1.0; *m[3] = 1.0;
  /* clang-format on */
}

void mat2_zeros(mat2_t *m) {
  /* clang-format off */
  *m[0] = 0.0; *m[1] = 0.0;
  *m[2] = 0.0; *m[3] = 0.0;
  /* clang-format on */
}

void mat3_transpose(mat3_t *m) {
  /* clang-format off */
  mat3_t m_copy;
  m_copy[0] = *m[0]; m_copy[1] = *m[3]; m_copy[2] = *m[6];
  m_copy[3] = *m[1]; m_copy[4] = *m[4]; m_copy[5] = *m[7];
  m_copy[6] = *m[2]; m_copy[7] = *m[5]; m_copy[8] = *m[8];
  /* clang-format on */

  /* clang-format off */
  *m[0] = m_copy[0]; *m[1] = m_copy[1]; *m[2] = m_copy[2];
  *m[3] = m_copy[3]; *m[4] = m_copy[4]; *m[5] = m_copy[5];
  *m[6] = m_copy[6]; *m[7] = m_copy[7]; *m[8] = m_copy[8];
  /* clang-format on */
}

void mat3_eye(mat3_t *m) {
  /* clang-format on */
  *m[0] = 1.0; *m[1] = 0.0; *m[2] = 0.0;
  *m[3] = 0.0; *m[4] = 1.0; *m[5] = 0.0;
  *m[6] = 0.0; *m[7] = 0.0; *m[8] = 1.0;
  /* clang-format off */
}

void mat3_ones(mat3_t *m) {
  /* clang-format on */
  *m[0] = 1.0; *m[1] = 1.0; *m[2] = 1.0;
  *m[3] = 1.0; *m[4] = 1.0; *m[5] = 1.0;
  *m[6] = 1.0; *m[7] = 1.0; *m[8] = 1.0;
  /* clang-format off */
}

void mat3_zeros(mat3_t *m) {
  /* clang-format on */
  *m[0] = 0.0; *m[1] = 0.0; *m[2] = 0.0;
  *m[3] = 0.0; *m[4] = 0.0; *m[5] = 0.0;
  *m[6] = 0.0; *m[7] = 0.0; *m[8] = 0.0;
  /* clang-format off */
}

void mat_transpose(mat_t *m) {
  if (m->transpose == 0) {
    m->transpose = 1;
    return;
  }
  if (m->transpose == 1) {
    m->transpose = 0;
    return;
  }
}

void mat_eye(mat_t *m, const int rows, const int cols) {
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      if (i == j) {
        m->data[i + j] = 1.0;
      } else {
        m->data[i + j] = 0.0;
      }
    }
  }
}

void mat_ones(mat_t *m, const int rows, const int cols) {
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      m->data[i + j] = 1.0;
    }
  }
}

void mat_zeros(mat_t *m, const int rows, const int cols) {
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      m->data[i + j] = 0.0;
    }
  }
}

void mat_mul(const mat_t *m1, const mat_t *m2, mat_t *out) {


}

real_t mat_val(const mat_t *m, const int row, const int col) {
  return 2.0;
}

int mat_block(const mat_t *m,
              const int row_start,
              const int col_start,
              const int row_end,
              const int col_end,
              mat_t *block) {
  return 0;
}
