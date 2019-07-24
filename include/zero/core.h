#ifndef CORE_H
#define CORE_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include <cblas.h>

// typedef float real_t;
typedef double real_t;

typedef real_t pt2_t[2];
typedef real_t pt3_t[3];
typedef real_t hp3_t[3];
typedef real_t hp4_t[4];

typedef real_t vec2_t[2];
typedef real_t vec3_t[3];
typedef real_t vec4_t[4];
typedef real_t vec5_t[5];

typedef real_t mat2_t[4];
typedef real_t mat3_t[9];
typedef real_t mat4_t[16];

typedef struct mat_t {
  real_t *data;
  int rows;
  int cols;
} mat_t;

real_t deg2rad(const real_t d);
real_t rad2deg(const real_t r);

void print_matrix(const char *prefix, const real_t *data,
                  const size_t rows, const size_t cols);
void print_vector(const char *prefix, const real_t *data, const size_t length);

void eye(real_t *A, const size_t rows, const size_t cols);
void ones(real_t *A, const size_t rows, const size_t cols);
void zeros(real_t *A, const size_t rows, const size_t cols);
// real_t mat_val(const real_t *m, const size_t row, const size_t col);
// int mat_block(const real_t *m,
//               const size_t row_start,
//               const size_t col_start,
//               const size_t row_end,
//               const size_t col_end,
//               real_t *block);

/* void dot(const real_t *A, const size_t A_m, const size_t A_n, */
/*          const real_t *B, const size_t B_m, const size_t B_n, */
/*          real_t *C, const size_t C_m, const size_t C_n); */

#endif // CORE_H
