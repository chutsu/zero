#ifndef CORE_H
#define CORE_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

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

typedef struct vec_t {
  real_t *data;
  size_t length;
} vec_t;

typedef struct mat_t {
  real_t *data;
  size_t rows;
  size_t cols;
  int transpose;
} mat_t;

vec_t *vec_new(const int length);
void vec_free(vec_t *v);
mat_t *mat_new(const int rows, const int cols);
void mat_free(mat_t *m);

real_t deg2rad(const real_t d);
real_t rad2deg(const real_t r);

void vec2_ones(vec2_t *v);
void vec3_ones(vec3_t *v);
void vec4_ones(vec4_t *v);
void vec5_ones(vec5_t *v);

void vec2_zeros(vec2_t *v);
void vec3_zeros(vec3_t *v);
void vec4_zeros(vec4_t *v);
void vec5_zeros(vec5_t *v);

void vec_ones(vec_t *v, const int length);
void vec_zeros(vec_t *v, const int length);

void mat2_transpose(mat2_t *m);
void mat2_eye(mat2_t *m);
void mat2_ones(mat2_t *m);
void mat2_zeros(mat2_t *m);

void mat3_transpose(mat3_t *m);
void mat3_eye(mat3_t *m);
void mat3_ones(mat3_t *m);
void mat3_zeros(mat3_t *m);

void mat_transpose(mat_t *m);
void mat_eye(mat_t *m, const int rows, const int cols);
void mat_ones(mat_t *m, const int rows, const int cols);
void mat_zeros(mat_t *m, const int rows, const int cols);
void mat_mul(const mat_t *m1, const mat_t *m2, mat_t *out);
real_t mat_val(const mat_t *m, const int row, const int col);
int mat_block(const mat_t *m,
              const int row_start,
              const int col_start,
              const int row_end,
              const int col_end,
              mat_t *block);

#endif // CORE_H
