#ifndef CORE_H
#define CORE_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

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
} mat_t;

void vec_new(vec_t *v, const int length);
void vec_free(vec_t *v);
void mat_new(mat_t *m, const int rows, const int cols);
void mat_free(mat_t *m);

real_t deg2rad(const real_t d);
real_t rad2deg(const real_t r);

#endif // CORE_H
