#ifndef CORE_H
#define CORE_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <assert.h>

/* typedef float real_t; */
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

int fltcmp(const double x, const double y);

double lerpd(const double a, const double b, const double t);
float lerpf(const float a, const float b, const float t);

void print_matrix(const char *prefix, const real_t *data,
                  const size_t m, const size_t n);
void print_vector(const char *prefix, const real_t *data, const size_t length);

void eye(real_t *A, const size_t m, const size_t n);
void ones(real_t *A, const size_t m, const size_t n);
void zeros(real_t *A, const size_t m, const size_t n);

void mat_set(real_t *A,
             const size_t stride,
             const size_t i,
             const size_t j,
             const real_t val);
real_t mat_val(const real_t *A,
               const size_t stride,
               const size_t i,
               const size_t j);
void mat_block(const real_t *A,
               const size_t stride,
               const size_t rs,
               const size_t cs,
               const size_t re,
               const size_t ce,
               real_t *block);
void mat_transpose(const real_t *A, size_t m, size_t n, real_t *A_t);
void mat_add(const real_t *A, const real_t *B, real_t *C, size_t m, size_t n);
void mat_sub(const real_t *A, const real_t *B, real_t *C, size_t m, size_t n);
void mat_scale(real_t *A, const size_t m, const size_t n, const real_t scale);

void vec_add(const real_t *x, const real_t *y, real_t *z, size_t length);
void vec_sub(const real_t *x, const real_t *y, real_t *z, size_t length);
void vec_scale(real_t *A, const size_t length, const real_t scale);

void dot(const real_t *A, const size_t A_m, const size_t A_n,
         const real_t *B, const size_t B_m, const size_t B_n,
         real_t *C);

#endif // CORE_H
