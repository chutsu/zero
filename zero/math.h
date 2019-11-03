#ifndef CORE_H
#define CORE_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <assert.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

/******************************************************************************
 *																 GENERAL
 ******************************************************************************/

double deg2rad(const double d);
double rad2deg(const double r);
int fltcmp(const double x, const double y);
double lerpd(const double a, const double b, const double t);
float lerpf(const float a, const float b, const float t);
double sinc(const double x);

/******************************************************************************
 *															LINEAR ALGEBRA
 ******************************************************************************/

void print_matrix(const char *prefix, const double *data,
									const size_t m, const size_t n);
void print_vector(const char *prefix, const double *data, const size_t length);

void eye(double *A, const size_t m, const size_t n);
void ones(double *A, const size_t m, const size_t n);
void zeros(double *A, const size_t m, const size_t n);

void mat_set(double *A,
             const size_t stride,
             const size_t i,
             const size_t j,
             const double val);
double mat_val(const double *A,
               const size_t stride,
               const size_t i,
               const size_t j);
void mat_block(const double *A,
               const size_t stride,
               const size_t rs,
               const size_t cs,
               const size_t re,
               const size_t ce,
               double *block);
void mat_transpose(const double *A, size_t m, size_t n, double *A_t);
void mat_add(const double *A, const double *B, double *C, size_t m, size_t n);
void mat_sub(const double *A, const double *B, double *C, size_t m, size_t n);
void mat_scale(double *A, const size_t m, const size_t n, const double scale);

void vec_add(const double *x, const double *y, double *z, size_t length);
void vec_sub(const double *x, const double *y, double *z, size_t length);
void vec_scale(double *x, const size_t length, const double scale);
double vec_norm(const double *x, const size_t length);

void dot(const double *A, const size_t A_m, const size_t A_n,
         const double *B, const size_t B_m, const size_t B_n,
         double *C);

/******************************************************************************
 *															TRANSFORMS
 ******************************************************************************/

void tf_set_rot(double T[4*4], double C[3*3]);
void tf_set_trans(double T[4*4], double r[3]);
void tf_trans(const double T[4*4], double r[3]);
void tf_rot(const double T[4*4], double C[3*3]);
void tf_quat(const double T[4*4], double q[4]);
void tf_inv(const double T[4*4], double T_inv[4*4]);
void tf_point(const double T[4*4], const double p[3], double retval[3]);
void tf_hpoint(const double T[4*4], const double p[4], double retval[4]);
void euler321(const double euler[3], double C[3*3]);
void rot2quat(const double C[3*3], double q[4]);
void quat2euler(const double q[4], double euler[3]);
void quat2rot(const double q[4], double C[3*3]);
void quatlmul(const double p[4], const double q[4], double r[4]);
void quatrmul(const double p[4], const double q[4], double r[4]);
void quatmul(const double p[4], const double q[4], double r[4]);

#endif // CORE_H
