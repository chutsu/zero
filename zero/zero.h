#ifndef ZERO_H
#define ZERO_H

#define MAX_LINE_LENGTH 9046
#define USE_CBLAS
#define USE_LAPACK

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <assert.h>
#include <sys/time.h>

#ifdef USE_CBLAS
#include <cblas.h>
#endif

#ifdef USE_LAPACK
#include <lapacke.h>
#endif

/******************************************************************************
 *                                LOGGING
 ******************************************************************************/

/* DEBUG */
#ifdef NDEBUG
#define DEBUG(M, ...)
#else
#define DEBUG(M, ...)                                                          \
  fprintf(stderr, "[DEBUG] %s:%d: " M "\n", __func__, __LINE__, ##__VA_ARGS__)
#endif

/* LOG */
#define LOG_ERROR(M, ...)                                                      \
  fprintf(stderr, "[ERROR] [%s] " M "\n", __func__, ##__VA_ARGS__)
#define LOG_WARN(M, ...) fprintf(stderr, "[WARN] " M "\n", ##__VA_ARGS__)
#define LOG_INFO(M, ...) fprintf(stderr, "[INFO] " M "\n", ##__VA_ARGS__)

/* FATAL */
#define FATAL(M, ...)                                                          \
  fprintf(stderr, "[FATAL] " M "\n", ##__VA_ARGS__);                           \
  exit(-1);

/* CHECK */
#define CHECK(A, M, ...)                                                       \
  if (!(A)) {                                                                  \
    log_err(M, ##__VA_ARGS__);                                                 \
    goto error;                                                                \
  }

/******************************************************************************
 *                                   DATA
 ******************************************************************************/

char *malloc_string(const char *s);
int csv_rows(const char *fp);
int csv_cols(const char *fp);
char **csv_fields(const char *fp, int *nb_fields);
double **csv_data(const char *fp, int *nb_rows, int *nb_cols);
int **load_iarrays(const char *csv_path, int *nb_arrays);
double **load_darrays(const char *csv_path, int *nb_arrays);
double *load_vector(const char *file_path);

/******************************************************************************
 *                                 GENERAL
 ******************************************************************************/

#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

float randf(float a, float b);
double deg2rad(const double d);
double rad2deg(const double r);
int fltcmp(const double x, const double y);
double pythag(const double a, const double b);
double lerpd(const double a, const double b, const double t);
void lerp3d(const double *a, const double *b, const double t, double *x);
float lerpf(const float a, const float b, const float t);
void lerp3f(const float *a, const float *b, const float t, float *x);
double sinc(const double x);

/******************************************************************************
 *                              LINEAR ALGEBRA
 ******************************************************************************/

void print_matrix(const char *prefix,
                  const double *data,
                  const size_t m,
                  const size_t n);
void print_vector(const char *prefix, const double *data, const size_t length);

void eye(double *A, const size_t m, const size_t n);
void ones(double *A, const size_t m, const size_t n);
void zeros(double *A, const size_t m, const size_t n);

double *mat_new(const size_t m, const size_t n);
int mat_cmp(const double *A, const double *B, const size_t m, const size_t n);
int mat_equals(const double *A,
               const double *B,
               const size_t m,
               const size_t n,
               const double thresh);
int mat_save(const char *save_path, const double *A, const int m, const int n);
double *mat_load(const char *save_path, int *nb_rows, int *nb_cols);
void mat_set(double *A,
             const size_t stride,
             const size_t i,
             const size_t j,
             const double val);
double
mat_val(const double *A, const size_t stride, const size_t i, const size_t j);
void mat_copy(const double *src, const int m, const int n, double *dest);
void mat_block_get(const double *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t cs,
                   const size_t re,
                   const size_t ce,
                   double *block);
void mat_block_set(double *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t cs,
                   const size_t re,
                   const size_t ce,
                   const double *block);
void mat_diag_get(const double *A, const int m, const int n, double *d);
void mat_diag_set(double *A, const int m, const int n, const double *d);
void mat_triu(const double *A, const size_t n, double *U);
void mat_tril(const double *A, const size_t n, double *L);
double mat_trace(const double *A, const size_t m, const size_t n);
void mat_transpose(const double *A, size_t m, size_t n, double *A_t);
void mat_add(const double *A, const double *B, double *C, size_t m, size_t n);
void mat_sub(const double *A, const double *B, double *C, size_t m, size_t n);
void mat_scale(double *A, const size_t m, const size_t n, const double scale);

double *vec_new(const size_t length);
void vec_copy(const double *src, const size_t length, double *dest);
int vec_equals(const double *x, const double *y, const size_t length);
void vec_add(const double *x, const double *y, double *z, size_t length);
void vec_sub(const double *x, const double *y, double *z, size_t length);
void vec_scale(double *x, const size_t length, const double scale);
double vec_norm(const double *x, const size_t length);

void dot(const double *A,
         const size_t A_m,
         const size_t A_n,
         const double *B,
         const size_t B_m,
         const size_t B_n,
         double *C);
void skew(const double x[3], double A[3 * 3]);
void fwdsubs(const double *L, const double *b, double *y, const size_t n);
void bwdsubs(const double *U, const double *y, double *x, const size_t n);
int check_jacobian(const char *jac_name,
                   const double *fdiff,
                   const double *jac,
                   const size_t m,
                   const size_t n,
                   const double threshold,
                   const int print);

void dot_cblas(const double *A,
               const size_t A_m,
               const size_t A_n,
               const double *B,
               const size_t B_m,
               const size_t B_n,
               double *C);

/******************************************************************************
 *                                  SVD
 ******************************************************************************/

int svd(double *A, int m, int n, double *U, double *s, double *V_t);
int svdcomp(double *A, int m, int n, double *w, double *V);
int pinv(double *A, const int m, const int n, double *A_inv);

/******************************************************************************
 *                                  CHOL
 ******************************************************************************/

double *chol(const double *A, const size_t n);
void chol_solve(const double *A, const double *b, double *x, const size_t n);

#ifdef USE_LAPACK
void chol_solve2(const double *A, const double *b, double *x, const size_t n);
#endif

/******************************************************************************
 *                                   TIME
 ******************************************************************************/

typedef uint64_t timestamp_t;

struct timespec tic();
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);
float time_now();

/******************************************************************************
 *                                TRANSFORMS
 ******************************************************************************/

void tf(const double params[7], double T[4 * 4]);
void tf_params(const double T[4 * 4], double params[7]);
void tf_rot_set(double T[4 * 4], const double C[3 * 3]);
void tf_trans_set(double T[4 * 4], const double r[3]);
void tf_trans_get(const double T[4 * 4], double r[3]);
void tf_rot_get(const double T[4 * 4], double C[3 * 3]);
void tf_quat_get(const double T[4 * 4], double q[4]);
void tf_inv(const double T[4 * 4], double T_inv[4 * 4]);
void tf_point(const double T[4 * 4], const double p[3], double retval[3]);
void tf_hpoint(const double T[4 * 4], const double p[4], double retval[4]);
void tf_perturb_rot(double T[4 * 4], const double step_size, const int i);
void tf_perturb_trans(double T[4 * 4], const double step_size, const int i);
void rvec2rot(const double *rvec, const double eps, double *R);
void euler321(const double euler[3], double C[3 * 3]);
void rot2quat(const double C[3 * 3], double q[4]);
void quat2euler(const double q[4], double euler[3]);
void quat2rot(const double q[4], double C[3 * 3]);
void quatlmul(const double p[4], const double q[4], double r[4]);
void quatrmul(const double p[4], const double q[4], double r[4]);
void quatmul(const double p[4], const double q[4], double r[4]);
void quatdelta(const double dalpha[3], double dq[4]);

/*****************************************************************************
 *                                   IMAGE
 *****************************************************************************/

typedef struct image_t {
  uint8_t *data;
  int width;
  int height;
} image_t;

void image_init(image_t *img, uint8_t *data, int width, int height);

/*****************************************************************************
 *                                  PINHOLE
 *****************************************************************************/

void pinhole_K(const double fx,
               const double fy,
               const double cx,
               const double cy,
               double K[3 * 3]);
double pinhole_focal_length(const int image_width, const double fov);
int pinhole_project(const double K[3 * 3], const double p_C[3], double x[2]);
void pinhole_calc_K(const double image_width,
                    const double image_height,
                    const double lens_hfov,
                    const double lens_vfov,
                    double K[3 * 3]);

/*****************************************************************************
 *                                 RADTAN
 *****************************************************************************/

void radtan4_distort(const double k1,
                     const double k2,
                     const double p1,
                     const double p2,
                     const double p[2],
                     double p_d[2]);

void radtan4_point_jacobian(const double k1,
                            const double k2,
                            const double p1,
                            const double p2,
                            const double p[2],
                            double J_point[2 * 2]);

void radtan4_param_jacobian(const double k1,
                            const double k2,
                            const double p1,
                            const double p2,
                            const double p[2],
                            double J_param[2 * 4]);

/*****************************************************************************
 *                                  EQUI
 *****************************************************************************/

void equi4_distort(const double k1,
                   const double k2,
                   const double k3,
                   const double k4,
                   const double p[2],
                   double p_d[2]);

void equi4_point_jacobian(const double k1,
                          const double k2,
                          const double k3,
                          const double k4,
                          const double p[2],
                          double J_point[2 * 2]);

void equi4_param_jacobian(const double k1,
                          const double k2,
                          const double k3,
                          const double k4,
                          const double p[2],
                          double J_param[2 * 4]);

#ifdef __cplusplus
}
#endif
#endif // ZERO_H
