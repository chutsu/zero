#ifndef ZERO_H
#define ZERO_H

/**
 * @defgroup macros Macros
 * @defgroup logging Logging
 * @defgroup fs Filesystem
 * @defgroup data Data
 * @defgroup time Time
 * @defgroup maths Maths
 * @defgroup lm Linear Algebra
 * @defgroup svd SVD
 * @defgroup chol Cholesky
 * @defgroup tf Transforms
 * @defgroup image Image
 * @defgroup cv Computer Vision
 * @defgroup sf Sensor Fusion
 */

#define PRECISION 1
#define MAX_LINE_LENGTH 9046
#define USE_CBLAS
/* #define USE_LAPACK */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <dirent.h>
#include <assert.h>
#include <sys/time.h>

#include "zero/stb_image.h"

#ifdef USE_CBLAS
#include <cblas.h>
#endif

#ifdef USE_LAPACK
#include <lapacke.h>
#endif

/******************************************************************************
 *                                MACROS
 ******************************************************************************/

/**
 * @ingroup macros
 * @{
 */

/**
 * Mark variable unused.
 * @param[in] expr Variable to mark as unused
 */
#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

/**
 * Check if condition is satisfied.
 *
 * If the condition is not satisfied a message M will be logged and a goto
 * error is called.
 *
 * @param[in] A Condition to be checked
 * @param[in] M Error message
 * @param[in] ... Varadic arguments for error message
 */
#define CHECK(A, M, ...)                                                       \
  if (!(A)) {                                                                  \
    LOG_ERROR(M, ##__VA_ARGS__);                                               \
    goto error;                                                                \
  }

/* @} */ /* ingroup macros */

/******************************************************************************
 *                                LOGGING
 ******************************************************************************/

/**
 * @ingroup logging
 * @{
 */

#define KRED "\x1B[1;31m" ///< Console color red
#define KGRN "\x1B[1;32m" ///< Console color green
#define KYEL "\x1B[1;33m" ///< Console color yellow
#define KBLU "\x1B[1;34m" ///< Console color blue
#define KMAG "\x1B[1;35m" ///< Console color magenta
#define KCYN "\x1B[1;36m" ///< Console color cyan
#define KWHT "\x1B[1;37m" ///< Console color white
#define KNRM "\x1B[1;0m"  ///< Reset console color

/// Macro function that returns the caller's filename
#define __FILENAME__                                                           \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

/**
 * Debug
 *
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#ifdef NDEBUG
#define DEBUG(M, ...)
#else
#define DEBUG(M, ...) fprintf(stdout, "[DEBUG] " M "\n", ##__VA_ARGS__)
#endif

/**
 * Log error
 *
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#define LOG_ERROR(M, ...)                                                      \
  fprintf(stderr,                                                              \
          KRED "[ERROR] [%s:%d] " M KNRM "\n",                                 \
          __FILENAME__,                                                        \
          __LINE__,                                                            \
          ##__VA_ARGS__)

/**
 * Log info
 *
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#define LOG_INFO(M, ...)																											 \
  fprintf(stderr,                                                              \
          "[INFO] [%s:%d] " M "\n",																					   \
          __FILENAME__,                                                        \
          __LINE__,                                                            \
          ##__VA_ARGS__)

/**
 * Log warn
 *
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#define LOG_WARN(M, ...)                                                       \
  fprintf(stderr,                                                              \
          KYEL "[WARN] [%s:%d] " M KNRM "\n",																   \
          __FILENAME__,                                                        \
          __LINE__,                                                            \
          ##__VA_ARGS__)

/**
 * Fatal
 *
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#define FATAL(M, ...)                                                          \
  fprintf(stdout,                                                              \
          KRED "[FATAL] [%s:%d] " M KNRM "\n",                                 \
          __FILENAME__,                                                        \
          __LINE__,                                                            \
          ##__VA_ARGS__);                                                      \
  exit(-1)

/* @} */ /* ingroup logging */

/******************************************************************************
 *                               FILESYSTEM
 ******************************************************************************/

/**
 * @ingroup fs
 * @{
 */

char **list_files(const char *path, int *nb_files);
void list_files_free(char **data, const int n);
char *file_read(const char *fp);
void skip_line(FILE *fp);
int file_rows(const char *fp);
int file_copy(const char *src, const char *dest);

/* @} */ /* ingroup fs */

/******************************************************************************
 *                                   DATA
 ******************************************************************************/

/**
 * @ingroup data
 * @{
 */

#if PRECISION == 1
typedef float real_t;
#elif PRECISION == 2
typedef double real_t;
#else
#error "Precision not defined!"
#endif

char *malloc_string(const char *s);
int dsv_rows(const char *fp);
int dsv_cols(const char *fp, const char delim);
char **dsv_fields(const char *fp, const char delim, int *nb_fields);
real_t **dsv_data(const char *fp, const char delim, int *nb_rows, int *nb_cols);
real_t **csv_data(const char *fp, int *nb_rows, int *nb_cols);
int **load_iarrays(const char *csv_path, int *nb_arrays);
real_t **load_darrays(const char *csv_path, int *nb_arrays);
real_t *load_vector(const char *file_path);

/* @} */ /* ingroup data */

/******************************************************************************
 *                                  TIME
 ******************************************************************************/

/**
 * @ingroup time
 * @{
 */

typedef uint64_t timestamp_t;

struct timespec tic();
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);
float time_now();

/* @} */ /* ingroup time */

/******************************************************************************
 *                                  MATHS
 ******************************************************************************/

/**
 * @ingroup maths
 * @{
 */

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

float randf(float a, float b);
real_t deg2rad(const real_t d);
real_t rad2deg(const real_t r);
int fltcmp(const real_t x, const real_t y);
real_t pythag(const real_t a, const real_t b);
real_t lerp(const real_t a, const real_t b, const real_t t);
void lerp3(const real_t *a, const real_t *b, const real_t t, real_t *x);
real_t sinc(const real_t x);
real_t mean(const real_t* x, const size_t length);
real_t median(const real_t* x, const size_t length);
real_t var(const real_t *x, const size_t length);
real_t stddev(const real_t *x, const size_t length);

/* @} */ /* ingroup maths */

/******************************************************************************
 *                              LINEAR ALGEBRA
 ******************************************************************************/

/**
 * @ingroup lm
 * @{
 */

void print_matrix(const char *prefix,
                  const real_t *A,
                  const size_t m,
                  const size_t n);
void print_vector(const char *prefix, const real_t *v, const size_t n);

void eye(real_t *A, const size_t m, const size_t n);
void ones(real_t *A, const size_t m, const size_t n);
void zeros(real_t *A, const size_t m, const size_t n);

real_t *mat_new(const size_t m, const size_t n);
int mat_cmp(const real_t *A, const real_t *B, const size_t m, const size_t n);
int mat_equals(const real_t *A,
               const real_t *B,
               const size_t m,
               const size_t n,
               const real_t tol);
int mat_save(const char *save_path, const real_t *A, const int m, const int n);
real_t *mat_load(const char *save_path, int *nb_rows, int *nb_cols);
void mat_set(real_t *A,
             const size_t stride,
             const size_t i,
             const size_t j,
             const real_t val);
real_t
mat_val(const real_t *A, const size_t stride, const size_t i, const size_t j);
void mat_copy(const real_t *src, const int m, const int n, real_t *dest);
void mat_block_get(const real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t cs,
                   const size_t re,
                   const size_t ce,
                   real_t *block);
void mat_block_set(real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t cs,
                   const size_t re,
                   const size_t ce,
                   const real_t *block);
void mat_diag_get(const real_t *A, const int m, const int n, real_t *d);
void mat_diag_set(real_t *A, const int m, const int n, const real_t *d);
void mat_triu(const real_t *A, const size_t n, real_t *U);
void mat_tril(const real_t *A, const size_t n, real_t *L);
real_t mat_trace(const real_t *A, const size_t m, const size_t n);
void mat_transpose(const real_t *A, size_t m, size_t n, real_t *A_t);
void mat_add(const real_t *A, const real_t *B, real_t *C, size_t m, size_t n);
void mat_sub(const real_t *A, const real_t *B, real_t *C, size_t m, size_t n);
void mat_scale(real_t *A, const size_t m, const size_t n, const real_t scale);

real_t *vec_new(const size_t length);
void vec_copy(const real_t *src, const size_t length, real_t *dest);
int vec_equals(const real_t *x, const real_t *y, const size_t length);
void vec_add(const real_t *x, const real_t *y, real_t *z, size_t length);
void vec_sub(const real_t *x, const real_t *y, real_t *z, size_t length);
void vec_scale(real_t *x, const size_t length, const real_t scale);
real_t vec_norm(const real_t *x, const size_t length);

void dot(const real_t *A,
         const size_t A_m,
         const size_t A_n,
         const real_t *B,
         const size_t B_m,
         const size_t B_n,
         real_t *C);
void skew(const real_t x[3], real_t A[3 * 3]);
void fwdsubs(const real_t *L, const real_t *b, real_t *y, const size_t n);
void bwdsubs(const real_t *U, const real_t *y, real_t *x, const size_t n);
int check_jacobian(const char *jac_name,
                   const real_t *fdiff,
                   const real_t *jac,
                   const size_t m,
                   const size_t n,
                   const real_t tol,
                   const int verbose);

#ifdef USE_CBLAS
void cblas_dot(const real_t *A,
               const size_t A_m,
               const size_t A_n,
               const real_t *B,
               const size_t B_m,
               const size_t B_n,
               real_t *C);
#endif

/* @} */ /* ingroup lm */

/******************************************************************************
 *                                   SVD
 ******************************************************************************/

/**
 * @ingroup svd
 * @{
 */

int svd(real_t *A, int m, int n, real_t *U, real_t *s, real_t *V_t);
int svdcomp(real_t *A, int m, int n, real_t *w, real_t *V);
int pinv(real_t *A, const int m, const int n, real_t *A_inv);

#ifdef USE_LAPACK

#endif

/* @} */ /* ingroup svd */

/******************************************************************************
 *                                   CHOL
 ******************************************************************************/

/**
 * @ingroup chol
 * @{
 */

void chol(const real_t *A, const size_t n, real_t *L);
void chol_solve(const real_t *A, const real_t *b, real_t *x, const size_t n);

#ifdef USE_LAPACK
void lapack_chol_solve(const real_t *A,
                       const real_t *b,
                       real_t *x,
                       const size_t n);
#endif

/* @} */ /* ingroup chol */

/******************************************************************************
 *                                TRANSFORMS
 ******************************************************************************/

/**
 * @ingroup tf
 * @{
 */

void tf(const real_t params[7], real_t T[4 * 4]);
void tf_params(const real_t T[4 * 4], real_t params[7]);
void tf_rot_set(real_t T[4 * 4], const real_t C[3 * 3]);
void tf_trans_set(real_t T[4 * 4], const real_t r[3]);
void tf_trans_get(const real_t T[4 * 4], real_t r[3]);
void tf_rot_get(const real_t T[4 * 4], real_t C[3 * 3]);
void tf_quat_get(const real_t T[4 * 4], real_t q[4]);
void tf_inv(const real_t T[4 * 4], real_t T_inv[4 * 4]);
void tf_point(const real_t T[4 * 4], const real_t p[3], real_t retval[3]);
void tf_hpoint(const real_t T[4 * 4], const real_t p[4], real_t retval[4]);
void tf_perturb_rot(real_t T[4 * 4], const real_t step_size, const int i);
void tf_perturb_trans(real_t T[4 * 4], const real_t step_size, const int i);
void rvec2rot(const real_t *rvec, const real_t eps, real_t *R);
void euler321(const real_t euler[3], real_t C[3 * 3]);
void rot2quat(const real_t C[3 * 3], real_t q[4]);
void quat2euler(const real_t q[4], real_t euler[3]);
void quat2rot(const real_t q[4], real_t C[3 * 3]);
void quat_lmul(const real_t p[4], const real_t q[4], real_t r[4]);
void quat_rmul(const real_t p[4], const real_t q[4], real_t r[4]);
void quat_mul(const real_t p[4], const real_t q[4], real_t r[4]);
void quat_delta(const real_t dalpha[3], real_t dq[4]);

/* @} */ /* ingroup tf */

/******************************************************************************
 *                                 IMAGE
 ******************************************************************************/

/**
 * @ingroup image
 * @{
 */

typedef struct image_t {
  int width;
  int height;
  int channels;
  uint8_t *data;
} image_t;

void image_setup(image_t *img,
                 const int width,
                 const int height,
                 uint8_t *data);
image_t *image_load(const char *file_path);
void image_print_properties(const image_t *img);
void image_free(image_t *img);

/* @} */ /* ingroup image */

/******************************************************************************
 *                                  CV
 ******************************************************************************/

/**
 * @ingroup cv
 * @{
 */

/******************************** RADTAN **************************************/

void radtan4_distort(const real_t params[4], const real_t p[2], real_t p_d[2]);
void radtan4_point_jacobian(const real_t params[4],
                            const real_t p[2],
                            real_t J_point[2 * 2]);
void radtan4_params_jacobian(const real_t params[4],
                             const real_t p[2],
                             real_t J_param[2 * 4]);

/********************************* EQUI ***************************************/

void equi4_distort(const real_t params[4], const real_t p[2], real_t p_d[2]);
void equi4_point_jacobian(const real_t params[4],
                          const real_t p[2],
                          real_t J_point[2 * 2]);
void equi4_params_jacobian(const real_t params[4],
                           const real_t p[2],
                           real_t J_param[2 * 4]);

/******************************** PINHOLE *************************************/

real_t pinhole_focal(const int image_width, const real_t fov);
void pinhole_project(const real_t params[4], const real_t p_C[3], real_t x[2]);

void pinhole_point_jacobian(const real_t params[4], real_t J_point[2 * 3]);
void pinhole_params_jacobian(const real_t params[4],
                             const real_t x[2],
                             real_t J[2 * 4]);

/**************************** PINHOLE-RADTAN4 *********************************/

void pinhole_radtan4_project(const real_t params[8],
                             const real_t p_C[3],
                             real_t x[2]);
void pinhole_radtan4_project_jacobian(const real_t params[8],
                                      const real_t p_C[3],
                                      real_t J[2 * 3]);
void pinhole_radtan4_params_jacobian(const real_t params[8],
                                     const real_t p_C[3],
                                     real_t J[2 * 8]);

/***************************** PINHOLE-EQUI4 **********************************/

void pinhole_equi4_project(const real_t params[8],
                           const real_t p_C[3],
                           real_t x[2]);
void pinhole_equi4_project_jacobian(const real_t params[8],
                                    const real_t p_C[3],
                                    real_t J[2 * 3]);
void pinhole_equi4_params_jacobian(const real_t params[8],
                                   const real_t p_C[3],
                                   real_t J[2 * 8]);

/******************************************************************************
 *                              SENSOR FUSION
 ******************************************************************************/

/**
 * @ingroup sf
 * @{
 */

/* POSE --------------------------------------------------------------------- */

typedef struct pose_t {
  uint64_t param_id;
  timestamp_t ts;
  real_t data[7];
} pose_t;

void pose_setup(pose_t *pose,
                uint64_t *param_id,
                const timestamp_t ts,
                const real_t *param);
void pose_print(const pose_t *pose);

/* SPEED AND BIASES --------------------------------------------------------- */

typedef struct speed_biases_t {
  uint64_t param_id;
  timestamp_t ts;
  real_t data[9];
} speed_biases_t;

void speed_biases_setup(speed_biases_t *sb,
                      uint64_t *param_id,
                      const timestamp_t ts,
                      const real_t *param);
void speed_biases_print(const speed_biases_t *sb);

/* FEATURE ------------------------------------------------------------------ */

typedef struct feature_t {
  uint64_t param_id;
  real_t data[3];
} feature_t;

void feature_setup(feature_t *p,
                   uint64_t *param_id,
                   const real_t *param);
void feature_print(const feature_t *feature);

/* EXTRINSICS --------------------------------------------------------------- */

typedef struct extrinsics_t {
  uint64_t param_id;
  real_t data[7];
} extrinsics_t;

void extrinsics_setup(extrinsics_t *extrinsics,
                      uint64_t *param_id,
                      const real_t *param);
void extrinsics_print(const extrinsics_t *extrinsics);

/* CAMERA ------------------------------------------------------------------- */

typedef struct camera_t {
  uint64_t param_id;
  int cam_idx;
  int resolution[2];
  char proj_model[20];
  char dist_model[20];
  real_t data[8];
} camera_t;

void camera_setup(camera_t *camera,
                  uint64_t *param_id,
                  const int cam_idx,
                  const int cam_res[2],
                  const char *proj_model,
                  const char *dist_model,
                  const real_t *data);
void camera_print(const camera_t *camera);

/* POSE FACTOR -------------------------------------------------------------- */

typedef struct pose_factor_t {
  real_t pose_meas[7];
  pose_t *pose_est;

  real_t covar[6 * 6];
  real_t r[6];
  int r_size;

  real_t J0[6 * 6];
  real_t *jacs[1];
  int nb_params;
} pose_factor_t;

void pose_factor_setup(pose_factor_t *factor,
                       pose_t *pose,
                       const real_t var[6]);
void pose_factor_reset(pose_factor_t *factor);
int pose_factor_eval(pose_factor_t *factor);

/* CAMERA FACTOR ------------------------------------------------------------ */

typedef struct cam_factor_t {
  pose_t *pose;
  extrinsics_t *extrinsics;
  camera_t *camera;
  feature_t *feature;

  real_t covar[2*2];
  real_t z[2];

  real_t r[2];
  int r_size;

  real_t J0[2*6];
  real_t J1[2*6];
  real_t J2[2*8];
  real_t J3[2*3];
  real_t *jacs[4];
  int nb_params;
} cam_factor_t;

void cam_factor_setup(cam_factor_t *factor,
                      pose_t *pose,
                      extrinsics_t *extrinsics,
                      camera_t *camera,
                      const real_t var[2]);
void cam_factor_reset(cam_factor_t *factor);
int cam_factor_eval(cam_factor_t *factor);

/* IMU FACTOR --------------------------------------------------------------- */

#define MAX_IMU_BUF_SIZE 10000

typedef struct imu_params_t {
  uint64_t param_id;
  int imu_idx;
  real_t rate;

  real_t n_aw[3];
  real_t n_gw[3];
  real_t n_a[3];
  real_t n_g[3];
  real_t g;
} imu_params_t;

typedef struct imu_buf_t {
  timestamp_t ts[MAX_IMU_BUF_SIZE];
  real_t acc[MAX_IMU_BUF_SIZE][3];
  real_t gyr[MAX_IMU_BUF_SIZE][3];
  int size;
} imu_buf_t;

typedef struct imu_factor_t {
  imu_params_t *imu_params;
  imu_buf_t imu_buf;
  pose_t *pose_i;
  pose_t *pose_j;
  speed_biases_t *sb_i;
  speed_biases_t *sb_j;

  real_t covar[15 * 15];
  real_t r[15];
  int r_size;

  real_t J0[2 * 6];
  real_t J1[2 * 9];
  real_t J2[2 * 6];
  real_t J3[2 * 9];
  real_t *jacs[4];
  int nb_params;
} imu_factor_t;

void imu_buf_setup(imu_buf_t *imu_buf);
void imu_buf_add(imu_buf_t *imu_buf,
                 timestamp_t ts,
                 real_t acc[3],
                 real_t gyr[3]);
void imu_buf_clear(imu_buf_t *imu_buf);
void imu_buf_copy(const imu_buf_t *from, imu_buf_t *to);
void imu_buf_print(const imu_buf_t *imu_buf);

void imu_factor_setup(imu_factor_t *factor,
                      imu_params_t *imu_params,
                      imu_buf_t *imu_buf,
                      pose_t *pose_i,
                      speed_biases_t *sb_i,
                      pose_t *pose_j,
                      speed_biases_t *sb_j);
void imu_factor_reset(imu_factor_t *factor);
int imu_factor_eval(imu_factor_t *factor);

/* SLIDING WINDOW ESTIMATOR ------------------------------------------------- */

#define MAX_POSES 1000
#define MAX_CAMS 4
#define MAX_FEATURES 1000
#define MAX_PARAMS MAX_POSES + MAX_CAMS + MAX_CAMS + MAX_FEATURES
#define MAX_H_SIZE \
  MAX_POSES * 6 \
  + MAX_CAMS * 8 \
  + MAX_CAMS * 6 \
  + MAX_FEATURES * 3

typedef struct solver_t {
  cam_factor_t cam_factors[MAX_FEATURES];
  int nb_cam_factors;

  imu_factor_t imu_factors[MAX_POSES];
  int nb_imu_factors;

  pose_t poses[MAX_POSES];
  int nb_poses;

  camera_t cams[MAX_CAMS];
  int nb_cams;

  extrinsics_t extrinsics[MAX_CAMS];
  int nb_extrinsics;

  feature_t features[MAX_FEATURES];
  int nb_features;

  real_t H[MAX_H_SIZE];
  real_t g[MAX_H_SIZE];
  real_t x[MAX_H_SIZE];
  int x_size;
  int r_size;
} solver_t;

void solver_setup(solver_t *solver);
void solver_print(solver_t *solver);
int solver_eval(solver_t *solver);
void solver_optimize(solver_t *solver);

/* @} */ /* ingroup sf */

#endif // ZERO_H
