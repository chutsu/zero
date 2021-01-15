#ifndef ZERO_SE_H
#define ZERO_SE_H

#include "zero.h"

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

  real_t covar[2 * 2];
  real_t z[2];

  real_t r[2];
  int r_size;

  real_t J0[2 * 6];
  real_t J1[2 * 6];
  real_t J2[2 * 8];
  real_t J3[2 * 3];
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

typedef struct swe_t {
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
} swe_t;

void swe_setup(swe_t *graph);
void swe_print(swe_t *graph);
int swe_eval(swe_t *graph);
void swe_solve(swe_t *graph);

#endif // ZERO_SE_H
