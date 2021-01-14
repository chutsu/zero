#ifndef ZERO_SE_H
#define ZERO_SE_H

#include "zero.h"

/* PARAMETER TYPE */
#define NOT_SET -1
#define POSE 1
#define LANDMARK 2
#define CAMERA 3
#define EXTRINSIC 4
#define SPEED_BIAS 5

/* PROJECTION MODELS */
#define PINHOLE 1

/* DISTORTION MODELS */
#define RADTAN4 1
#define EQUI4 2

typedef struct pose_t {
  uint64_t param_id;
  timestamp_t ts;
  real_t data[7];
} pose_t;

typedef struct speed_bias_t {
  uint64_t param_id;
  timestamp_t ts;
  real_t data[9];
} speed_bias_t;

typedef struct landmark_t {
  uint64_t param_id;
  real_t data[3];
} landmark_t;

typedef struct extrinsics_t {
  uint64_t param_id;
  timestamp_t ts;
  real_t data[7];
} extrinsics_t;

typedef struct camera_t {
  uint64_t param_id;
  int cam_idx;
  int resolution[2];
  char proj_model[20];
  char dist_model[20];
  real_t data[8];
} camera_t;

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

typedef struct pose_factor_t {
  timestamp_t ts;

  real_t covar[6 * 6];
  pose_t pose_meas;
  pose_t *pose_est;

} pose_factor_t;

typedef struct cam_factor_t {
  timestamp_t ts;

  pose_t *sensor_pose;
  extrinsics_t *extrinsics;
  landmark_t *landmark;
  camera_t *camera;
  imu_params_t *imu_params;

  real_t z[2];
  real_t covar[2*2];
} cam_factor_t;

typedef struct imu_factor_t {
  timestamp_t ts;

  pose_t *pose_i;
  pose_t *pose_j;
  speed_bias_t *sb_i;
  speed_bias_t *sb_j;
  extrinsics_t *extrinsics;

  real_t z[2];
  real_t covar[2*2];
} imu_factor_t;

typedef struct fgraph_t {
  int r_size;
  int x_size;

  cam_factor_t *cam_factors;
  int nb_cam_factors;

  imu_factor_t *imu_factors;
  int nb_imu_factors;

  pose_t *poses;
  int nb_poses;

  camera_t *cams;
  int nb_cams;
} fgraph_t;

void pose_setup(pose_t *pose,
                uint64_t *param_id,
                const timestamp_t ts,
                const real_t *param);
void speed_bias_setup(speed_bias_t *sb,
                      uint64_t *param_id,
                      const timestamp_t ts,
                      const real_t *param);
void landmark_setup(landmark_t *p,
                    uint64_t *param_id,
                    const real_t *param);
void extrinsics_setup(extrinsics_t *extrinsics,
                      uint64_t *param_id,
                      const timestamp_t ts,
                      const real_t *param);
void camera_setup(camera_t *camera,
                  uint64_t *param_id,
                  const int cam_idx,
                  const int cam_res[2],
                  const char *proj_model,
                  const char *dist_model,
                  const real_t *data);

int pose_factor_eval(const pose_factor_t *factor, real_t *r, real_t **jacs);
int cam_factor_eval(const cam_factor_t *factor, real_t *r, real_t **jacs);
int imu_factor_eval(const imu_factor_t *factor, real_t *r, real_t **jacs);

void fgraph_setup(fgraph_t *graph);
void fgraph_print(fgraph_t *graph);
int fgraph_eval(fgraph_t *graph, double *H, double *b);
void fgraph_solve(fgraph_t *graph);

#endif // ZERO_SE_H
