#include "se.h"

void pose_setup(pose_t *pose,
                uint64_t *param_id,
                const timestamp_t ts,
                const real_t *data) {
  pose->param_id = *param_id;
  pose->ts = ts;
  *param_id += 1;

  /* Quaternion */
  pose->data[0] = data[0];
  pose->data[1] = data[1];
  pose->data[2] = data[2];
  pose->data[3] = data[3];

  /* Translation */
  pose->data[4] = data[4];
  pose->data[5] = data[5];
  pose->data[6] = data[6];
}

void speed_bias_setup(speed_bias_t *sb,
                      uint64_t *param_id,
                      const timestamp_t ts,
                      const real_t *data) {
  sb->param_id = *param_id;
  sb->ts = ts;
  *param_id += 1;

  /* Velocity */
  sb->data[0] = data[0];
  sb->data[1] = data[1];
  sb->data[2] = data[2];

  /* Accel biases */
  sb->data[3] = data[3];
  sb->data[4] = data[4];
  sb->data[5] = data[5];

  /* Gyro biases */
  sb->data[6] = data[6];
  sb->data[7] = data[7];
  sb->data[8] = data[8];
}

void landmark_setup(landmark_t *p,
                    uint64_t *param_id,
                    const real_t *data) {
  p->param_id = *param_id;
  *param_id += 1;

  p->data[0] = data[0];
  p->data[1] = data[1];
  p->data[2] = data[2];
}

void extrinsics_setup(extrinsics_t *extrinsics,
                      uint64_t *param_id,
                      const timestamp_t ts,
                      const real_t *data) {
  extrinsics->param_id = *param_id;
  extrinsics->ts = ts;
  *param_id += 1;

  /* Quaternion */
  extrinsics->data[0] = data[0];
  extrinsics->data[1] = data[1];
  extrinsics->data[2] = data[2];
  extrinsics->data[3] = data[3];

  /* Translation */
  extrinsics->data[4] = data[4];
  extrinsics->data[5] = data[5];
  extrinsics->data[6] = data[6];
}

void camera_setup(camera_t *camera,
                  uint64_t *param_id,
                  const int cam_idx,
                  const int cam_res[2],
                  const char *proj_model,
                  const char *dist_model,
                  const real_t data[8]) {
  camera->param_id = *param_id;
  camera->cam_idx = cam_idx;
  camera->resolution[0] = cam_res[0];
  camera->resolution[1] = cam_res[1];

  strcpy(camera->proj_model, proj_model);
  strcpy(camera->dist_model, dist_model);

  camera->data[0] = data[0];
  camera->data[1] = data[1];
  camera->data[2] = data[2];
  camera->data[3] = data[3];
  camera->data[4] = data[4];
  camera->data[5] = data[5];
  camera->data[6] = data[6];
  camera->data[7] = data[7];
}

int pose_factor_eval(const pose_factor_t *factor, real_t *r, real_t **jacs) {
  assert(factor && r && jacs);

  /* Map params */
  real_t pose_est[4 * 4] = {0};
  real_t pose_meas[4 * 4] = {0};
  tf(factor->pose_est->data, pose_est);
  tf(factor->pose_meas.data, pose_meas);

  /* Invert estimated pose */
  real_t pose_est_inv[4 * 4] = {0};
  tf_inv(pose_est, pose_est_inv);

  /* Calculate delta pose */
  real_t dpose[4 * 4] = {0};
  dot(pose_est, 4, 4, pose_meas, 4, 4, dpose);

  /* Calculate pose error */
  const real_t dqw = dpose[0];
  const real_t dqx = dpose[1];
  const real_t dqy = dpose[2];
  const real_t dqz = dpose[3];
  const real_t drx = dpose[4];
  const real_t dry = dpose[5];
  const real_t drz = dpose[6];
  /* -- dtheta */
  r[0] = 2.0 * dqx;
  r[1] = 2.0 * dqy;
  r[2] = 2.0 * dqz;
  /* -- dr */
  r[3] = drx;
  r[4] = dry;
  r[5] = drz;

  /* Calculate Jacobians */
  /* clang-format off */
  real_t *J = jacs[0];
  J[0]  = -dqw; J[1]  =  dqz; J[2]  = -dqy; J[3]  =  0.0; J[4]  =  0.0; J[5]  =  0.0;
  J[6]  =  dqz; J[7]  = -dqw; J[8]  =  dqx; J[9]  =  0.0; J[10] =  0.0; J[11] =  0.0;
  J[12] =  dqy; J[13] = -dqx; J[14] = -dqw; J[15] =  0.0; J[16] =  0.0; J[17] =  0.0;
  J[18] =  0.0; J[19] =  0.0; J[20] =  0.0; J[21] = -1.0; J[22] =  0.0; J[23] =  0.0;
  J[24] =  0.0; J[25] =  0.0; J[26] =  0.0; J[27] =  0.0; J[28] = -1.0; J[29] =  0.0;
  J[30] =  0.0; J[31] =  0.0; J[32] =  0.0; J[33] =  0.0; J[34] =  0.0; J[35] = -1.0;
  /* clang-format on */

  return 0;
}

int cam_factor_eval(const cam_factor_t *factor,
                    real_t *r,
                    real_t **jacs) {
  assert(factor && r && J);
  assert(factor->sensor_pose);
  assert(factor->extrinsics);
  assert(factor->landmark);
  assert(factor->camera);
  assert(factor->imu_params);

  /* Sensor pose */
  real_t T_WS[4 * 4] = {0};
  tf(factor->sensor_pose->data, T_WS);

  /* Sensor-Camera extrinsics */
  real_t T_SC[4 * 4] = {0};
  tf(factor->extrinsics->data, T_SC);

  /* Camera pose */
  real_t T_WC[4 * 4] = {0};
  real_t T_CW[4 * 4] = {0};
  dot(T_WS, 4, 4, T_SC, 4, 4, T_WC);
  tf_inv(T_WC, T_CW);

  /* Landmark */
  real_t *p_W = factor->landmark->data;
  real_t p_C[3 * 1] = {0};
  tf_point(T_CW, p_W, p_C);

  /* Project point from world to image plane */
  real_t z_hat[2];
  real_t *cam_params = factor->camera->data;
  pinhole_radtan4_project(cam_params, p_C, z_hat);

  /* Calculate residuals */
  real_t err[2] = {0};
  err[0] = factor->z[0] - z_hat[0];
  err[1] = factor->z[1] - z_hat[1];

  real_t sqrt_info[2 * 2] = {0};
  sqrt_info[0] = 1.0 / factor->covar[0];
  sqrt_info[1] = 0.0;
  sqrt_info[2] = 0.0;
  sqrt_info[3] = 1.0 / factor->covar[1];
  dot(sqrt_info, 2, 2, err, 2, 1, r);

  /* Calculate jacobians */
  /* -- Sensor pose jacobian */
  /* J[0] =  */
  /* -- Extrinsics jacobian */
  /* -- Landmark jacobian */
  /* -- Camera params jacobian */

  return 0;
}

int imu_factor_eval(const imu_factor_t *factor,
                    real_t *r,
                    real_t **jacs) {
  assert(factor && r && J);


  return 0;
}

void fgraph_setup(fgraph_t *graph) {
  assert(graph);

  graph->r_size = 0;
  graph->x_size = 0;

  graph->cam_factors = NULL;
  graph->nb_cam_factors = 0;

  graph->imu_factors = NULL;
  graph->nb_imu_factors = 0;

  graph->poses = NULL;
  graph->nb_poses = 0;
}

void fgraph_print(fgraph_t *graph) {
  printf("graph:\n");
  printf("r_size: %d\n", graph->r_size);
  printf("x_size: %d\n", graph->x_size);
  printf("nb_cam_factors: %d\n", graph->nb_cam_factors);
  printf("nb_imu_factors: %d\n", graph->nb_imu_factors);
  printf("nb_poses: %d\n", graph->nb_poses);
}

int fgraph_eval(fgraph_t *graph, double *H, double *b) {
  assert(graph != NULL && H != NULL && b != NULL);

  return 0;
}

void fgraph_solve(fgraph_t *graph) {


}
