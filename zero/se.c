#include "se.h"

/* POSE --------------------------------------------------------------------- */

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

/* SPEED AND BIASES --------------------------------------------------------- */

void speed_biases_setup(speed_biases_t *sb,
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

/* FEATURE ------------------------------------------------------------------ */

void feature_setup(feature_t *p,
                   uint64_t *param_id,
                   const real_t *data) {
  p->param_id = *param_id;
  *param_id += 1;

  p->data[0] = data[0];
  p->data[1] = data[1];
  p->data[2] = data[2];
}

/* EXTRINSICS --------------------------------------------------------------- */

void extrinsics_setup(extrinsics_t *extrinsics,
                      uint64_t *param_id,
                      const real_t *data) {
  extrinsics->param_id = *param_id;
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

/* CAMERA ------------------------------------------------------------------- */

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

void camera_print(const camera_t *camera) {
  printf("cam_idx: %d\n", camera->cam_idx);
  printf("cam_res: [%d, %d]\n", camera->resolution[0], camera->resolution[1]);
  printf("proj_model: %s\n", camera->proj_model);
  printf("dist_model: %s\n", camera->dist_model);
  printf("data: [");
  for (int i = 0; i < 8; i++) {
    if ((i + 1) < 8) {
      printf("%f, ", camera->data[i]);
    } else {
      printf("%f", camera->data[i]);
    }
  }
  printf("]\n");
}

/* POSE FACTOR -------------------------------------------------------------- */

void pose_factor_setup(pose_factor_t *factor,
                       pose_t *pose,
                       const real_t var[6]) {
  zeros(factor->pose_meas, 7, 1);
  factor->pose_est = pose;

  zeros(factor->covar, 6, 6);
  factor->covar[0] = 1.0 / (var[0] * var[0]);
  factor->covar[7] = 1.0 / (var[1] * var[1]);
  factor->covar[14] = 1.0 / (var[2] * var[2]);
  factor->covar[21] = 1.0 / (var[3] * var[3]);
  factor->covar[28] = 1.0 / (var[4] * var[4]);
  factor->covar[35] = 1.0 / (var[5] * var[5]);

  zeros(factor->r, 6, 1);
  factor->r_size = 6;

  zeros(factor->J0, 6, 6);
  factor->jacs[0] = factor->J0;
  factor->nb_params = 1;
}

void pose_factor_reset(pose_factor_t *factor) {
	zeros(factor->r, 6, 1);
  zeros(factor->J0, 6, 6);
}

int pose_factor_eval(pose_factor_t *factor) {
  assert(factor != NULL);

  /* Map params */
  real_t pose_est[4 * 4] = {0};
  real_t pose_meas[4 * 4] = {0};
  tf(factor->pose_est->data, pose_est);
  tf(factor->pose_meas, pose_meas);

  /* Invert estimated pose */
  real_t pose_est_inv[4 * 4] = {0};
  tf_inv(pose_est, pose_est_inv);

  /* Calculate delta pose */
  real_t dpose[4 * 4] = {0};
  dot(pose_est, 4, 4, pose_meas, 4, 4, dpose);

  /* Calculate pose error */
  real_t *r = factor->r;
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
  real_t *J = factor->J0;
  J[0]  = -dqw; J[1]  =  dqz; J[2]  = -dqy; J[3]  =  0.0; J[4]  =  0.0; J[5]  =  0.0;
  J[6]  = -dqz; J[7]  = -dqw; J[8]  =  dqx; J[9]  =  0.0; J[10] =  0.0; J[11] =  0.0;
  J[12] =  dqy; J[13] = -dqx; J[14] = -dqw; J[15] =  0.0; J[16] =  0.0; J[17] =  0.0;
  J[18] =  0.0; J[19] =  0.0; J[20] =  0.0; J[21] = -1.0; J[22] =  0.0; J[23] =  0.0;
  J[24] =  0.0; J[25] =  0.0; J[26] =  0.0; J[27] =  0.0; J[28] = -1.0; J[29] =  0.0;
  J[30] =  0.0; J[31] =  0.0; J[32] =  0.0; J[33] =  0.0; J[34] =  0.0; J[35] = -1.0;
  factor->jacs[0] = J;
  /* clang-format on */

  return 0;
}

/* CAMERA FACTOR ------------------------------------------------------------ */

void cam_factor_setup(cam_factor_t *factor,
                      pose_t *pose,
                      extrinsics_t *extrinsics,
                      camera_t *camera,
                      const real_t var[2]) {
  factor->pose = pose;
  factor->extrinsics = extrinsics;
  factor->camera = camera;

  factor->covar[0] = 1.0 / (var[0] * var[0]);
  factor->covar[1] = 0.0;
  factor->covar[2] = 0.0;
  factor->covar[3] = 1.0 / (var[1] * var[1]);

  zeros(factor->r, 2, 1);

  zeros(factor->J0, 2, 6);
  zeros(factor->J1, 2, 6);
  zeros(factor->J2, 2, 8);
  zeros(factor->J3, 2, 3);
  factor->jacs[0] = factor->J0;
  factor->jacs[1] = factor->J1;
  factor->jacs[2] = factor->J2;
  factor->jacs[3] = factor->J3;
  factor->nb_params = 4;
}

void cam_factor_reset(cam_factor_t *factor) {
  zeros(factor->r, 2, 1);
  zeros(factor->J0, 2, 6);
  zeros(factor->J1, 2, 6);
  zeros(factor->J2, 2, 8);
  zeros(factor->J3, 2, 3);
}

int cam_factor_eval(cam_factor_t *factor) {
  assert(factor != NULL);
  assert(factor->pose);
  assert(factor->extrinsics);
  assert(factor->feature);
  assert(factor->camera);

  /* Map params */
  /* -- Sensor pose */
  real_t T_WS[4 * 4] = {0};
  tf(factor->pose->data, T_WS);
  /* -- Sensor-Camera extrinsics */
  real_t T_SC[4 * 4] = {0};
  tf(factor->extrinsics->data, T_SC);
  /* -- Camera pose */
  real_t T_WC[4 * 4] = {0};
  real_t T_CW[4 * 4] = {0};
  dot(T_WS, 4, 4, T_SC, 4, 4, T_WC);
  tf_inv(T_WC, T_CW);
  /* -- Landmark */
  real_t *p_W = factor->feature->data;
  real_t p_C[3 * 1] = {0};
  tf_point(T_CW, p_W, p_C);
  /* -- Project point from world to image plane */
  real_t z_hat[2];
  real_t *cam_params = factor->camera->data;
  pinhole_radtan4_project(cam_params, p_C, z_hat);

  /* Calculate residuals */
  /* -- Residual */
  real_t err[2] = {0};
  err[0] = factor->z[0] - z_hat[0];
  err[1] = factor->z[1] - z_hat[1];
  /* -- Weighted residual */
  real_t sqrt_info[2 * 2] = {0};
  sqrt_info[0] = 1.0 / factor->covar[0];
  sqrt_info[1] = 0.0;
  sqrt_info[2] = 0.0;
  sqrt_info[3] = 1.0 / factor->covar[1];
  dot(sqrt_info, 2, 2, err, 2, 1, factor->r);

  /* Calculate jacobians */
  /* -- Camera params jacobian */
  /* -- Extrinsics jacobian */
  /* -- Pose jacobian */
  /* -- Landmark jacobian */

  return 0;
}

/* IMU FACTOR --------------------------------------------------------------- */

void imu_buf_setup(imu_buf_t *imu_buf) {
  for (int k = 0; k < MAX_IMU_BUF_SIZE; k++) {
    imu_buf->ts[k] = 0.0;

    imu_buf->acc[k][0] = 0.0;
    imu_buf->acc[k][1] = 0.0;
    imu_buf->acc[k][2] = 0.0;

    imu_buf->gyr[k][0] = 0.0;
    imu_buf->gyr[k][1] = 0.0;
    imu_buf->gyr[k][2] = 0.0;
  }

	imu_buf->size = 0;
}

void imu_buf_print(const imu_buf_t *imu_buf) {
  for (int k = 0; k < imu_buf->size; k++) {
    const real_t *acc = imu_buf->acc[k];
    const real_t *gyr = imu_buf->gyr[k];

    printf("ts: %ld ", imu_buf->ts[k]);
    printf("acc: [%.2f, %.2f, %.2f] ", acc[0], acc[1], acc[2]);
    printf("gyr: [%.2f, %.2f, %.2f] ", gyr[0], gyr[1], gyr[2]);
    printf("\n");
  }
}

void imu_buf_add(imu_buf_t *imu_buf,
                 timestamp_t ts,
                 real_t acc[3],
                 real_t gyr[3]) {
  int k = imu_buf->size;
  imu_buf->ts[k] = ts;
  imu_buf->acc[k][0] = acc[0];
  imu_buf->acc[k][1] = acc[1];
  imu_buf->acc[k][2] = acc[2];
  imu_buf->gyr[k][0] = gyr[0];
  imu_buf->gyr[k][1] = gyr[1];
  imu_buf->gyr[k][2] = gyr[2];
  imu_buf->size++;
}

void imu_buf_clear(imu_buf_t *imu_buf) {
  for (int k = 0; k < imu_buf->size; k++) {
    timestamp_t *ts = &imu_buf->ts[k];
    real_t *acc = imu_buf->acc[k];
    real_t *gyr = imu_buf->gyr[k];

    *ts = 0;
    acc[0] = 0.0;
    acc[1] = 0.0;
    acc[2] = 0.0;
    gyr[0] = 0.0;
    gyr[1] = 0.0;
    gyr[2] = 0.0;
  }
  imu_buf->size = 0;
}

void imu_buf_copy(const imu_buf_t *from, imu_buf_t *to) {
  to->size = 0;
  for (int k = 0; k < from->size; k++) {
    to->ts[k] = from->ts[k];

    to->acc[k][0] = from->acc[k][0];
    to->acc[k][1] = from->acc[k][1];
    to->acc[k][2] = from->acc[k][2];

    to->gyr[k][0] = from->gyr[k][0];
    to->gyr[k][1] = from->gyr[k][1];
    to->gyr[k][2] = from->gyr[k][2];
  }
  to->size = from->size;
}

void imu_factor_setup(imu_factor_t *factor,
                      imu_params_t *imu_params,
                      imu_buf_t *imu_buf,
                      pose_t *pose_i,
                      speed_biases_t *sb_i,
                      pose_t *pose_j,
                      speed_biases_t *sb_j) {
  factor->imu_params = imu_params;
  imu_buf_copy(imu_buf, &factor->imu_buf);
  factor->pose_i = pose_i;
  factor->sb_i = sb_i;
  factor->pose_j = pose_j;
  factor->sb_j = sb_j;

  zeros(factor->covar, 15, 15);
  zeros(factor->r, 15, 1);
  factor->r_size = 15;

  factor->jacs[0] = factor->J0;
  factor->jacs[1] = factor->J1;
  factor->jacs[2] = factor->J2;
  factor->jacs[3] = factor->J3;
  factor->nb_params = 4;
}

void imu_factor_reset(imu_factor_t *factor) {
  zeros(factor->r, 15, 1);
  zeros(factor->J0, 2, 6);
  zeros(factor->J1, 2, 9);
  zeros(factor->J2, 2, 6);
  zeros(factor->J3, 2, 9);
}

int imu_factor_eval(imu_factor_t *factor) {
  assert(factor != NULL);

  /* factor->jacs[0] */


  return 0;
}

/* SOLVER ------------------------------------------------------------------- */

void solver_setup(solver_t *solver) {
  assert(solver);

  solver->nb_cam_factors = 0;
  solver->nb_imu_factors = 0;

  solver->nb_poses = 0;
  solver->nb_cams = 0;
  solver->nb_extrinsics = 0;
  solver->nb_features = 0;

  solver->x_size = 0;
  solver->r_size = 0;
}

void solver_print(solver_t *solver) {
  printf("solver:\n");
  printf("r_size: %d\n", solver->r_size);
  printf("x_size: %d\n", solver->x_size);
  printf("nb_cam_factors: %d\n", solver->nb_cam_factors);
  printf("nb_imu_factors: %d\n", solver->nb_imu_factors);
  printf("nb_poses: %d\n", solver->nb_poses);
}

static void solver_evaluator(solver_t *solver,
                             int **param_orders,
                             int *param_sizes,
                             int nb_params,
                             real_t *r,
                             int r_size,
                             real_t **jacs) {
  real_t *H = solver->H;
  int H_size = solver->x_size;
  real_t *g = solver->g;

  for (int i = 0; i < nb_params; i++) {
    int *idx_i = param_orders[i];
    int size_i = param_sizes[i];
    const real_t *J_i = jacs[i];

    real_t J_i_trans[MAX_H_SIZE] = {0};
    mat_transpose(J_i, r_size, size_i, J_i_trans);

    for (int j = i; j < nb_params; j++) {
      int *idx_j = param_orders[j];
      int size_j = param_sizes[i];
      const real_t *J_j = jacs[j];

      real_t H_ij[MAX_H_SIZE] = {0};
      dot(J_i_trans, size_i, r_size, J_j, r_size, size_j, H_ij);

      /* Fill Hessian H */
      /* H_ij = J_i' * J_j */
      /* H_ji = H_ij' */
      int stride = H_size;
      int rs = *idx_i;
      int cs = *idx_j;
      int re = rs + size_i;
      int ce = cs + size_j;
      if (i == j) {
        mat_block_set(H, stride, rs, cs, re, ce, H_ij);
      } else {
        real_t H_ji[MAX_H_SIZE] = {0};
        mat_transpose(H_ij, size_i, size_j, H_ji);
        mat_block_set(H, stride, rs, cs, re, ce, H_ij);
        mat_block_set(H, stride, cs, rs, ce, re, H_ij);
      }

      /* Fill in the R.H.S of H dx = g */
      /* g = -J_i * r */
      mat_scale(J_i_trans, H_size, r_size, -1);
      dot(J_i_trans, H_size, r_size, r, r_size, 1, g);
    }
  }

  /* Update parameter order */
  for (int i = 0; i < nb_params; i++) {
    param_orders[i] = param_orders[i] + param_sizes[i];
  }
}

int solver_eval(solver_t *solver) {
  assert(solver != NULL);

  int pose_idx = 0;
  int lmks_idx = solver->nb_poses * 6;
  int exts_idx = lmks_idx + solver->nb_features * 3;
  int cams_idx = exts_idx + solver->nb_extrinsics * 6;

  /* Evaluate camera factors */
  for (int i = 0; i < solver->nb_cam_factors; i++) {
    cam_factor_t *factor = &solver->cam_factors[i];
    cam_factor_eval(factor);

    int *param_orders[4] = {&pose_idx, &exts_idx, &cams_idx, &lmks_idx};
    int param_sizes[4] = {6, 6, 8, 3};
    int nb_params = 4;

    solver_evaluator(solver,
                     param_orders,
                     param_sizes,
                     nb_params,
                     factor->r,
                     factor->r_size,
                     factor->jacs);
  }

  return 0;
}

/* int solver_optimize(solver_t *solver) { */
/*   struct timespec solve_tic = tic(); */
/*   real_t lambda_k = 1e-4; */
/*  */
/*   int iter = 0; */
/*   int max_iter = 10; */
/*   int verbose = 1; */
/*  */
/*   for (iter = 0; iter < max_iter; iter++) { */
/*     #<{(| Cost k |)}># */
/*     #<{(| x = solver_get_state(solver); |)}># */
/*     #<{(| solver_eval(solver, H, g, &marg_size, &remain_size); |)}># */
/*     #<{(| const matx_t H_diag = (H.diagonal().asDiagonal()); |)}># */
/*     #<{(| H = H + lambda_k * H_diag; |)}># */
/*     #<{(| dx = H.ldlt().solve(g); |)}># */
/*     #<{(| e = solver_residuals(solver); |)}># */
/*     #<{(| cost = 0.5 * e.transpose() * e; |)}># */
/*  */
/*     #<{(| Cost k+1 |)}># */
/*     #<{(| solver_update(solver, dx); |)}># */
/*     #<{(| e = solver_residuals(solver); |)}># */
/*     const real_t cost_k = 0.5 * e.transpose() * e; */
/*  */
/*     const real_t cost_delta = cost_k - cost; */
/*     const real_t solve_time = toc(&solve_tic); */
/*     const real_t iter_time = (iter == 0) ? 0 : (solve_time / iter); */
/*  */
/*     if (verbose) { */
/*       printf("iter[%d] ", iter); */
/*       printf("cost[%.2e] ", cost); */
/*       printf("cost_k[%.2e] ", cost_k); */
/*       printf("cost_delta[%.2e] ", cost_delta); */
/*       printf("lambda[%.2e] ", lambda_k); */
/*       printf("iter_time[%.4f] ", iter_time); */
/*       printf("solve_time[%.4f]  ", solve_time); */
/*       printf("\n"); */
/*  */
/*       // // Calculate reprojection error */
/*       // size_t nb_keypoints = e.size() / 2.0; */
/*       // real_t sse = 0.0; */
/*       // for (size_t i = 0; i < nb_keypoints; i++) { */
/*       //   sse += pow(e.segment(i * 2, 2).norm(), 2); */
/*       // } */
/*       // const real_t rmse = sqrt(sse / nb_keypoints); */
/*       // printf("rmse reproj error: %.2f\n", rmse); */
/*     } */
/*  */
/*     #<{(| Determine whether to accept update |)}># */
/*     if (cost_k < cost) { */
/*       #<{(| Accept update |)}># */
/*       lambda_k /= update_factor; */
/*       cost = cost_k; */
/*     } else { */
/*       #<{(| Reject update |)}># */
/*       #<{(| solver_set_state(solver, x); // Restore state |)}># */
/*       lambda_k *= update_factor; */
/*     } */
/*  */
/*     #<{(| Termination criterias |)}># */
/*     if (fabs(cost_delta) < cost_change_threshold) { */
/*       break; */
/*     } else if ((solve_time + iter_time) > time_limit) { */
/*       break; */
/*     } */
/*   } */
/*  */
/*   #<{(| solve_time = toc(&solve_tic); |)}># */
/*   #<{(| if (verbose) { |)}># */
/*   #<{(|   printf("cost: %.2e\t", cost); |)}># */
/*   #<{(|   printf("solver took: %.4fs\n", solve_time); |)}># */
/*   #<{(| } |)}># */
/* } */
