#ifndef BA_H
#define BA_H

#include <string.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>

#include "zero/zero.h"

static void load_camera(const char *data_path, double K[3 * 3]) {
  /* Setup csv path */
  char cam_csv[1000] = {0};
  strcat(cam_csv, data_path);
  strcat(cam_csv, "/camera.csv");

  /* Parse csv file */
  int nb_rows = 0;
  int nb_cols = 0;
  double **cam_K = csv_data(cam_csv, &nb_rows, &nb_cols);
  if (cam_K == NULL) {
    FATAL("Failed to load csv file [%s]!", cam_csv);
  }
  if (nb_rows != 3 || nb_cols != 3) {
    LOG_ERROR("Error while parsing camera file [%s]!", cam_csv);
    LOG_ERROR("-- Expected 3 rows got %d instead!", nb_rows);
    LOG_ERROR("-- Expected 3 cols got %d instead!", nb_cols);
    FATAL("Invalid camera file [%s]!", cam_csv);
  }

  /* Flatten 2D array to 1D array */
  int index = 0;
  for (int i = 0; i < nb_rows; i++) {
    for (int j = 0; j < nb_cols; j++) {
      K[index] = cam_K[i][j];
      index++;
    }
    free(cam_K[i]);
  }
  free(cam_K);
}

static pose_t *load_camera_poses(const char *data_path, int *nb_cam_poses) {
  char cam_poses_csv[1000] = {0};
  strcat(cam_poses_csv, data_path);
  strcat(cam_poses_csv, "/camera_poses.csv");
  return load_poses(cam_poses_csv, nb_cam_poses);
}

static pose_t *load_target_pose(const char *data_path) {
  char target_pose_csv[1000] = {0};
  strcat(target_pose_csv, data_path);
  strcat(target_pose_csv, "/target_pose.csv");

  int nb_poses = 0;
  return load_poses(target_pose_csv, &nb_poses);
}

struct keypoints_t {
  double **data;
  int size;
} typedef keypoints_t;

void keypoints_free(keypoints_t *keypoints) {
  for (int i = 0; i < keypoints->size; i++) {
    free(keypoints->data[i]);
  }
  free(keypoints->data);
  free(keypoints);
}

void keypoints_print(const keypoints_t *keypoints) {
  printf("nb_keypoints: %d\n", keypoints->size);
  printf("keypoints:\n");
  for (int i = 0; i < keypoints->size; i++) {
    printf("-- (%f, %f)\n", keypoints->data[i][0], keypoints->data[i][1]);
  }
}

static keypoints_t *parse_keypoints_line(char *line) {
  keypoints_t *keypoints = calloc(1, sizeof(keypoints_t));
  keypoints->data = NULL;
  keypoints->size = 0;

  char entry[100] = {0};
  int kp_ready = 0;
  double kp[2] = {0};
  int kp_index = 0;

  /* Parse line */
  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      /* Initialize keypoints */
      if (keypoints->data == NULL) {
        size_t array_size = strtod(entry, NULL);
        keypoints->data = calloc(array_size, sizeof(double *));
        keypoints->size = array_size / 2.0;

      } else { /* Parse keypoint */
        if (kp_ready == 0) {
          kp[0] = strtod(entry, NULL);
          kp_ready = 1;

        } else {
          kp[1] = strtod(entry, NULL);
          keypoints->data[kp_index] = malloc(sizeof(double) * 2);
          keypoints->data[kp_index][0] = kp[0];
          keypoints->data[kp_index][1] = kp[1];

          kp_ready = 0;
          kp_index++;
        }
      }

      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  return keypoints;
}

static keypoints_t **load_keypoints(const char *data_path, int *nb_frames) {
  char keypoints_csv[1000] = {0};
  strcat(keypoints_csv, data_path);
  strcat(keypoints_csv, "/keypoints.csv");

  FILE *csv_file = fopen(keypoints_csv, "r");
  *nb_frames = csv_rows(keypoints_csv);
  keypoints_t **keypoints = calloc(*nb_frames, sizeof(keypoints_t *));

  char line[1024] = {0};
  int frame_idx = 0;
  while (fgets(line, 1024, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    keypoints[frame_idx] = parse_keypoints_line(line);
    frame_idx++;
  }
  fclose(csv_file);

  return keypoints;
}

static double **load_points(const char *data_path, int *nb_points) {
  char points_csv[1000] = {0};
  strcat(points_csv, data_path);
  strcat(points_csv, "/points.csv");

  /* Initialize memory for points */
  *nb_points = csv_rows(points_csv);
  double **points = malloc(sizeof(double *) * *nb_points);
  for (int i = 0; i < *nb_points; i++) {
    points[i] = malloc(sizeof(double) * 3);
  }

  /* Load file */
  FILE *infile = fopen(points_csv, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  /* Loop through data */
  char line[1024] = {0};
  size_t len_max = 1024;
  int point_idx = 0;
  int col_idx = 0;

  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        points[point_idx][col_idx] = strtod(entry, NULL);
        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    point_idx++;
  }

  /* Cleanup */
  fclose(infile);

  return points;
}

static int **load_point_ids(const char *data_path, int *nb_points) {
  char csv_path[1000] = {0};
  strcat(csv_path, data_path);
  strcat(csv_path, "/point_ids.csv");
  return load_iarrays(csv_path, nb_points);
}

struct ba_data_t {
  double cam_K[3 * 3];

  int nb_frames;
  pose_t *cam_poses;
  pose_t *target_pose;

  keypoints_t **keypoints;
  int **point_ids;

  double **points;
  int nb_points;
} typedef ba_data_t;

ba_data_t *ba_load_data(const char *data_path) {
  ba_data_t *data = malloc(sizeof(ba_data_t));

  int nb_ids = 0;
  load_camera(data_path, data->cam_K);
  data->cam_poses = load_camera_poses(data_path, &data->nb_frames);
  data->target_pose = load_target_pose(data_path);
  data->keypoints = load_keypoints(data_path, &data->nb_frames);
  data->point_ids = load_point_ids(data_path, &nb_ids);
  data->points = load_points(data_path, &data->nb_points);

  return data;
}

void ba_data_free(ba_data_t *data) {
  /* Camera poses */
  free(data->cam_poses);

  /* Target pose */
  free(data->target_pose);

  /* Keypoints */
  for (int i = 0; i < data->nb_frames; i++) {
    keypoints_free(data->keypoints[i]);
  }
  free(data->keypoints);

  /* Point IDs */
  for (int i = 0; i < data->nb_frames; i++) {
    free(data->point_ids[i]);
  }
  free(data->point_ids);

  /* Points */
  for (int i = 0; i < data->nb_points; i++) {
    free(data->points[i]);
  }
  free(data->points);

  free(data);
}

int ba_residual_size(ba_data_t *data) {
  /* Calculate residual size */
  int r_size = 0;
  for (int k = 0; k < data->nb_frames; k++) {
    r_size += data->point_ids[k][0];
  }
  r_size = r_size * 2;
  /* ^ Scale 2 because each pixel error are size 2 */

  return r_size;
}

double *ba_residuals(ba_data_t *data, int *r_size) {
  /* Initialize memory for residuals */
  *r_size = ba_residual_size(data);
  double *r = calloc(*r_size, sizeof(double));

  /* Target pose */
  double T_WT[4 * 4] = {0};
  pose2tf(data->target_pose, T_WT);

  /* Loop over time */
  int res_idx = 0; /* Residual index */
  for (int k = 0; k < data->nb_frames; k++) {
    /* Form camera pose */
    double T_WC[4 * 4] = {0};
    pose2tf(&data->cam_poses[k], T_WC);

    /* Invert camera pose T_WC to T_CW */
    double T_CW[4 * 4] = {0};
    tf_inv(T_WC, T_CW);

    /* Get point ids and measurements at time step k */
    const int nb_ids = data->point_ids[k][0];
    const int *point_ids = &data->point_ids[k][1];

    for (int i = 0; i < nb_ids; i++) {
      /* Get point in world frame */
      const int id = point_ids[i];
      const double *p_W = data->points[id];

      /* Transform point in world frame to camera frame */
      double p_C[3] = {0};
      tf_point(T_CW, p_W, p_C);

      /* Project point in camera frame down to image plane */
      double z_hat[2] = {0};
      pinhole_project(data->cam_K, p_C, z_hat);

      /* Calculate reprojection error */
      const double *z = data->keypoints[k]->data[i];
      r[res_idx] = z[0] - z_hat[0];
      r[res_idx + 1] = z[1] - z_hat[1];
      res_idx += 2;
    }
  }

  return r;
}

static void J_intrinsics_point(const double K[3 * 3], double J[2 * 2]) {
  /* J = [K[0, 0], 0.0,  */
  /* 		  0.0, K[1, 1]]; */
  zeros(J, 2, 2);
  J[0] = K[0];
  J[3] = K[4];
}

static void J_project(const double p_C[3], double J[2 * 3]) {
  const double x = p_C[0];
  const double y = p_C[1];
  const double z = p_C[2];

  /* J = [1 / z, 0, -x / z^2, */
  /* 		  0, 1 / z, -y / z^2]; */
  zeros(J, 2, 3);
  J[0] = 1.0 / z;
  J[2] = -x / (z * z);
  J[4] = 1.0 / z;
  J[5] = -y / (z * z);
}

static void J_camera_rotation(const double q_WC[4],
                              const double r_WC[3],
                              const double p_W[3],
                              double J[3 * 3]) {
  /* Convert quaternion to rotatoin matrix */
  double C_WC[3 * 3] = {0};
  quat2rot(q_WC, C_WC);

  /* J = C_WC * skew(p_W - r_WC); */
  double C_CW[3 * 3] = {0};
  mat_transpose(C_WC, 3, 3, C_CW);

  double x[3] = {0};
  vec_sub(p_W, r_WC, x, 3);

  double S[3 * 3] = {0};
  skew(x, S);

  dot(C_CW, 3, 3, S, 3, 3, J);
}

static void J_camera_translation(const double q_WC[4], double J[3 * 3]) {
  /* Convert quaternion to rotatoin matrix */
  double C_WC[3 * 3] = {0};
  quat2rot(q_WC, C_WC);

  /* J = -C_CW */
  mat_transpose(C_WC, 3, 3, J);
  mat_scale(J, 3, 3, -1.0);
}

static void J_target_point(const double q_WC[4], double J[3 * 3]) {
  /* Convert quaternion to rotatoin matrix */
  double C_WC[3 * 3] = {0};
  quat2rot(q_WC, C_WC);

  /* J = C_CW */
  mat_transpose(C_WC, 3, 3, J);
}

double *ba_jacobian(ba_data_t *data, int *J_rows, int *J_cols) {
  /* Initialize memory for jacobian */
  *J_rows = ba_residual_size(data);
  *J_cols = (data->nb_frames * 6) + (data->nb_points * 3);
  double *J = calloc(*J_rows * *J_cols, sizeof(double));
  zeros(J, *J_rows, *J_cols);

  /* Loop over camera poses */
  int pose_idx = 0;
  int meas_idx = 0;

  for (int k = 0; k < data->nb_frames; k++) {
    /* Form camera pose */
    double T_WC[4 * 4] = {0};
    double q_WC[4] = {0};
    double r_WC[3] = {0};
    pose2tf(&data->cam_poses[k], T_WC);
    tf_quat_get(T_WC, q_WC);
    tf_trans_get(T_WC, r_WC);

    /* Invert T_WC to T_CW */
    double T_CW[4 * 4] = {0};
    tf_inv(T_WC, T_CW);

    /* Get point ids and measurements at time step k */
    const int nb_ids = data->point_ids[k][0];
    const int *point_ids = &data->point_ids[k][1];

    /* Loop over observations at time k */
    for (int i = 0; i < nb_ids; i++) {
      /* Get point in world frame */
      const int id = point_ids[i];
      const double *p_W = data->points[id];

      /* Transform point in world frame to camera frame */
      double p_C[3] = {0};
      tf_point(T_CW, p_W, p_C);

      /* Camera pose jacobian */
      /* -- Setup row start, row end, column start and column end */
      const int rs = meas_idx * 2;
      const int re = rs + 1;
      int cs = pose_idx * 6;
      int ce = cs + 5;

      /* -- Form jacobians */
      double J_K[2 * 2] = {0};
      double J_P[2 * 3] = {0};
      double J_C[3 * 3] = {0};
      double J_r[3 * 3] = {0};
      J_intrinsics_point(data->cam_K, J_K);
      J_project(p_C, J_P);
      J_camera_rotation(q_WC, r_WC, p_W, J_C);
      J_camera_translation(q_WC, J_r);
      /* -- J_cam_rot = -1 * J_K * J_P * J_C; */
      double J_KP[2 * 3] = {0};
      double J_cam_rot[2 * 3] = {0};
      dot(J_K, 2, 2, J_P, 2, 3, J_KP);
      dot(J_KP, 2, 3, J_C, 3, 3, J_cam_rot);
      mat_scale(J_cam_rot, 2, 3, -1);
      /* -- J_cam_pos = -1 * J_K * J_P * J_r; */
      double J_cam_pos[2 * 3] = {0};
      dot(J_K, 2, 2, J_P, 2, 3, J_KP);
      dot(J_KP, 2, 3, J_r, 3, 3, J_cam_pos);
      mat_scale(J_cam_pos, 2, 3, -1);
      /* -- Fill in the big jacobian */
      mat_block_set(J, *J_cols, rs, cs, re, cs + 2, J_cam_rot);
      mat_block_set(J, *J_cols, rs, cs + 3, re, ce, J_cam_pos);

      /* Point jacobian */
      /* -- Setup row start, row end, column start and column end */
      cs = (data->nb_frames * 6) + point_ids[i] * 3;
      ce = cs + 2;
      /* -- Form jacobians */
      double J_p[3 * 3] = {0};
      J_target_point(q_WC, J_p);
      /* -- J_point = -1 * J_K * J_P * J_target_point(q_WC); */
      double J_point[2 * 3] = {0};
      dot(J_KP, 2, 3, J_p, 3, 3, J_point);
      mat_scale(J_point, 2, 3, -1);
      /* -- Fill in the big jacobian */
      mat_block_set(J, *J_cols, rs, cs, re, ce, J_point);

      meas_idx++;
    }
    pose_idx++;
  }

  return J;
}

void ba_update(ba_data_t *data,
               double *e,
               int e_size, double
               *E,
               int E_rows,
               int E_cols) {
  assert(e_size == E_rows);
  /* Form weight matrix */
  /* W = diag(repmat(sigma, data->nb_measurements, 1)); */

  /* Solve Gauss-Newton system [H dx = g]: Solve for dx */
	/* -- Calculate L.H.S of Gauss-Newton */
  /* H = (E' * W * E); */
  double *E_t = mat_new(E_cols, E_rows);
  double *H = mat_new(E_cols, E_cols);
  mat_transpose(E, E_rows, E_cols, E_t);
  dot(E_t, E_cols, E_rows, E, E_rows, E_cols, H);

  /* -- Apply damping */
	/* H = H + lambda * I */
  double *damp_term = mat_new(E_cols, E_cols);
  double *H_damped = mat_new(E_cols, E_cols);
  eye(damp_term, E_cols, E_cols);
  mat_scale(damp_term, E_cols, E_cols, 1e-6);
  mat_add(H, damp_term, H_damped, E_cols, E_cols);

	/* H = H + lambda * H_diag */
  /* double *h_diag = vec_new(E_cols); */
  /* double *H_diag = mat_new(E_cols, E_cols); */
  /* double *H_damped = mat_new(E_cols, E_cols); */
  /* mat_diag_get(H, E_cols, E_cols, h_diag); */
  /* mat_diag_set(H_diag, E_cols, E_cols, h_diag); */
  /* mat_scale(H_diag, E_cols, E_cols, 1000.0); */
  /* mat_add(H, H_diag, H_damped, E_cols, E_cols); */

	/* -- Calculate R.H.S of Gauss-Newton */
  /* g = -E' * W * e; */
  double *g = vec_new(E_cols);
  mat_scale(E_t, E_cols, E_rows, -1.0);
  dot(E_t, E_cols, E_rows, e, e_size, 1, g);
  free(E_t);

  /* Use Cholesky on [H dx = g] to solve for dx */
  /* double *dx = vec_new(E_cols); */
  /* chol_lls_solve(H_damped, g, dx, E_cols); */
  /* chol_lls_solve2(H_damped, g, dx, E_cols); */
  /* free(H); */
  /* free(H_hat); */
  /* free(g); */

  double *dx = vec_new(E_cols);
  {
    gsl_matrix *A =  gsl_matrix_alloc(E_cols, E_cols);
    int idx = 0;
    for (int i = 0; i < E_cols; i++) {
      for (int j = 0; j < E_cols; j++) {
        gsl_matrix_set(A, i, j, H_damped[idx]);
        idx++;
      }
    }

    gsl_vector *b =  gsl_vector_alloc(E_cols);
    for (int i = 0; i < E_cols; i++) {
      gsl_vector_set(b, i, g[i]);
    }

    gsl_vector *x =  gsl_vector_alloc(E_cols);
    gsl_linalg_cholesky_decomp1(A);
    gsl_linalg_cholesky_solve(A, b, x);
    for (int i = 0; i < E_cols; i++) {
      dx[i] = gsl_vector_get(x, i);
    }
  }

  /* Update camera poses */
  for (int k = 0; k < data->nb_frames; k++) {
    const int s = k * 6;

    /* Update camera rotation */
    /* dq = quatdelta(dalpha) */
    /* q_WC_k = quatmul(dq, q_WC_k) */
    const double dalpha[3] = {dx[s], dx[s + 1], dx[s + 2]};
    double dq[4] = {0};
    double q_new[4] = {0};
    quatdelta(dalpha, dq);
    quatmul(dq, data->cam_poses[k].q, q_new);
    data->cam_poses[k].q[0] = q_new[0];
    data->cam_poses[k].q[1] = q_new[1];
    data->cam_poses[k].q[2] = q_new[2];
    data->cam_poses[k].q[3] = q_new[3];

    /* Update camera position */
    /* r_WC_k += dr_WC */
    const double dr_WC[3] = {dx[s + 3], dx[s + 4], dx[s + 5]};
    data->cam_poses[k].r[0] += dr_WC[0];
    data->cam_poses[k].r[1] += dr_WC[1];
    data->cam_poses[k].r[2] += dr_WC[2];
  }

  /* Update points */
  for (int i = 0; i < data->nb_points; i++) {
    const int s = (data->nb_frames * 6) + (i * 3);
    const double dp_W[3] = {dx[s], dx[s + 1], dx[s + 2]};
    data->points[i][0] += dp_W[0];
    data->points[i][1] += dp_W[1];
    data->points[i][2] += dp_W[2];
  }

  /* Clean up */
  free(dx);
}

double ba_cost(const double *e, const int length) {
  /* cost = 0.5 * e' * e */
  double cost = 0.0;
  dot(e, 1, length, e, length, 1, &cost);
  return cost * 0.5;
}

void ba_solve(ba_data_t *data) {
  int max_iter = 1000;
  double cost_prev = 0.0;

  for (int iter = 0; iter < max_iter; iter++) {
    /* Residuals */
    int e_size = 0;
    double *e = ba_residuals(data, &e_size);

    /* Jacobians */
    int E_rows = 0;
    int E_cols = 0;
    double *E = ba_jacobian(data, &E_rows, &E_cols);

    /* Update and calculate cost */
    ba_update(data, e, e_size, E, E_rows, E_cols);
    const double cost = ba_cost(e, e_size);
    printf("iter: %d\t cost: %.4e\n", iter, cost);
    /* printf("iter: %d\t cost: %f\n", iter, cost); */
    free(e);
    free(E);

    /* Termination criteria */
    double cost_diff = fabs(cost - cost_prev);
    if (cost_diff < 1.0e-6) {
      printf("Done!\n");
      break;
    }
    cost_prev = cost;
  }
}

#endif // BA_H
