#ifndef FACTOR_H
#define FACTOR_H

#include "zero/math.h"
#include "zero/cv.h"

typedef uint64_t timestamp_t;
#define LANDMARK_TYPE 0
#define POSE_TYPE 1

/*****************************************************************************
 * Landmark Variable
 ****************************************************************************/

typedef struct landmark_t {
  size_t id;
  size_t size;
  size_t local_size;

  double param[3];
} landmark_t;

void landmark_setup(landmark_t *lm, const size_t id, const double p_W[3]) {
  lm->id = id;
  lm->size = 3;
  lm->local_size = 3;

  lm->param[0] = p_W[0];
  lm->param[1] = p_W[1];
  lm->param[2] = p_W[2];
}

void landmark_plus(landmark_t *lm, const double dx[3]) {
  lm->param[0] = lm->param[0] + dx[0];
  lm->param[1] = lm->param[1] + dx[1];
  lm->param[2] = lm->param[2] + dx[2];
}

/*****************************************************************************
 * Pose Variable
 ****************************************************************************/

typedef struct pose_t {
  timestamp_t ts;
  size_t id;
  size_t size;
  size_t local_size;

  double param[7]; // (qw, qx, qy, qz), (x, y, z)
} pose_t;

void pose_setup(pose_t *pose,
                const timestamp_t ts,
                const size_t id,
                const double T[16]) {
	assert(pose != NULL);
	assert(T != NULL);

  pose->ts = ts;
  pose->id = id;
  pose->size = 7;
  pose->local_size = 6;

  double q[4] = {0};
  tf_quat(T, q);
  pose->param[0] = q[0];
  pose->param[1] = q[1];
  pose->param[2] = q[2];
  pose->param[3] = q[3];

  double r[3] = {0};
  pose->param[4] = r[0];
  pose->param[5] = r[1];
  pose->param[6] = r[2];
}

void pose_quat(const pose_t *pose, double q[4]) {
	assert(pose != NULL);
	assert(q != NULL);

	q[0] = pose->param[0];
	q[1] = pose->param[1];
	q[2] = pose->param[2];
	q[3] = pose->param[3];
}

void pose_rot(const pose_t *pose, double C[9]) {
	assert(pose != NULL);
	assert(C != NULL);

  const double q[4] = {pose->param[0],
                       pose->param[1],
                       pose->param[2],
                       pose->param[3]};
  quat2rot(q, C);
}

void pose_tf(const pose_t *pose, double T[16]) {
	assert(pose != NULL);
	assert(T != NULL);

	/* Setup */
  const double q[4] = {pose->param[0],
                       pose->param[1],
                       pose->param[2],
                       pose->param[3]};
  const double r[3] = {pose->param[4],
                       pose->param[5],
                       pose->param[6]};

	/* Convert quaternion to rotation matrix */
	double C[9];
  quat2rot(q, C);

	/* Form transform matrix */
	/* clang-format off */
	T[0] = C[0]; T[1] = C[1]; T[2] = C[2]; T[3] = r[0];
	T[4] = C[3]; T[5] = C[4]; T[6] = C[5]; T[7] = r[1];
	T[8] = C[6]; T[9] = C[7]; T[10] = C[8]; T[11] = r[2];
	T[12] = 0.0; T[13] = 0.0; T[14] = 0.0; T[15] = 1.0;
	/* clang-format on */
}

void pose_plus(pose_t *pose, const double dx[6]) {
	/* Quaternion from pose */
  double q[4];
  pose_quat(pose, q);

  /* Calculate dq from rotation vector */
  const double rvec[3] = {dx[0], dx[1], dx[2]};
  const double half_norm = 0.5 * vec_norm(rvec, 3);
	/* clang-format off */
  const double dq[4] = {cos(half_norm),
                        sinc(half_norm) * 0.5 * rvec[0],
                        sinc(half_norm) * 0.5 * rvec[1],
                        sinc(half_norm) * 0.5 * rvec[2]};
	/* clang-format on */

	/* Update rotation component */
	double q_updated[4] = {0};
	quatmul(q, dq, q_updated);
  pose->param[0] = q_updated[0];
  pose->param[1] = q_updated[1];
  pose->param[2] = q_updated[2];
  pose->param[3] = q_updated[3];

  /* Update translation component */
  pose->param[4] = pose->param[4] - dx[3];
  pose->param[5] = pose->param[5] - dx[4];
  pose->param[6] = pose->param[6] - dx[5];
}

// /*****************************************************************************
//  * Factor
//  ****************************************************************************/
//
// struct factor_t {
//   timestamp_t ts = 0;
//   size_t id = 0;
//   size_t residual_size = 0;
//   std::vector<variable_t *> param_blocks;
//   std::vector<size_t> param_sizes;
//
//   factor_t() {}
//   factor_t(const timestamp_t &ts_,
//            const size_t id_,
//            const size_t residual_size_,
//            const std::vector<variable_t *> param_blocks_,
//            const std::vector<size_t> param_sizes_) :
//     ts{ts_}, id{id_}, residual_size{residual_size_},
//     param_blocks{param_blocks_}, param_sizes{param_sizes_}  {}
//   virtual ~factor_t() {}
//
//   virtual int eval(double *residuals, double **jacobians) const = 0;
// };
//
// /*****************************************************************************
//  * Bundle Adjustment Factor
//  ****************************************************************************/
//
// struct ba_factor_t : factor_t {
//   vec2_t z = zeros(2, 1);
//   landmark_t *p_W = nullptr;
//   pose_t *T_WC = nullptr;
//
//   mat2_t info = I(2);
//   mat2_t sq_info = zeros(2, 2);
//
//   ba_factor_t(const timestamp_t &ts_,
//               const size_t id_,
//               const vec2_t &z_,
//               landmark_t *p_W_,
//               pose_t *T_WC_,
//               mat2_t info_=I(2))
//       : factor_t{ts_, id_, 2, {p_W_, T_WC_}, {3, 6}},
//         z{z_},
//         p_W{p_W_},
//         T_WC{T_WC_},
//         info{info_} {
//     Eigen::LLT<mat2_t> llt_info(info);
//     sq_info = llt_info.matrixL().transpose();
//   }
//
//   int eval(double *residuals, double **jacobians) const;
// };
//
// /*****************************************************************************
//  * Camera Factor
//  ****************************************************************************/
//
// // template <typename CM, typename DM>
// struct cam_factor_t : factor_t {
// 	// camera_geometry_t<CM, DM> camera;
//   vec2_t z = zeros(2, 1);
//   landmark_t *p_W = nullptr;
//   pose_t *T_WS = nullptr;
//   pose_t *T_SC = nullptr;
//
//   mat2_t info = I(2);
//   mat2_t sq_info = zeros(2, 2);
//
//   cam_factor_t(const timestamp_t &ts_,
//                const size_t id_,
//                const vec2_t &z_,
//                landmark_t *p_W_,
//                pose_t *T_WS_=nullptr,
//                pose_t *T_SC_=nullptr,
//                mat2_t info_=I(2))
//       : factor_t{ts_, id_, 2, {p_W_, T_WS_, T_SC_}, {3, 6, 6}},
//         z{z_},
//         p_W{p_W_},
//         T_WS{T_WS_},
//         T_SC{T_SC_},
//         info{info_} {
//     Eigen::LLT<mat2_t> llt_info(info);
//     sq_info = llt_info.matrixL().transpose();
//   }
//
//   /**
//    * Evaluate
//    */
//   int eval(double *residuals, double **jacobians) const;
// };
//
// /*****************************************************************************
//  * IMU Factor
//  ****************************************************************************/
//
// // struct imu_error_t : factor_t {
// //   timestamps_t ts;
// //   vec3s_t gyro;
// //   vec3s_t accel;
// //
// //   pose_t *T_WS_0 = nullptr;
// //   pose_t *T_WS_1 = nullptr;
// //
// //   imu_error_t() {}
// //   virtual ~imu_error_t() {}
// // };
//
// /*****************************************************************************
//  * Factor Graph
//  ****************************************************************************/
//
// struct graph_t {
//   std::unordered_map<size_t, variable_t *> variables;
//   std::vector<factor_t *> factors;
//
//   size_t residual_size = 0;
//   size_t nb_poses = 0;
//   size_t nb_landmarks = 0;
//
//   std::unordered_map<factor_t *, vecx_t> residuals;
//   std::unordered_map<factor_t *, std::vector<matx_t>> jacobians;
//   std::unordered_map<variable_t *, size_t> param_index;
// };
//
// void graph_free(graph_t &graph);
// size_t graph_next_variable_id(graph_t &graph);
// size_t graph_next_factor_id(graph_t &graph);
// size_t graph_add_pose(graph_t &graph, const timestamp_t &ts, const mat4_t &pose);
// size_t graph_add_landmark(graph_t &graph, const vec3_t &landmark);
// size_t graph_add_factor(graph_t &graph, factor_t *factor);
// size_t graph_add_ba_factor(graph_t &graph,
//                            const timestamp_t &ts,
//                            const vec2_t &z,
//                            landmark_t *p_W,
//                            pose_t *T_WS);
// // size_t graph_add_camera_factor(graph_t &graph,
// //                                const timestamp_t &ts,
// //                                const int cam_idx,
// //                                const vec2_t &z,
// //                                const vec3_t &p_W,
// //                                const mat4_t &T_WC);
// int graph_eval(graph_t &graph);
// void graph_setup_problem(graph_t &graph, matx_t &J, vecx_t &r);
// void graph_update(graph_t &graph, const vecx_t &dx);
// int graph_solve(graph_t &graph, int max_iter=30);

#endif // FACTOR_H
