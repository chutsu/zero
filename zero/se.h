#ifndef SE_H
#define SE_H

#include "zero.h"

#define NOT_SET -1
#define POSE 1
#define LANDMARK 2
#define CAMERA 3
#define EXTRINSIC 4
#define SPEED_BIAS 5

typedef uint64_t param_id_t;

struct param_t {
  int type;
  timestamp_t ts;
  param_id_t param_id;
};

/* struct pose_t { */
/*   struct param_t meta; */
/*  */
/*   double *param; */
/*   size_t param_size; */
/*   size_t min_param_size; */
/* }; */

struct landmark_t {
  struct param_t meta;

  double *param;
  size_t param_size;
  size_t min_param_size;
};

struct extrinsic_t {
  struct param_t meta;

  double *param;
  size_t param_size;
  size_t min_param_size;
};

struct camera_t {
  struct param_t meta;

  int cam_idx;
  int resolution[2];
  int proj_model;
  int dist_model;

  double *param;
  size_t param_size;
  size_t min_param_size;
};

void param_init(struct param_t *param);
/* void pose_init(struct pose_t *pose); */
void landmark_init(struct landmark_t *landmark);
void extrinsic_init(struct extrinsic_t *extrinsic);
void camera_init(struct camera_t *camera);

#endif // SE_H
