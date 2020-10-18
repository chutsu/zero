#include "se.h"

void param_init(struct param_t *param) {
  param->type = NOT_SET;
  param->ts = NOT_SET;
  param->param_id = NOT_SET;
}

/* void pose_init(struct pose_t *pose, const ) { */
/*  */
/* } */

void landmark_init(struct landmark_t *landmark) {
  landmark->meta.type = NOT_SET;
  landmark->meta.ts = NOT_SET;
  landmark->meta.param_id = NOT_SET;

}

void extrinsic_init(struct extrinsic_t *extrinsic) {

}

void camera_init(struct camera_t *camera) {

}
