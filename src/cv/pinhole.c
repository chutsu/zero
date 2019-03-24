#include "zero/cv/pinhole.h"

void pinhole_K(const real_t fx,
               const real_t fy,
               const real_t cx,
               const real_t cy,
               mat3_t *K) {
  *K[0] = fx;
  *K[1] = 0.0;
  *K[2] = cx;
  *K[3] = 0.0;
  *K[4] = fy;
  *K[5] = cy;
  *K[6] = 0.0;
  *K[7] = 0.0;
  *K[8] = 1.0;
}

real_t pinhole_focal_length(const int image_width, const real_t fov) {
  return ((image_width / 2.0) / tan(deg2rad(fov) / 2.0));
}

int pinhole_project(const mat3_t *K, const vec3_t *p, vec2_t *x) {
  const real_t fx = *K[0];
  const real_t fy = *K[4];
  const real_t cx = *K[2];
  const real_t cy = *K[5];

  *x[0] = *p[0] * fx + cx;
  *x[1] = *p[1] * fy + cy;

  return 0;
}

void pinhole_calc_K(const real_t image_width,
                    const real_t image_height,
                    const real_t lens_hfov,
                    const real_t lens_vfov,
                    mat3_t *K) {
  const real_t fx = pinhole_focal_length(image_width, lens_hfov);
  const real_t fy = pinhole_focal_length(image_height, lens_vfov);
  const real_t cx = image_width / 2.0;
  const real_t cy = image_height / 2.0;
  return pinhole_K(fx, fy, cx, cy, K);
}
