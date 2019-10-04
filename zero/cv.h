#ifndef CV_H
#define CV_H

#include <stdint.h>

#include "zero/math.h"

/*****************************************************************************
 * IMAGE
 *****************************************************************************/

typedef struct image_t {
  uint8_t *data;
  int width;
  int height;
} image_t;

void image_init(image_t *img, uint8_t *data, int width, int height);

/*****************************************************************************
 * PINHOLE
 *****************************************************************************/

void pinhole_K(const real_t fx,
               const real_t fy,
               const real_t cx,
               const real_t cy,
               mat3_t *K);
real_t pinhole_focal_length(const int image_width, const real_t fov);
int pinhole_project(const mat3_t *K, const vec3_t *p, vec2_t *x);
void pinhole_calc_K(const real_t image_width,
                    const real_t image_height,
                    const real_t lens_hfov,
                    const real_t lens_vfov,
                    mat3_t *K);

/*****************************************************************************
 * RADTAN
 *****************************************************************************/

void radtan4_distort(const real_t k1,
                     const real_t k2,
                     const real_t p1,
                     const real_t p2,
                     const pt2_t p,
                     pt2_t *p_d);

void radtan4_point_jacobian(const real_t k1,
                            const real_t k2,
                            const real_t p1,
                            const real_t p2,
                            const pt2_t p,
                            mat4_t *J_point);

void radtan4_param_jacobian(const real_t k1,
                            const real_t k2,
                            const real_t p1,
                            const real_t p2,
                            const pt2_t p,
                            mat_t *J_param);

/*****************************************************************************
 * EQUI
 *****************************************************************************/

void equi4_distort(const real_t k1,
                   const real_t k2,
                   const real_t k3,
                   const real_t k4,
                   const pt2_t p,
                   pt2_t *p_d);

void equi4_point_jacobian(const real_t k1,
                          const real_t k2,
                          const real_t k3,
                          const real_t k4,
                          const pt2_t p,
                          mat4_t *J_point);

void equi4_param_jacobian(const real_t k1,
                          const real_t k2,
                          const real_t k3,
                          const real_t k4,
                          const pt2_t p,
                          mat_t *J_param);

#endif  // CV_H
