#ifndef PINHOLE_H
#define PINHOLE_H

#include "zero/core.h"

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

#endif // PINHOLE_H
