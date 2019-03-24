#ifndef RADTAN_H
#define RADTAN_H

#include "zero/core.h"

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

#endif // RADTAN_H
