#ifndef EQUI_H
#define EQUI_H

#include "zero/core.h"

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

#endif // EQUI_H
