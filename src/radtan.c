#include "zero/radtan.h"

void radtan4_distort(const real_t k1,
                     const real_t k2,
                     const real_t p1,
                     const real_t p2,
                     const pt2_t p,
                     pt2_t *p_d) {
    /* Point */
  const real_t x = p[0];
  const real_t y = p[1];

  /* Apply radial distortion */
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;
  const real_t radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
  const real_t x_dash = x * radial_factor;
  const real_t y_dash = y * radial_factor;

    /* Apply tangential distortion */
    const real_t xy = x * y;
    const real_t x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
    const real_t y_ddash = y_dash + (p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy);

    /* Distorted point */
  *p_d[0] = x_ddash;
  *p_d[1] = y_ddash;
}

void radtan4_point_jacobian(const real_t k1,
                                  const real_t k2,
                                  const real_t p1,
                                  const real_t p2,
                                  const pt2_t p,
                                  mat4_t *J_point) {
    /* Point */
  const real_t x = p[0];
  const real_t y = p[1];

  /* Apply radial distortion */
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  /* Point Jacobian */
    *J_point[0] = k1 * r2 + k2 * r4 + 2 * p1 * y + 6 * p2 * x + x * (2 * k1 * x + 4 * k2 *x * r2) + 1;
    *J_point[1] = 2 * p1 * x + 2 * p2 * y + y * (2 * k1 * x + 4 * k2 * x * r2);
    *J_point[2] = *J_point[1];
    *J_point[3] = k1 * r2 + k2 * r4 + 6 * p1 * y + 2 * p2 * x + y * (2 * k1 * y + 4 * k2 * y * r2) + 1;
}

void radtan4_param_jacobian(const real_t k1,
                                  const real_t k2,
                                  const real_t p1,
                                  const real_t p2,
                                  const pt2_t p,
                                  mat_t *J_param) {
    /* Point */
  const real_t x = p[0];
  const real_t y = p[1];

  /* Setup */
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t xy = x * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  /* Param Jacobian */
  J_param->data[0] = x * r2;
  J_param->data[1] = x * r4;
  J_param->data[2] = 2 * xy;
  J_param->data[3] = 3 * x2 + y2;

  J_param->data[4] = y * r2;
  J_param->data[5] = y * r4;
  J_param->data[6] = x2 + 3 * y2;
  J_param->data[7] = 2 * xy;
}
