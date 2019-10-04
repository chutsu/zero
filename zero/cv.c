#include "zero/cv.h"

/*****************************************************************************
 * IMAGE
 *****************************************************************************/

void image_init(image_t *img, uint8_t *data, int width, int height) {
  img->data = data;
  img->width = width;
  img->height = height;
}

/*****************************************************************************
 * PINHOLE
 *****************************************************************************/

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

/*****************************************************************************
 * RADTAN
 *****************************************************************************/

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
  *J_point[0] = k1 * r2 + k2 * r4 + 2 * p1 * y + 6 * p2 * x +
                x * (2 * k1 * x + 4 * k2 * x * r2) + 1;
  *J_point[1] = 2 * p1 * x + 2 * p2 * y + y * (2 * k1 * x + 4 * k2 * x * r2);
  *J_point[2] = *J_point[1];
  *J_point[3] = k1 * r2 + k2 * r4 + 6 * p1 * y + 2 * p2 * x +
                y * (2 * k1 * y + 4 * k2 * y * r2) + 1;
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

/*****************************************************************************
 * EQUI
 *****************************************************************************/

void equi4_distort(const real_t k1,
                   const real_t k2,
                   const real_t k3,
                   const real_t k4,
                   const pt2_t p,
                   pt2_t *p_d) {
  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th4 = th2 * th2;
  const real_t th6 = th4 * th2;
  const real_t th8 = th4 * th4;
  const real_t thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const real_t s = thd / r;

  const real_t x_dash = s * x;
  const real_t y_dash = s * y;

  *p_d[0] = x_dash;
  *p_d[1] = y_dash;
}

void equi4_point_jacobian(const real_t k1,
                          const real_t k2,
                          const real_t k3,
                          const real_t k4,
                          const pt2_t p,
                          mat4_t *J_point) {
  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th4 = th2 * th2;
  const real_t th6 = th4 * th2;
  const real_t th8 = th4 * th4;
  const real_t thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);

  const real_t th_r = 1.0 / (r * r + 1.0);
  const real_t thd_th =
      1.0 + 3.0 * k1 * th2 + 5.0 * k2 * th4 + 7.0 * k3 * th6 + 9.0 * k4 * th8;
  const real_t s = thd / r;
  const real_t s_r = thd_th * th_r / r - thd / (r * r);
  const real_t r_x = 1.0 / r * x;
  const real_t r_y = 1.0 / r * y;

  *J_point[0] = s + x * s_r * r_x;
  *J_point[1] = x * s_r * r_y;
  *J_point[2] = y * s_r * r_x;
  *J_point[3] = s + y * s_r * r_y;
}

void equi4_param_jacobian(const real_t k1,
                          const real_t k2,
                          const real_t k3,
                          const real_t k4,
                          const pt2_t p,
                          mat_t *J_param) {
  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th3 = th2 * th;
  const real_t th5 = th3 * th2;
  const real_t th7 = th5 * th2;
  const real_t th9 = th7 * th2;

  assert(J_param->rows == 2);
  assert(J_param->cols == 4);
  J_param->data[0] = x * th3 / r;
  J_param->data[1] = x * th5 / r;
  J_param->data[2] = x * th7 / r;
  J_param->data[3] = x * th9 / r;

  J_param->data[4] = y * th3 / r;
  J_param->data[5] = y * th5 / r;
  J_param->data[6] = y * th7 / r;
  J_param->data[7] = y * th9 / r;
}
