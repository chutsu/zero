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

void pinhole_K(const double fx,
               const double fy,
               const double cx,
               const double cy,
               double K[9]) {
  K[0] = fx;
  K[1] = 0.0;
  K[2] = cx;
  K[3] = 0.0;
  K[4] = fy;
  K[5] = cy;
  K[6] = 0.0;
  K[7] = 0.0;
  K[8] = 1.0;
}

double pinhole_focal_length(const int image_width, const double fov) {
  return ((image_width / 2.0) / tan(deg2rad(fov) / 2.0));
}

int pinhole_project(const double K[9], const double p[3], double x[2]) {
  const double fx = K[0];
  const double fy = K[4];
  const double cx = K[2];
  const double cy = K[5];

  const double px = p[0] / p[2];
  const double py = p[1] / p[2];

  x[0] = px * fx + cx;
  x[1] = py * fy + cy;

  return 0;
}

void pinhole_calc_K(const double image_width,
                    const double image_height,
                    const double lens_hfov,
                    const double lens_vfov,
                    double K[9]) {
  const double fx = pinhole_focal_length(image_width, lens_hfov);
  const double fy = pinhole_focal_length(image_height, lens_vfov);
  const double cx = image_width / 2.0;
  const double cy = image_height / 2.0;
  return pinhole_K(fx, fy, cx, cy, K);
}

/*****************************************************************************
 * RADTAN
 *****************************************************************************/

void radtan4_distort(const double k1,
                     const double k2,
                     const double p1,
                     const double p2,
                     const double p[2],
                     double p_d[2]) {
  /* Point */
  const double x = p[0];
  const double y = p[1];

  /* Apply radial distortion */
  const double x2 = x * x;
  const double y2 = y * y;
  const double r2 = x2 + y2;
  const double r4 = r2 * r2;
  const double radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
  const double x_dash = x * radial_factor;
  const double y_dash = y * radial_factor;

  /* Apply tangential distortion */
  const double xy = x * y;
  const double x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
  const double y_ddash = y_dash + (p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy);

  /* Distorted point */
  p_d[0] = x_ddash;
  p_d[1] = y_ddash;
}

void radtan4_point_jacobian(const double k1,
                            const double k2,
                            const double p1,
                            const double p2,
                            const double p[2],
                            double J_point[2 * 2]) {
  /* Point */
  const double x = p[0];
  const double y = p[1];

  /* Apply radial distortion */
  const double x2 = x * x;
  const double y2 = y * y;
  const double r2 = x2 + y2;
  const double r4 = r2 * r2;

  /* Point Jacobian is 2x2 */
  /* clang-format off */
  J_point[0] = k1 * r2 + k2 * r4 + 2 * p1 * y + 6 * p2 * x +
                x * (2 * k1 * x + 4 * k2 * x * r2) + 1;
  J_point[1] = 2 * p1 * x + 2 * p2 * y + y * (2 * k1 * x + 4 * k2 * x * r2);
  J_point[2] = J_point[1];
  J_point[3] = k1 * r2 + k2 * r4 + 6 * p1 * y + 2 * p2 * x +
               y * (2 * k1 * y + 4 * k2 * y * r2) + 1;
  /* clang-format on */
}

void radtan4_param_jacobian(const double k1,
                            const double k2,
                            const double p1,
                            const double p2,
                            const double p[2],
                            double J_param[2 * 4]) {
  /* Point */
  const double x = p[0];
  const double y = p[1];

  /* Setup */
  const double x2 = x * x;
  const double y2 = y * y;
  const double xy = x * y;
  const double r2 = x2 + y2;
  const double r4 = r2 * r2;

  /* Param Jacobian is 2x4 */
  J_param[0] = x * r2;
  J_param[1] = x * r4;
  J_param[2] = 2 * xy;
  J_param[3] = 3 * x2 + y2;

  J_param[4] = y * r2;
  J_param[5] = y * r4;
  J_param[6] = x2 + 3 * y2;
  J_param[7] = 2 * xy;
}

/*****************************************************************************
 * EQUI
 *****************************************************************************/

void equi4_distort(const double k1,
                   const double k2,
                   const double k3,
                   const double k4,
                   const double p[2],
                   double p_d[2]) {
  const double x = p[0];
  const double y = p[1];
  const double r = sqrt(x * x + y * y);

  const double th = atan(r);
  const double th2 = th * th;
  const double th4 = th2 * th2;
  const double th6 = th4 * th2;
  const double th8 = th4 * th4;
  const double thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const double s = thd / r;

  const double x_dash = s * x;
  const double y_dash = s * y;

  p_d[0] = x_dash;
  p_d[1] = y_dash;
}

void equi4_point_jacobian(const double k1,
                          const double k2,
                          const double k3,
                          const double k4,
                          const double p[2],
                          double J_point[2 * 2]) {
  const double x = p[0];
  const double y = p[1];
  const double r = sqrt(x * x + y * y);

  const double th = atan(r);
  const double th2 = th * th;
  const double th4 = th2 * th2;
  const double th6 = th4 * th2;
  const double th8 = th4 * th4;
  const double thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);

  const double th_r = 1.0 / (r * r + 1.0);
  const double thd_th =
      1.0 + 3.0 * k1 * th2 + 5.0 * k2 * th4 + 7.0 * k3 * th6 + 9.0 * k4 * th8;
  const double s = thd / r;
  const double s_r = thd_th * th_r / r - thd / (r * r);
  const double r_x = 1.0 / r * x;
  const double r_y = 1.0 / r * y;

  /* Point Jacobian is 2x2 */
  J_point[0] = s + x * s_r * r_x;
  J_point[1] = x * s_r * r_y;
  J_point[2] = y * s_r * r_x;
  J_point[3] = s + y * s_r * r_y;
}

void equi4_param_jacobian(const double k1,
                          const double k2,
                          const double k3,
                          const double k4,
                          const double p[2],
                          double J_param[2 * 4]) {
  const double x = p[0];
  const double y = p[1];
  const double r = sqrt(x * x + y * y);

  const double th = atan(r);
  const double th2 = th * th;
  const double th3 = th2 * th;
  const double th5 = th3 * th2;
  const double th7 = th5 * th2;
  const double th9 = th7 * th2;

  /* Param Jacobian is 2x4 */
  J_param[0] = x * th3 / r;
  J_param[1] = x * th5 / r;
  J_param[2] = x * th7 / r;
  J_param[3] = x * th9 / r;

  J_param[4] = y * th3 / r;
  J_param[5] = y * th5 / r;
  J_param[6] = y * th7 / r;
  J_param[7] = y * th9 / r;
}
