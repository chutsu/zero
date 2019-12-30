#ifndef CV_H
#define CV_H

#include <stdint.h>

#include "zero/core.h"

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

void pinhole_K(const double fx,
               const double fy,
               const double cx,
               const double cy,
               double K[3 * 3]);
double pinhole_focal_length(const int image_width, const double fov);
int pinhole_project(const double K[3 * 3], const double p[3], double x[2]);
void pinhole_calc_K(const double image_width,
                    const double image_height,
                    const double lens_hfov,
                    const double lens_vfov,
                    double K[3 * 3]);

/*****************************************************************************
 * RADTAN
 *****************************************************************************/

void radtan4_distort(const double k1,
                     const double k2,
                     const double p1,
                     const double p2,
                     const double p[2],
                     double p_d[2]);

void radtan4_point_jacobian(const double k1,
                            const double k2,
                            const double p1,
                            const double p2,
                            const double p[2],
                            double J_point[2 * 2]);

void radtan4_param_jacobian(const double k1,
                            const double k2,
                            const double p1,
                            const double p2,
                            const double p[2],
                            double J_param[2 * 4]);

/*****************************************************************************
 * EQUI
 *****************************************************************************/

void equi4_distort(const double k1,
                   const double k2,
                   const double k3,
                   const double k4,
                   const double p[2],
                   double p_d[2]);

void equi4_point_jacobian(const double k1,
                          const double k2,
                          const double k3,
                          const double k4,
                          const double p[2],
                          double J_point[2 * 2]);

void equi4_param_jacobian(const double k1,
                          const double k2,
                          const double k3,
                          const double k4,
                          const double p[2],
                          double J_param[2 * 4]);

#endif // CV_H
