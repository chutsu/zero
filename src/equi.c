#include "zero/equi.h"

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
  const real_t thd_th = 1.0 + 3.0 * k1 * th2 + 5.0 * k2 * th4 + 7.0 * k3 * th6 + 9.0 * k4 * th8;
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
