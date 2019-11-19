#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <float.h>
#include <math.h>

#include "core.hpp"
#include "pwm.hpp"

/******************************************************************************
 * PID
 *****************************************************************************/

struct pid_ctrl_t {
  float error_prev;
  float error_sum;

  float error_p;
  float error_i;
  float error_d;

  float k_p;
  float k_i;
  float k_d;
};

void pid_ctrl_setup(pid_ctrl_t *pid,
                    const float k_p,
                    const float k_i,
                    const float k_d);

float pid_ctrl_update(pid_ctrl_t *pid,
                      const float setpoint,
                      const float actual,
                      const float dt);

void pid_ctrl_reset(pid_ctrl_t *pid);

/*****************************************************************************
 * ESC
 *****************************************************************************/

struct esc_t {
  pwm_t m1;
  pwm_t m2;
  pwm_t m3;
  pwm_t m4;
};

void esc_setup(esc_t *esc);

/*****************************************************************************
 * ATTITUDE CONTROLLER
 *****************************************************************************/

struct att_ctrl_t {
  pid_ctrl_t roll;
  pid_ctrl_t pitch;
  pid_ctrl_t yaw;

  float roll_limits[2];
  float pitch_limits[2];
  float max_thrust;

  float outputs[4];
  float dt;
};

void att_ctrl_setup(att_ctrl_t *att_ctrl);

#endif // CONTROL_HPP
