#include "control.hpp"

/******************************************************************************
 * PID
 *****************************************************************************/

void pid_ctrl_setup(pid_ctrl_t *pid,
							 		  const float k_p,
							 		  const float k_i,
							 		  const float k_d) {
  pid->error_prev = 0.0;
  pid->error_sum = 0.0;

  pid->error_p = 0.0;
  pid->error_i = 0.0;
  pid->error_d = 0.0;

  pid->k_p = k_p;
  pid->k_i = k_i;
  pid->k_d = k_d;
}

float pid_ctrl_update(pid_ctrl_t *pid,
								  		const float setpoint,
								  		const float actual,
								  		const float dt) {
  // Calculate errors
  const double error = setpoint - actual;
  pid->error_sum += error * dt;

  // Calculate output
  pid->error_p = pid->k_p * error;
  pid->error_i = pid->k_i * pid->error_sum;
  pid->error_d = pid->k_d * (error - pid->error_prev) / dt;
  const double output = pid->error_p + pid->error_i + pid->error_d;

  pid->error_prev = error;
  return output;
}

void pid_ctrl_reset(pid_ctrl_t *pid) {
  pid->error_prev = 0.0;
  pid->error_sum = 0.0;

  pid->error_p = 0.0;
  pid->error_i = 0.0;
  pid->error_d = 0.0;
}

/*****************************************************************************
 * ESC
 *****************************************************************************/

void esc_setup(esc_t *esc, uint8_t pins[4]) {
	const float freq = 1000.0f;
	pwm_setup(&esc->m1, pins[0], freq);
	pwm_setup(&esc->m2, pins[1], freq);
	pwm_setup(&esc->m3, pins[2], freq);
	pwm_setup(&esc->m4, pins[3], freq);
}

/*****************************************************************************
 * ATTITUDE CONTROLLER
 *****************************************************************************/

void att_ctrl_setup(att_ctrl_t *ctrl) {
	pid_ctrl_setup(&ctrl->roll, 0.0f, 0.0f, 0.0f);
	pid_ctrl_setup(&ctrl->pitch, 0.0f, 0.0f, 0.0f);
	pid_ctrl_setup(&ctrl->yaw, 0.0f, 0.0f, 0.0f);

	ctrl->roll_limits[0] = 0.0f;
	ctrl->roll_limits[1] = 0.0f;
	ctrl->pitch_limits[0] = 0.0f;
	ctrl->pitch_limits[1] = 0.0f;
	ctrl->max_thrust = 0.0f;

	ctrl->outputs[0] = 0.0f;
	ctrl->outputs[1] = 0.0f;
	ctrl->outputs[2] = 0.0f;
	ctrl->outputs[3] = 0.0f;
	ctrl->dt = 0.0f;
}

void att_ctrl_update(att_ctrl_t *ctrl,
                     const float setpoints[4],
                     const float T_WB[16],
                     const float dt) {
  // Check rate
  ctrl->dt += dt;
  if (ctrl->dt < 0.001f) {
    return;
  }

  // Form actual
  float r_WB[3] = {0};
	tf_trans(T_WB, r_WB);
  const float rpy[3] = quat2euler(tf_quat(T_WB));
  const float actual[4] = {rpy[0], rpy[1], rpy[2], z};

  // Calculate yaw error
  float actual_yaw = rad2deg(actual[2]);
  float setpoint_yaw = rad2deg(setpoints[2]);
  float error_yaw = setpoint_yaw - actual_yaw;

  // Wrap yaw
  if (error_yaw > 180.0) {
    error_yaw -= 360.0;
  } else if (error_yaw < -180.0) {
    error_yaw += 360.0;
  }
  error_yaw = deg2rad(error_yaw);

  // Roll, pitch and yaw
  float r = pid_ctrl_update(&ctrl->roll, setpoints[0], actual[0], ctrl->dt);
  float p = pid_ctrl_update(&ctrl->pitch, setpoints[1], actual[1], ctrl->dt);
  float y = pid_ctrl_update(&ctrl->yaw, error_yaw, 0.0, ctrl->dt);
  r = (r < ctrl->roll_limits[0]) ? ctrl->roll_limits[0] : r;
  r = (r > ctrl->roll_limits[1]) ? ctrl->roll_limits[1] : r;
  p = (p < ctrl->pitch_limits[0]) ? ctrl->pitch_limits[0] : p;
  p = (p > ctrl->pitch_limits[1]) ? ctrl->pitch_limits[1] : p;

  // Thrust
  float t = ctrl->max_thrust * setpoints[3];
  t = (t > ctrl->max_thrust) ? ctrl->max_thrust : t;
  t = (t < 0.0) ? 0.0 : t;

  // Map roll, pitch, yaw and thrust to motor outputs
  const float m1 = -p - y + t;
  const float m2 = -r + y + t;
  const float m3 = p - y + t;
  const float m4 = r + y + t;
  float outputs[4] = {m1, m2, m3, m4};

  // Limit outputs
	ctrl->outputs[0] = (m1 > ctrl->max_thrust) ? ctrl->max_thrust : m1;
	ctrl->outputs[0] = (m1 < 0.0) ? 0.0 : m1;
	ctrl->outputs[1] = (m2 > ctrl->max_thrust) ? ctrl->max_thrust : m2;
	ctrl->outputs[1] = (m2 < 0.0) ? 0.0 : m2;
	ctrl->outputs[2] = (m3 > ctrl->max_thrust) ? ctrl->max_thrust : m3;
	ctrl->outputs[2] = (m3 < 0.0) ? 0.0 : m3;
	ctrl->outputs[3] = (m4 > ctrl->max_thrust) ? ctrl->max_thrust : m4;
	ctrl->outputs[3] = (m4 < 0.0) ? 0.0 : m4;

  ctrl->dt = 0.0;
}
