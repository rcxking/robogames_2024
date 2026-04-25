/*
 * PID.cpp
 *
 * Class implementation to implement PID motor velocity control for a single
 * motor.
 */
#include "PID.h"

void PID::SendPWMCommand(const double cur_vel, const double des_vel) {
  /*
   * Compute the error between the desired and current velocities (des - cur).
   * If the difference is positive, the current velocity is less than the
   * desired velocity and the motors needs to speed up.  If the difference is
   * negative, the current velocity is greater than the desired velocity and the
   * motor needs to slow down.  At a difference of 0 no additional changes need
   * to be made.
   */
  const double error = des_vel - cur_vel;

  // Add to accumulated errors integral term
  accumulated_error_ += (ki_ * error);

  // Compute derivative term
  const double deriv = error - prev_error_;

  // Apply proportional constants to determine the change in velocity (rad/s)
  const double delta_vel_rad_per_sec = (kp_ * error) + (accumulated_error_) +
                                       (kd_ * deriv);

  // Update last error
  prev_error_ = error;

  // Apply current PWM command
  cur_pwm_cmd_ += delta_vel_rad_per_sec;

  // Threshold curren PWM command to be in range [1000, 2000]
  cur_pwm_cmd_ = max(1000, cur_pwm_cmd_);
  cur_pwm_cmd_ = min(2000, cur_pwm_cmd_);
}
