#ifndef __PID_H__
#define __PID_H__

/*
 * PID.h
 *
 * Class definition to implement PID motor velocity control for a single motor.
 */
#include "Arduino.h"

class PID {
public:
  // Constructor
  PID(const double kp, const double ki, const double kd) :
    kp_(kp), ki_(ki), kd_(kd) {}

  /*
   * Given the current and desired motor velocities compute and send the next
   * PWM motor command to the motor controller.
   *
   * Parameters:
   *   cur_vel (double): Current wheel velocity (rad/s)
   *   des_vel (double): Desired wheel velocity (rad/s)
   */
  void SendPWMCommand(const double cur_vel, const double des_vel);

private:
  // Proportional, integral, derivative constants
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;

  // Previous error
  double prev_error_ = 0.0;

  // Accumulated error for integral term
  double accumulated_error_ = 0.0;

  /*
   * Current motor controller PWM signal.  Needs to be in the range [1000,
   * 2000].  1500 is STOP; 2000 is full forward; and 1000 is full reverse.
   */
  int cur_pwm_cmd_ = 1500;

  /*
   * Given the current and desired motor velocities compute the next PWM command
   * to send to the motor controllers.
   */
  int ComputePWMCommand(const double cur_vel, const double des_vel) const;
};

#endif
