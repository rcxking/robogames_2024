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
   * Given the current and desired motor velocities compute the next PWM command
   * to send to the motor controllers.
   */
  int ComputePWMCommand(const double cur_vel, const double des_vel) const;

private:
  // Proportional, integral, derivative constants
  double kp_, ki_, kd_;
};

#endif
