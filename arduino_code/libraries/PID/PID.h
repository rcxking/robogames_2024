#ifndef __PID_H__
#define __PID_H__

/*
 * PID.h
 *
 * Class definition to implement PID motor velocity control for a single motor.
 */
#include "Arduino.h"

#include <Servo.h>

class PID {
public:
  /*
   * Constructor
   *
   * Parameters:
   *   kp (double): Proportional constant
   *   ki (double): Integral constant
   *   kd (double): Derivative constant
   *   motor_pin (int): Arduino pin connected to the motor controller
   */
  PID(const double kp, const double ki, const double kd, const int motor_pin) :
    kp_(kp), ki_(ki), kd_(kd) {
    // Connect to the specified motor controller
    controller_.attach(motor_pin);
  }

  /*
   * Given the current and desired motor velocities compute and send the next
   * PWM motor command to the motor controller.
   *
   * Parameters:
   *   cur_vel (double): Current wheel velocity (rad/s)
   *   des_vel (double): Desired wheel velocity (rad/s)
   */
  void SendPWMCommand(const double cur_vel, const double des_vel);

  // Accessors/modifiers for PID constants.  Used when tuning PID constants
  void SetKP(const double kp) { kp_ = kp; }
  void SetKI(const double ki) { ki_ = ki; }
  void SetKD(const double kd) { kd_ = kd; }

  double GetKP() const { return kp_; }
  double GetKI() const { return ki_; }
  double GetKD() const { return kd_; }

  // Displays the current PID constants' values
  String DisplayPIDConstants() const {
    return String("KP: " + kp_ + "; KI: " + ki_ + "; KD: " + kd_);
  }

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
   * The Talon SRX motor controller uses PWM signals; create a Servo object to
   * write PWM signals to the controller.
   */
  Servo controller_;

  /*
   * Given the current and desired motor velocities compute the next PWM command
   * to send to the motor controllers.
   */
  int ComputePWMCommand(const double cur_vel, const double des_vel) const;
};

#endif
