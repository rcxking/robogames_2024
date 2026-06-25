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
   */
  PID(const double kp, const double ki, const double kd) :
    des_vel_(0.0), kp_(kp), ki_(ki), kd_(kd) {
  }

  /*
   * Initializes the underlying Servo object to the specified motor pin.
   *
   * Parameters:
   *   motor_pin (int): Pin to attach to
   */
  void ConnectToMotor(const int motor_pin) {
    Serial.print("Attaching Servo controller_ to pin: ");
    Serial.println(motor_pin);

    // Connect to the specified motor controller
    controller_.attach(motor_pin);

    // For safety send the STOP command to the controller
    controller_.writeMicroseconds(STOP_PWM);
  }

  /*
   * Given the current compute and send the next PWM motor command to the motor
   * controller.  Desired velocity used is des_vel_.
   *
   * Parameters:
   *   cur_vel (double): Current wheel velocity (rad/s)
   */
  void SendPWMCommand(const double cur_vel);

  // Accessor/modifier for desired velocity (rad/s)
  void SetDesiredVelocity(const double des_vel) { des_vel_ = des_vel; }
  double GetDesiredVelocity() const { return des_vel_; }

  // Accessors/modifiers for PID constants.  Used when tuning PID constants
  void SetKP(const double kp) { kp_ = kp; }
  void SetKI(const double ki) { ki_ = ki; }
  void SetKD(const double kd) { kd_ = kd; }

  double GetKP() const { return kp_; }
  double GetKI() const { return ki_; }
  double GetKD() const { return kd_; }

  // Displays the current PID constants' values
  String DisplayPIDConstants() const {
    return "KP: " + String(kp_, 9) + "; KI: " + String(ki_, 9) + "; KD: "
      + String(kd_, 9);
  }

private:
  // Desired velocity (rad/s)
  double des_vel_ = 0.0;

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
  static constexpr int STOP_PWM = 1500;
  int cur_pwm_cmd_ = STOP_PWM;

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
