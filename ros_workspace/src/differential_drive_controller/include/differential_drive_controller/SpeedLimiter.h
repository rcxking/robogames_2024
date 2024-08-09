/*
 * SpeedLimiter.h
 *
 * Class definition of an object that provides limit functions to the
 * velocity/acceleration/jerk.
 */
#ifndef __SPEED_LIMITER_H__
#define __SPEED_LIMITER_H__

namespace differential_drive_controller {

  class SpeedLimiter {
    public:
      /*
       * Constructor
       *
       * has_velocity_limits: Apply velocity limits?
       * has_acceleration_limits: Apply acceleration limits?
       * has_jerk_limits: Apply jerk limits?
       * min_velocity: Minimum velocity (m/s)
       * max_velocity: Maximum velocity (m/s)
       * min_acceleration: Minimum acceleration (m/s^2)
       * max_acceleration: Maximum acceleration (m/s^2)
       * min_jerk: Minimum jerk (m/s^3)
       * max_jerk: Maximum jerk (m/s^3)
       */
      SpeedLimiter(
          const bool has_velocity_limits=false,
          const bool has_acceleration_limits=false,
          const bool has_jerk_limits=false,
          const double min_velocity=0.0,
          const double max_velocity=0.0,
          const double min_acceleration=0.0,
          const double max_acceleration=0.0,
          const double min_jerk=0.0,
          const double max_jerk=0.0);

      /*
       * Limit velocity and acceleration.
       * v: Velocity (m/s)
       * v0: Previous velocity to v (m/s)
       * v1: Previous velocity to v0 (m/s)
       * dt: Time step (s)
       * Returns: Limiting factor (1.0 if none)
       */
      double limit(double &v, const double v0, const double v1, const double dt);

      /*
       * Limits velocity.
       * v: Velocity (m/s)
       * Returns: Limiting factor (1.0 if none)
       */
      double limit_velocity(double &v);

      /*
       * Limits acceleration.
       * v: Velocity (m/s)
       * v0: Previous velocity (m/s)
       * dt: Time step (s)
       * Returns: Limiting factor (1.0 if none)
       */
      double limit_acceleration(double &v, const double v0, const double dt);

      /*
       * Limits jerk.
       * v: Velocity (m/s)
       * v0: Previous velocity to v (m/s)
       * v1: Previous velocity to v0 (m/s)
       * dt: Time step (s)
       * Returns: Limiting factor (1.0 if none)
       */
      double limit_jerk(double &v, const double v0, const double v1, const double dt);

      // Enable/disable limits
      bool has_velocity_limits;
      bool has_acceleration_limits;
      bool has_jerk_limits;

      // Velocity limits (m/s)
      double min_velocity, max_velocity;

      // Acceleration limits (m/s^2)
      double min_acceleration, max_acceleration;

      // Jerk limits (m/s^3)
      double min_jerk, max_jerk;
  }; // End class SpeedLimiter
} // End namespace differential_drive_controller

#endif
