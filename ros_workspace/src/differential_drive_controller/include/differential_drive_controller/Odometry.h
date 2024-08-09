/*
 * Odometry.h
 *
 * Class definition of the object responsible for keeping track of the robot's
 * odometry.
 */
#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>

namespace differential_drive_controller {
  namespace bacc = boost::accumulators;

  class Odometry {
    public:
      // Integration function for integrating odometry
      typedef boost::function<void(double, double)> IntegrationFunction;

      /*
       * Constructor
       *
       * velocity_rolling_window_size: Rolling window size used to compute
       * velocity mean.
       */
      Odometry(const size_t velocity_rolling_window_size = 10);

      /*
       * Initializes odometry.
       * time: Current time
       */
      void init(const ros::Time &time);

      /*
       * Updates odometry with the latest wheel positions.
       * left_pos: Left wheel position (radians)
       * right_pos: Right wheel position (radians)
       * time: Current time
       *
       * Returns true if odometry is successfully updated; false if no update.
       */
      bool update(const double left_pos, const double right_pos, const ros::Time &time);

      /*
       * Updates odometry with the latest velocity command.
       * linear: Linear velocity (meters/second)
       * angular: Angular velocity (radians/second)
       * time: Current time
       */
      void updateOpenLoop(const double linear, const double angular, const ros::Time &time);

      /*
       * Sets wheel properties.
       * wheel_separation: Distance between left and right wheel midpoints (meters)
       * left_wheel_radius: Left wheel radius (meters)
       * right_wheel_radius: Right wheel radius (meters)
       */
      void setWheelParams(const double wheel_separation, const double left_wheel_radius, const double right_wheel_radius);

      /*
       * Sets velocity rolling window size.
       * velocity_rolling_window_size: Size of the velocity rolling window
       */
      void setVelocityRollingWindowSize(const size_t velocity_rolling_window_size);

      // Accessors
      double getHeading() const { return heading_; }
      double getX() const { return x_; }
      double getY() const { return y_; }
      double getLinear() const { return linear_; }
      double getAngular() const { return angular_; }

    private:
      // Rolling mean accumulator and window
      typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean>> RollingMeanAcc;
      typedef bacc::tag::rolling_window RollingWindow;

      /*
       * Integrates linear/angular velocities using 2nd order Runge-Kutta.
       * linear: Linear velocity computed by encoders (meters/second)
       * angular: Angular velocity computed by encoders (radians/second)
       */
      void integrateRungeKutta2(const double linear, const double angular);

      /*
       * Integrates linear/angular velocities using exact method.
       * linear: Linear velocity computed by encoders (meters/second)
       * angular: Angular velocity computed by encoders (radians/second)
       */
      void integrateExact(const double linear, const double angular);

      // Reset linear/angular velocity accumulators
      void resetAccumulators();

      // Current timestamp
      ros::Time timestamp_;

      // Current pose
      double x_, y_;   // meters
      double heading_; // radians

      // Current velocities
      double linear_;  // meters/second
      double angular_; // radians/second

      // Wheel parameters
      double wheel_separation_;                       // meters
      double left_wheel_radius_, right_wheel_radius_; // meters

      // Previous wheel positions (radians)
      double left_wheel_old_pos_, right_wheel_old_pos_;

      // Rolling mean accumulators for linear/angular velocities
      size_t velocity_rolling_window_size_;
      RollingMeanAcc linear_acc_, angular_acc_;

      // Integration function
      IntegrationFunction integrate_fun_;
  }; // End class Odometry
} // End namespace differential_drive_controller

#endif
