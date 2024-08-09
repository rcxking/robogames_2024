/*
 * Odometry.cpp
 *
 * Class implementation of the Odometry module.
 */
#include <differential_drive_controller/Odometry.h>

namespace differential_drive_controller {
  namespace bacc = boost::accumulators;

  Odometry::Odometry(const size_t velocity_rolling_window_size)
    : timestamp_(0.0),
      x_(0.0),
      y_(0.0),
      heading_(0.0),
      linear_(0.0),
      angular_(0.0),
      wheel_separation_(0.0),
      left_wheel_radius_(0.0),
      right_wheel_radius_(0.0),
      left_wheel_old_pos_(0.0),
      right_wheel_old_pos_(0.0),
      velocity_rolling_window_size_(velocity_rolling_window_size),
      linear_acc_(RollingWindow::window_size = velocity_rolling_window_size),
      angular_acc_(RollingWindow::window_size = velocity_rolling_window_size),
      integrate_fun_(std::bind(&Odometry::integrateExact, this,
            std::placeholders::_1, std::placeholders::_2)) {}

  void Odometry::init(const ros::Time &time) {
    // Reset accumulators and current timestamp
    resetAccumulators();
    timestamp_ = time;
  }

  bool Odometry::update(const double left_pos, const double right_pos,
                        const ros::Time &time) {
    // Get current wheel joint positions
    const double left_wheel_cur_pos = left_pos * left_wheel_radius_;
    const double right_wheel_cur_pos = right_pos * right_wheel_radius_;

    // Estimate velocity of wheels using current and old positions
    const double left_wheel_est_vel = left_wheel_cur_pos - left_wheel_old_pos_;
    const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;

    // Update old position with current position
    left_wheel_old_pos_ = left_wheel_cur_pos;
    right_wheel_old_pos_ = right_wheel_cur_pos;

    // Compute linear/angular velocity differences
    const double linear = (right_wheel_est_vel + left_wheel_est_vel) * 0.5;
    const double angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation_;

    // Integrate odometry
    integrate_fun_(linear, angular);

    // Cannot estimate speed with extremely small time intervals
    const double dt = (time - timestamp_).toSec();
    if (dt < 0.0001) {
      return false;
    }

    timestamp_ = time;

    // Estimate speeds using a rolling mean filter
    linear_acc_(linear/dt);
    angular_acc_(angular/dt);

    linear_ = bacc::rolling_mean(linear_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

    return true;
  }

  void Odometry::updateOpenLoop(const double linear, const double angular, const ros::Time &time) {
    // Save last linear/angular velocities
    linear_ = linear;
    angular_ = angular;

    // Integrate odometry
    const double dt = (time - timestamp_).toSec();
    timestamp_ = time;
    integrate_fun_(linear * dt, angular * dt);
  }
} // End namespace differential_drive_controller
