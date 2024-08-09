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
} // End namespace differential_drive_controller
