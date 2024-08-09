/*
 * SpeedLimiter.cpp
 *
 * Class implementation of an object that provides limit functions to the
 * velocity/acceleration/jerk.
 */
#include <algorithm>
#include <differential_drive_controller/SpeedLimiter.h>

/*
 * Helper function to apply min/max parameters to the specified input.
 * x: Initial value to limit
 * min, max: x will be thresholded to be within range [min, max]
 * Returns: x in [min, max]
 */
template<typename T> T clamp(T x, T min, T max) {
  return std::min(std::max(min, x), max);
}

namespace differential_drive_controller {
  SpeedLimiter::SpeedLimiter(
      const bool has_velocity_limits,
      const bool has_acceleration_limits,
      const bool has_jerk_limits,
      const double min_velocity,
      const double max_velocity,
      const double min_acceleration,
      const double max_acceleration,
      const double min_jerk,
      const double max_jerk)
    : has_velocity_limits(has_velocity_limits),
      has_acceleration_limits(has_acceleration_limits),
      has_jerk_limits(has_jerk_limits),
      min_velocity(min_velocity),
      max_velocity(max_velocity),
      min_acceleration(min_acceleration),
      max_acceleration(max_acceleration),
      min_jerk(min_jerk),
      max_jerk(max_jerk) {}

  double SpeedLimiter::limit(double &v, const double v0, const double v1, const double dt) {
    const double tmp = v;

    limit_jerk(v, v0, v1, dt);
    limit_acceleration(v, v0, dt);
    limit_velocity(v);

    return tmp != 0.0 ? v / tmp : 1.0;
  }
} // End namespace differential_drive_controller
