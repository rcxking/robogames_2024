/*
 * odometry_motion_model.cpp
 *
 * Class implementation for an odometry motion model for planar robots.
 */
#include "robomagellan_localization/odometry_motion_model.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/utils.h>

#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <vector>

using std::placeholders::_1;

/*
 * Computes the shortest angle between the provided input angles.
 *
 * Parameters:
 *   a: First angle (rads)
 *   b: Second angle (rads)
 *
 * Returns:
 *   (double): Shortest angle (rads) between input angles a and b
 */
double AngleDiff(const double a, const double b) {
  // Normalizes input angles to range [-pi, +pi]
  const double norm_a = atan2(sin(a), cos(a));
  const double norm_b = atan2(sin(b), cos(b));

  /*
   * There are 2 ways to reach angle a from angle b.  One is by rotating
   * counterclockwise from angle b and the other is by rotation clockwise from
   * angle b.
   */
  const double d1 = norm_a - norm_b;
  double d2 = 2 * M_PI - fabs(d1);

  /*
   * If d1 is reachable by rotating counterclockwise, d2 must be reachable by
   * rotating clockwise.
   */
  if (d1 > 0) {
    d2 *= -1.0;
  }

  // Pick the angle that needs less rotating to execute
  return (fabs(d1) < fabs(d2)) ? d1 : d2;
}

OdometryMotionModel::OdometryMotionModel(const std::string &name) :
    Node(name),
    nr_samples_(300),
    alpha1_(0.0),
    alpha2_(0.0),
    alpha3_(0.0),
    alpha4_(0.0),
    last_odom_x_(0.0),
    last_odom_y_(0.0),
    last_odom_theta_(0.0),
    is_first_odom_(true) {
  // Configureable parameters
  declare_parameter("alpha1", 0.1);
  declare_parameter("alpha2", 0.1);
  declare_parameter("alpha3", 0.1);
  declare_parameter("alpha4", 0.1);
  declare_parameter("nr_samples", 300);

  alpha1_ = get_parameter("alpha1").as_double();
  alpha2_ = get_parameter("alpha2").as_double();
  alpha3_ = get_parameter("alpha3").as_double();
  alpha4_ = get_parameter("alpha4").as_double();

  nr_samples_ = get_parameter("nr_samples").as_int();

  // Ensure number of particles is valid
  if (nr_samples_ > 0) {
    // Create/initialize particles
    samples_.poses = std::vector<geometry_msgs::msg::Pose>(nr_samples_,
                                                    geometry_msgs::msg::Pose());
  } else {
    // ERROR: Invalid number of particles specified
    RCLCPP_FATAL_STREAM(get_logger(),
        "ERROR: Invalid number of samples requested: " << nr_samples_);
    return;
  }

  // Initialize publishers/subscribers
  pose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "odometry_motion_model/samples", 10);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "robomagellan_controller/odom", 10, std::bind(
        &OdometryMotionModel::OdomCallback, this, _1));
}

void OdometryMotionModel::OdomCallback(const nav_msgs::msg::Odometry &odom) {
  // Convert orientation from quaternion to Euler angles
  const tf2::Quaternion q(odom.pose.pose.orientation.x,
                          odom.pose.pose.orientation.y,
                          odom.pose.pose.orientation.z,
                          odom.pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Is this the first odometry message received?
  if (is_first_odom_) {
    last_odom_x_     = odom.pose.pose.position.x;
    last_odom_y_     = odom.pose.pose.position.y;
    last_odom_theta_ = yaw;
    samples_.header.frame_id = odom.header.frame_id;
    is_first_odom_ = false;
    return;
  }

  // Implement odometry motion model from Probabilistic Robotics, Chapter 5.4
  const double odom_x_increment = odom.pose.pose.position.x - last_odom_x_;
  const double odom_y_increment = odom.pose.pose.position.y - last_odom_y_;
  const double odom_theta_increment = AngleDiff(yaw, last_odom_theta_);

  /*
   * The odometry motion model decomposes the transformation between 2 planar
   * poses into a sequence of a rotation, a translation, and a second rotation.
   * However when going in reverse this translates into a rotation by either
   * +-pi radians (to turn the robot around), translating the robot forward to
   * the new position, and then rotating either +-pi radians to turn the robot
   * forward again.  This causes numerical instabilities when updating the
   * particles' poses with the first rotation and the translation.
   *
   * The solution is to determine whether the robot is moving backwards by
   * checking if the initial rotation is more than +-pi/2.  If the robot is
   * moving backward add pi radians to the first rotation (so it's facing
   * forward) and negate the translation (so it travels backwards).
   */

  double delta_trans = std::sqrt(std::pow(odom_y_increment, 2) +
                                       std::pow(odom_x_increment, 2));
  double delta_rot1 = 0.0;
  if (delta_trans >= std::numeric_limits<double>::epsilon()) {
    delta_rot1 = AngleDiff(atan2(odom_y_increment, odom_x_increment), yaw);

    // Is the robot going in reverse?
    if (fabs(delta_rot1) > (M_PI/2)) {
      // Robot is reversing
      delta_rot1 += M_PI;

      // Normalize delta_rot1 to be in range [-pi, +pi]
      delta_rot1 = atan2(sin(delta_rot1), cos(delta_rot1));

      delta_trans *= -1.0;
    }
  }

  const double delta_rot2 = AngleDiff(odom_theta_increment, delta_rot1);

  // Compute noise affecting each of the 3 components
  const double rot1_variance = (alpha1_ * delta_rot1) + (alpha2_ * delta_trans);
  const double trans_variance = (alpha3_ * delta_trans) +
                                (alpha4_ * (delta_rot1 + delta_rot2));
  const double rot2_variance = (alpha1_ * delta_rot2) + (alpha2_ * delta_trans);

  // Generate random noise to the 3 components
  unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine noise_generator(seed);
  std::normal_distribution<double> rot1_noise(0.0, rot1_variance);
  std::normal_distribution<double> trans_noise(0.0, trans_variance);
  std::normal_distribution<double> rot2_noise(0.0, rot2_variance);

  // Compute noise-free motion components
  for (auto &sample : samples_.poses) {
    const double delta_rot1_draw = AngleDiff(delta_rot1, rot1_noise(noise_generator));
    const double delta_trans_draw = delta_trans - trans_noise(noise_generator);
    const double delta_rot2_draw = AngleDiff(delta_rot2, rot2_noise(noise_generator));

    // Reconstruct noise-free particle pose
    tf2::Quaternion sample_q(sample.orientation.x, sample.orientation.y,
                             sample.orientation.z, sample.orientation.w);
    tf2::Matrix3x3 sample_m(sample_q);

    double sample_roll, sample_pitch, sample_yaw;
    sample_m.getRPY(sample_roll, sample_pitch, sample_yaw);

    // Update particle's position
    sample.position.x += delta_trans_draw * cos(sample_yaw + delta_rot1_draw);
    sample.position.y += delta_trans_draw * sin(sample_yaw + delta_rot1_draw);

    // Update particle's orientation
    tf2::Quaternion updated_orientation;
    const double updated_yaw = sample_yaw + delta_rot1_draw + delta_rot2_draw;
    updated_orientation.setRPY(0.0, 0.0, updated_yaw);

    sample.orientation.x = updated_orientation.getX();
    sample.orientation.y = updated_orientation.getY();
    sample.orientation.z = updated_orientation.getZ();
    sample.orientation.w = updated_orientation.getW();
  }

  // Update last message members
  last_odom_x_ = odom.pose.pose.position.x;
  last_odom_y_ = odom.pose.pose.position.y;
  last_odom_theta_ = yaw;

  // Publish all particles
  pose_array_pub_->publish(samples_);
}

// Create and spawn a node for the odometry motion model
int main(int argc, char *argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create and start the odometry motion model
  auto node = std::make_shared<OdometryMotionModel>("odometry_motion_model");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
