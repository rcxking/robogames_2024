/*
 * odometry_motion_model.hpp
 *
 * Class definition for an implementation of the odometry motion model for 2D
 * planar robots.
 */
#ifndef __ODOMETRY_MOTION_MODEL_HPP__
#define __ODOMETRY_MOTION_MODEL_HPP__

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

class OdometryMotionModel : public rclcpp::Node {
public:
  /*
   * Constructor
   *
   * Parameters:
   *   name: Name of this node
   */
  OdometryMotionModel(const std::string &name);

private:
  // Subscriber to robot's odometry
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publisher a particle cloud to represent the robot's pose estimate
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;

  // Number of particles to use
  int nr_samples_;

  // Stores particles representing robot's pose estimate
  geometry_msgs::msg::PoseArray samples_;

  // Parameters governing how the noise affects the motion model
  double alpha1_, alpha2_, alpha3_, alpha4_;

  // Previous odometry pose (position in meters; theta in radians)
  double last_odom_x_, last_odom_y_, last_odom_theta_;

  // Is this the first odometry message received?
  bool is_first_odom_;

  /*
   * Callback to process new odometry messages.
   *
   * Parameters:
   *   odom: New odometry message
   */
  void OdomCallback(const nav_msgs::msg::Odometry &odom);
}; // End class OdometryMotionModel

#endif
