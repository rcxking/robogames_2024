/*
 * differential_drive_controller.hpp
 *
 * Class declaration of a ROS 2 Controls Plugin representing the differential
 * drive controller for the RoboMagellan robot.
 */
#ifndef __DIFFERENTIAL_DRIVE_CONTROLLER_HPP__
#define __DIFFERENTIAL_DRIVE_CONTROLLER_HPP__

#include "controller_interface/controller_interface.hpp"
#include "differential_drive_controller_parameters.hpp"
#include "differential_drive_controller/odometry.hpp"
#include "differential_drive_controller/speed_limiter.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <chrono>
#include <cmath>
#include <functional> // reference_wrapper
#include <memory> // shared_ptr
#include <queue>
#include <string>
#include <vector>

namespace differential_drive_controller
{
class DifferentialDriveController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  //! Default Constructor
  DifferentialDriveController();

  /**
   * @brief Configures the command interfaces (what we can control).
   */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief Configures the state interfaces (what feedback we received).
   */
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief Main controller update function.
   * @param time: Current ROS time update() was called.
   * @param period: Duration of this update cycle.
   */
  controller_interface::return_type update(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;

  /**
   * @brief Called when the controller is initialized.
   */
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief Called when the controller's parameters are configured.
   */
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief Called when the controller is activated.
   */
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief Called when the controller is deactivated.
   */
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief Called when the controller needs to perform cleanup operations.
   */
  controller_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief Called when an error occurs.
   */
  controller_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief Called when the controller shuts down.
   */
  controller_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State &previous_state) override;

protected:
  /*
   * Each wheel can be controlled by sending an angular velocity (rad/s) and
   * provides feedback via its encoder.
   */
  struct WheelHandle {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
  };

  const char *feedback_type() const;

  /**
   * @brief Creates a wheel for the given side and name.
   */
  controller_interface::CallbackReturn configure_side(
      const std::string &side, const std::vector<std::string> &wheel_names,
      std::vector<WheelHandle> &registered_handles);

  /*
   * Wheel handles for the left/right sides.  Each side's wheels are connected
   * with a belt drive so we only need to track 1 joint per side.
   */
  std::vector<WheelHandle> registered_left_wheel_handles_, registered_right_wheel_handles_;

  // ROS parameters
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  // Tracks robot's odometry
  Odometry odometry_;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};

  // Odometry publishers
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;

  // Odometry TF publishers
  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_ = nullptr;

  // Commanded Velocity subscribers
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    velocity_command_unstamped_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};

  // Last 2 received velocity commands
  std::queue<Twist> previous_commands_;

  // Linear/angular velocity limiters
  SpeedLimiter limiter_linear_, limiter_angular_;

  // Restricted velocity command publishers
  bool publish_limited_velocity_ = false;
  std::shared_ptr<rclcpp::Publisher<Twist>> limited_velocity_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> realtime_limited_velocity_publisher_ =
    nullptr;

  // Previous time the controller was updated
  rclcpp::Time previous_update_timestamp_{0};

  // Publish rate limiter
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

  bool is_halted = false;
  bool use_stamped_vel_ = true;

  bool reset();
  void halt();

}; // End class DifferentialDriveController
} // End namespace differential_drive_controller
#endif
