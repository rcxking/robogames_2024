/*
 * differential_drive_controller.hpp
 *
 * Class declaration of a ROS 2 Controls Plugin representing the differential
 * drive controller for the RoboMagellan robot.
 */
#ifndef __DIFFERENTIAL_DRIVE_CONTROLLER_HPP__
#define __DIFFERENTIAL_DRIVE_CONTROLLER_HPP__

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <functional> // reference_wrapper
#include <memory> // shared_ptr
#include <string>

namespace differential_drive_controller
{
class DifferentialDriveController : public controller_interface::ControllerInterface
{
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
      const std::string &side, const std::string &wheel_name,
      WheelHandle &registered_handle);

  /*
   * Wheel handles for the left/right sides.  Each side's wheels are connected
   * with a belt drive so we only need to track 1 joint per side.
   */
  WheelHandle registered_left_wheel_handle_, registered_right_wheel_handle_;

  // ROS parameters
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
private:

}; // End class DifferentialDriveController
} // End namespace differential_drive_controller
#endif
