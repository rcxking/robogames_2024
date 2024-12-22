/*
 * differential_drive_controller.hpp
 *
 * Class declaration of a ROS 2 Controls Plugin representing the differential
 * drive controller for the RoboMagellan robot.
 */
#ifndef __DIFFERENTIAL_DRIVE_CONTROLLER_HPP__
#define __DIFFERENTIAL_DRIVE_CONTROLLER_HPP__

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

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

private:

}; // End class DifferentialDriveController
} // End namespace differential_drive_controller
#endif
