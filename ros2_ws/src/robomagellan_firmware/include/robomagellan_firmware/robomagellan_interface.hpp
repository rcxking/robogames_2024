/*
 * robomagellan_interface.hpp
 *
 * Class definition of the hardware interface to the real robot.
 */
#ifndef __ROBOMAGELLAN_INTERFACE_HPP__
#define __ROBOMAGELLAN_INTERFACE_HPP__

#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <string>
#include <vector>

namespace robomagellan_firmware {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// System interfaces inherit from the lifecycle node interfaces
class RobomagellanInterface : public hardware_interface::SystemInterface {
public:
  // Constructor
  RobomagellanInterface();

  // Destructor
  virtual ~RobomagellanInterface();

  /*
   * Hardware interfaces must implement on_activate()/on_deactivate() from
   * rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  /*
   * Hardware interfaces must implement these functions from
   * hardware_interface::SystemInterface.
   */
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

  /*
   * Used to create state interfaces.  Each wheel will need a position and
   * velocity state interface.
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /*
   * Used to create command interfaces.  Each wheel will have a single velocity
   * interface.
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // For reading from the underlying hardware
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  // For writing to the underlying hardware
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // Serial connection to the Arduino
  LibSerial::SerialPort arduino_;

  // Arduino port name
  std::string port_;

  // Velocity commands sent to the motors
  std::vector<double> velocity_commands_;

  // Wheel positions
  std::vector<double> position_states_;

  // Wheel velocities
  std::vector<double> velocity_states_;

  // The last time this interface was run
  rclcpp::Time last_run_;
}; // End class RobomagellanInterface

} // End namespace robomagellan_firmware

#endif
