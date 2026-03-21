/*
 * robomagellan_interface.hpp
 *
 * Class definition of the hardware interface to the real robot.
 */
#ifndef __ROBOMAGELLAN_INTERFACE_HPP__
#define __ROBOMAGELLAN_INTERFACE_HPP__

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

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
  CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() overide;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
}; // End class RobomagellanInterface

} // End namespace robomagellan_firmware

#endif
