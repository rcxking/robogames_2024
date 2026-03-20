/*
 * robomagellan_interface.cpp
 *
 * Class implementation of the hardware interface to the real robot.
 */
#include "robomagellan_firmware/robomagellan_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace robomagellan_firmware {

// Default constructor is needed but on_init() does all the initialization
RobomagellanInterface::RobomagellanInterface() {
}

RobomagellanInterface::~RobomagellanInterface() {
  // TODO: Close any serial connections here
}

// HW interface initialization really happens here
CallbackReturn RobomagellanInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {
  const CallbackReturn result = hardware_interface::SystemInterface::on_init(params);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  /*
   * Allocate command/states equal to the number of joints (2 joints because
   * there's only 2 motors).
   */
  velocity_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  velocity_states_.reserve(info_.joints.size());

  last_run_ = rclcpp::Clock().now();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobomagellanInterface::export_state_interfaces() {
  // Generate a position state interface per wheel
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION,
          &position_states_[i])
    );

    state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
          &velocity_states_[i])
    );
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobomagellanInterface::export_command_interfaces() {
  // Generate a velocity command interface per wheel
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
          &velocity_commands_[i])
    );
  }
  return command_interfaces;
}

CallbackReturn RobomagellanInterface::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RobomagellanInterface"), "Starting robot hardware interfaces");

  // Reset commands and states
  velocity_commands_ = {0.0, 0.0};
  position_states_   = {0.0, 0.0};
  velocity_states_   = {0.0, 0.0};

  // TODO: Connect to any serial interfaces here
  RCLCPP_INFO(rclcpp::get_logger("RobomagellanInterface"),
      "Hardware started; ready to receive commands");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobomagellanInterface::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RobomagellanInterface"), "Stopping robot hardware");

  // TODO: Close any serial interfaces here
  RCLCPP_INFO(rclcpp::get_logger("RobomagellanInterface"), "Hardware stopped");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobomagellanInterface::read(const rclcpp::Time &,
                                                            const rclcpp::Duration &) {
  // TODO: Parse any serial data/strings here
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobomagellanInterface::write(const rclcpp::Time &,
                                                             const rclcpp::Duration &) {
  // TODO: Send any serial data here
  return hardware_interface::return_type::OK;
}

// Export the plugin
PLUGINLIB_EXPORT_CLASS(robomagellan_firmware::RobomagellanInterface,
                       hardware_interface::SystemInterface)
} // End namespace robomagellan_firmware
