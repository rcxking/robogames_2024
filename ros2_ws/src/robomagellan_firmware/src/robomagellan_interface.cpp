/*
 * robomagellan_interface.cpp
 *
 * Class implementation of the hardware interface to the real robot.
 */
#include "robomagellan_firmware/robomagellan_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <sstream>
#include <string>

namespace robomagellan_firmware {

// Default constructor is needed but on_init() does all the initialization
RobomagellanInterface::RobomagellanInterface() {
}

RobomagellanInterface::~RobomagellanInterface() {
  // Close Arduino serial connection
  if (arduino_.IsOpen()) {
    try {
      arduino_.Close();
    } catch (...) {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobomagellanInterface"),
          "ERROR in closing connection to port: " << port_);
    }
  }
}

// HW interface initialization really happens here
CallbackReturn RobomagellanInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {
  const CallbackReturn result = hardware_interface::SystemInterface::on_init(params);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  // Attempt to get the Arduino serial port name
  try {
    port_ = info_.hardware_parameters.at("port");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("RobomagellanInterface"), "Using Arduino serial port: " << port_);
  } catch (const std::out_of_range &e) {
    RCLCPP_FATAL(rclcpp::get_logger("RobomagellanInterface"), "ERROR: No serial port provided");
    return CallbackReturn::FAILURE;
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

  // Connect to Arduino
  RCLCPP_INFO_STREAM(rclcpp::get_logger("RobomagellanInterface"), "Attempting to connect to Arduino through port: " << port_);

  try {
    arduino_.Open(port_);

    // Arduino Mega maximum baud rate is 115200
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

    RCLCPP_INFO(rclcpp::get_logger("RobomagellanInterface"), "Successfully connected to Arduino");
  } catch (...) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobomagellanInterface"),
        "ERROR: Failed to connect to Arduino through port: " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("RobomagellanInterface"),
      "Hardware started; ready to receive commands");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobomagellanInterface::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RobomagellanInterface"), "Stopping robot hardware");

  // Close Arduino connection
  if (arduino_.IsOpen()) {
    RCLCPP_INFO(rclcpp::get_logger("RobomagellanInterface"), "Shutting down Arduino connection");

    try {
      arduino_.Close();
      RCLCPP_INFO(rclcpp::get_logger("RobomagellanInterface"), "Successfully shut down Arduino connection");
    } catch (...) {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobomagellanInterface"), "ERROR: Failed to shut down Arduino connection on port: " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RobomagellanInterface"), "Hardware stopped");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobomagellanInterface::read(const rclcpp::Time &,
                                                            const rclcpp::Duration &) {
  // Read the wheel velocities from the Arduino
  if (arduino_.IsDataAvailable()) {
    /*
     * Time delta between now and the previous time read() was called.  Need
     * this to integrate the wheel velocities to get wheel position.
     */
    const double dt = (rclcpp::Clock().now() - last_run_).seconds();

    /*
     * Arduino sends string of the format:
     * "L<left wheel vel>;R<right wheel vel>;"
     */
    std::string next_vels;
    arduino_.ReadLine(next_vels);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("RobomagellanInterface"),
        "Received next_vels: " << next_vels);

    std::string token;
    std::stringstream ss(next_vels);
    while (std::getline(ss, token, ';')) {
      // First character is L (left) or R (right) motor.  Ignore everything else
      const char first_char = token[0];

      if (first_char == 'L') {
        velocity_states_.at(0) = std::stod(token.substr(1, token.size()));
        position_states_.at(0) += velocity_states_.at(0) * dt;
      } else if (first_char == 'R') {
        velocity_states_.at(1) = std::stod(token.substr(1, token.size()));
        position_states_.at(1) += velocity_states_.at(1) * dt;
      } else {
        // ERROR: Unrecognized character
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RobomagellanInterface"),
            "ERROR: Unrecognized first_char: " << first_char);
      }
    }

    // Update last time read() was run
    last_run_ = rclcpp::Clock().now();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobomagellanInterface::write(const rclcpp::Time &,
                                                             const rclcpp::Duration &) {
  // TODO: Send any serial data here
  RCLCPP_INFO_STREAM(rclcpp::get_logger("RobomagellanInterface"),
      "Writing velocities: " << velocity_commands_[0] << "; " << velocity_commands_[1]);
  return hardware_interface::return_type::OK;
}

// Export the plugin
PLUGINLIB_EXPORT_CLASS(robomagellan_firmware::RobomagellanInterface,
                       hardware_interface::SystemInterface)
} // End namespace robomagellan_firmware
