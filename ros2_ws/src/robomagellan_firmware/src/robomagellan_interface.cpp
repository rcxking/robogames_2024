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
     * Arduino sends string of the format:
     * "L<left wheel pos (rad)>,<left wheel vel (rad/s)>;R<right wheel pos
     * (rad)>,<right wheel vel (rad/s)>;"
     */
    std::string next_info;
    arduino_.ReadLine(next_info);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RobomagellanInterface"),
        "Received next_info: " << next_info);

    std::string next_motor_info;
    std::stringstream ss(next_info);
    while (std::getline(ss, next_motor_info, ';')) {
      // Ensure next motor information isn't blank
      if (next_motor_info.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("RobomagellanInterface"),
            "ERROR: next_motor_info is blank!");
        continue;
      }

      // First character is L (left) or R (right) motor.  Ignore everything else
      const char first_char = next_motor_info[0];

      const bool first_char_l = (first_char == 'L');
      const bool first_char_r = (first_char == 'R');

      if (first_char_l || first_char_r) {
        // Extract motor position and velocity
        const std::string motor_info(next_motor_info.substr(1));
        std::string next_pos_rads, next_vel_rads_per_sec;
        std::stringstream ss2(motor_info);
        std::getline(ss2, next_pos_rads, ',');
        std::getline(ss2, next_vel_rads_per_sec, ',');

        // Ensure valid position/velocity
        if (next_pos_rads.empty() || next_vel_rads_per_sec.empty()) {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("RobomagellanInterface"),
              "ERROR: Invalid position and/or velocity! next_pos_rads: " <<
              next_pos_rads << "; next_vel_rads_per_sec: " <<
              next_vel_rads_per_sec);
          continue;
        }

        // Update position and velocity
        const size_t idx = (first_char_l) ? 0 : 1;
        position_states_.at(idx) = std::stod(next_pos_rads);
        velocity_states_.at(idx) = std::stod(next_vel_rads_per_sec);

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RobomagellanInterface"),
            "idx: " << idx << "; next_pos_rads: " << next_pos_rads <<
            "; next_vel_rads_per_sec: " << next_vel_rads_per_sec);
      } // End if (first_char_l || first_char_r)
    } // End while (std::getline(ss, next_motor_info, ';'))
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobomagellanInterface::write(const rclcpp::Time &,
                                                             const rclcpp::Duration &) {
  // Check if either desired velocity is different than the last sent velocities
  const bool left_vel_same = DoubleEquals(velocity_commands_[0],
                                          last_left_cmd_);
  const bool right_vel_same = DoubleEquals(velocity_commands_[1],
                                           last_right_cmd_);

  // Don't send the send commands repeatedly
  if (left_vel_same && right_vel_same) {
    return hardware_interface::return_type::OK;
  }

  // Write desired motor velocities to the Arduino
  RCLCPP_INFO_STREAM(rclcpp::get_logger("RobomagellanInterface"),
      "Writing velocities: " << velocity_commands_[0] << "; " << velocity_commands_[1]);

  /*
   * The motors_command is of the form:
   *
   * L<Desired left wheel velocity>;R<Desired right wheel velocity>;
   *
   * Desired wheel velocities come from the diff_drive_controller; they are in
   * radians/second.
   */
  std::stringstream motors_command;
  motors_command << "L" << velocity_commands_[0] << ";R"
    << velocity_commands_[1] << ";";

  // Send desired motors command to Arduino
  try {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("RobomagellanInterface"),
        "Attempting to send motors_command: " << motors_command.str());
    arduino_.Write(motors_command.str());
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("RobomagellanInterface"),
        "ERROR: Unable to send motors_command " << motors_command.str() <<
        " to Arduino on port: " << port_);
    return hardware_interface::return_type::ERROR;
  }

  // Update last sent velocities
  last_left_cmd_  = velocity_commands_[0];
  last_right_cmd_ = velocity_commands_[1];

  return hardware_interface::return_type::OK;
}

// Export the plugin
PLUGINLIB_EXPORT_CLASS(robomagellan_firmware::RobomagellanInterface,
                       hardware_interface::SystemInterface)
} // End namespace robomagellan_firmware
