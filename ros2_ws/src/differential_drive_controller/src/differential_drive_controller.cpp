/*
 * differential_drive_controller.cpp
 *
 * Class implementation of a differential drive controller using the ROS 2
 * Controls framework.
 */

#include "differential_drive_controller/differential_drive_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

// Custom topic names
namespace {
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
} // End namespace

namespace differential_drive_controller {
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

// Default Constructor
DifferentialDriveController::DifferentialDriveController() : controller_interface::ControllerInterface() {}

// Create command interfaces (what can be controlled)
InterfaceConfiguration DifferentialDriveController::command_interface_configuration() const {
  std::vector<std::string> conf_names;
  conf_names.push_back(params_.left_joint_name + "/" + HW_IF_VELOCITY);
  conf_names.push_back(params_.right_joint_name + "/" + HW_IF_VELOCITY);
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

// Create state interfaces (feedback received)
InterfaceConfiguration DifferentialDriveController::state_interface_configuration() const {
  std::vector<std::string> conf_names;
  conf_names.push_back(params_.left_joint_name + "/" + feedback_type());
  conf_names.push_back(params_.right_joint_name + "/" + feedback_type());
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}


// Called when controller is initialized
controller_interface::CallbackReturn DifferentialDriveController::on_init() {
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception &e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}


// Using position or velocity feedback?
const char * DifferentialDriveController::feedback_type() const {
  return params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

// Configure a wheel's side
controller_interface::CallbackReturn DifferentialDriveController::configure_side(
  const std::string &side, const std::vector<std::string> &wheel_names,
  std::vector<WheelHandle> & registered_handles) {
  auto logger = get_node()->get_logger();

  if (wheel_names.empty()) {
    RCLCPP_ERROR(logger, "No '%s' wheel names specified", side.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(wheel_names.size());
  for (const auto & wheel_name : wheel_names) {
    const auto interface_name = feedback_type();
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&wheel_name, &interface_name](const auto & interface) {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == interface_name;
      });

    if (state_handle == state_interfaces_.cend()) {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&wheel_name](const auto & interface) {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_VELOCITY;
      });

    if (command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
      WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// Called when the controller is asked to reset
bool DifferentialDriveController::reset() {
  // Reset odometry
  odometry_.resetOdometry();

  // Erase last commands
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);

  // Deactivate wheels
  registered_left_wheel_handles_.clear();
  registered_right_wheel_handles_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

// Called when the controller is asked to halt
void DifferentialDriveController::halt() {
  const auto halt_wheels = [](auto & wheel_handles) {
    for (const auto & wheel_handle : wheel_handles) {
      wheel_handle.velocity.get().set_value(0.0);
    }
  };

  halt_wheels(registered_left_wheel_handles_);
  halt_wheels(registered_right_wheel_handles_);
}
} // End namespace differential_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    differential_drive_controller::DifferentialDriveController,
    controller_interface::ControllerInterface
)
