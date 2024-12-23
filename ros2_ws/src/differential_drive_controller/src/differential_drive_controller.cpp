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

} // End namespace differential_drive_controller
