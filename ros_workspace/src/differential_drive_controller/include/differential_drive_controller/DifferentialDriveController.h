/*
 * DifferentialDriveController.h
 *
 * Class declaration of a differential drive controller.
 */
#ifndef __DIFFERENTIAL_DRIVE_CONTROLLER_H__
#define __DIFFERENTIAL_DRIVE_CONTROLLER_H__

#include <control_msgs/JointTrajectoryControllerState.h>
#include <controller_interface/controller.h>
#include <differential_drive_controller/DifferentialDriveControllerConfig.h>
#include <differential_drive_controller/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <string>
#include <tf/tfMessage.h>

namespace differential_drive_controller {

class DifferentialDriveController
  : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
  public:

    // Default Constructor
    DifferentialDriveController();

    /*
     * Initialize controller.
     * hw: Velocity joint interface for the wheels.
     * root_nh: Node handle at root namespace
     * controller_nh: Node handle inside the controller namespace
     */
    bool init(hardware_interface::VelocityJointInterface *hw,
              ros::NodeHandle &root_nh,
              ros::NodeHandle &controller_nh);

    /*
     * Main update function; updates odometry and computes the next velocity
     * commands.
     * time: Current time
     * period: Time since the last call to this function
     */
    void update(const ros::Time &time, const ros::Duration &period);

    /*
     * Starts the controller.
     * time: Current time
     */
    void starting(const ros::Time &time);

    /*
     * Stops the controller.
     * time: Current time
     */
    void stopping(const ros::Time &time);

  private:
    // Name of this controller
    std::string name_;

    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;
    bool open_loop_;

    // Hardware handles to the left/right joints
    std::vector<hardware_interface::JointHandle> left_wheel_joints_;
    std::vector<hardware_interface::JointHandle> right_wheel_joints_;

    // Previous time
    ros::Time time_previous_;

    // Previous velocities from the encoders
    std::vector<double> vel_left_previous_;
    std::vector<double> vel_right_previous_;

    // Previous desired velocities
    double vel_left_desired_previous_, vel_right_desired_previous_;

    // Struct representing a velocity command
    struct Commands {
      double lin; // Linear velocity (m/s)
      double ang; // Angular velocity (rad/s)
      ros::Time stamp; // Time command was sent

      Commands() : lin(0.0), ang(0.0), stamp(0.0) {}
    };

    // Subscribers/buffers for reading in the desired velocity command
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_;
    ros::Subscriber sub_command_;

    // Publisher of the computed velocity command
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>> cmd_vel_pub_;

    // Odometry publishers
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage>> tf_odom_pub_;
    Odometry odometry_;

    // Controller state publisher
    std::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState>> controller_state_pub_;

    // Wheel separation (distance between wheel midpoints) in meters
    double wheel_separation_;

    // Wheel radius (meters)
    double wheel_radius_;

    // Timeout to consider cmd_vel commands old (seconds)
    double cmd_vel_timeout_;

    // Whether to allow multiple publishers on cmd_vel topic or not
    bool allow_multiple_cmd_vel_publisher_;

    // Robot base TF frame
    std::string base_frame_id_;

    // Odom TF frame
    std::string odom_frame_id_;

    // Number of wheel joints
    size_t wheel_joints_size_;

    // Speed limiters
    //Commands

    /*
     * Sets all wheel velocities to 0.
     */
    void brake();

    /*
     * Velocity command callback.
     * command: Velocity command message
     */
    void CmdVelCallback(const geometry_msgs::Twist &command);
}; // End class DifferentialDriveController
} // End namespace differential_drive_controller

#endif
