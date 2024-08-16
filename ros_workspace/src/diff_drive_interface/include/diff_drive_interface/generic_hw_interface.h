/*
 * generic_hw_interface.h
 *
 * Class definition of a generic hardware interface for the differential drive
 * controller.
 */
#ifndef __GENERIC_HW_INTERFACE_H__
#define __GENERIC_HW_INTERFACE_H__

#include <string>
#include <vector>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <urdf/model.h>

namespace diff_drive_interface {

class GenericHWInterface : public hardware_interface::RobotHW {

  public:
    /*
     * Constructor.
     * nh: Node handle for topics
     * urdf: Optional pointer to a parsed URDF robot model
     */
    GenericHWInterface(const ros::NodeHandle &nh, urdf::Model *urdf_model=nullptr);

    // Destructor
    virtual ~GenericHWInterface() {}

    // Initialize the hardware interface
    virtual void init();

    // Read the state from the robot hardware
    virtual void read(ros::Duration &elapsed_time) = 0;

    /*
     * Read the state from the robot hardware.  This function is forwarded from
     * RobotHW::read().
     * time: Current time (unused)
     * period: Time passed since the last call to read()
     */
    virtual void read(const ros::Time &, const ros::Duration &period) override {
      ros::Duration elapsed_time = period;
      read(elapsed_time);
    }

    // Write the command to the robot hardware
    virtual void write(ros::Duration &elapsed_time) = 0;

    /*
     * Write the command to the robot hardware.  This function is forwarded
     * from RobotHW::write().
     * time: Current time (unused)
     * period: Time passed since the last call to write()
     */
    virtual void write(const ros::Time &, const ros::Duration &period) override {
      ros::Duration elapsed_time = period;
      write(elapsed_time);
    }

    // Set all members to default values
    virtual void reset();

    // Helper to debug a joint's state
    virtual void printState();
    std::string printStateHelper();

    // Helper to debug a joint's command
    std::string printCommandHelper();

  protected:
    // Get the URDF XML from the parameter server
    virtual void loadURDF(const ros::NodeHandle &nh, const std::string &param_name);

    // Name of this class
    std::string name_;

    // Internal NodeHandle
    ros::NodeHandle nh_;

    // Differential drive controller only has velocity interfaces
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    // Configuration
    std::vector<std::string> joint_names_;
    size_t num_joints_;
    urdf::Model *urdf_model_;

    // States
    std::vector<double> joint_velocity_;

    // Commands
    std::vector<double> joint_velocity_command_;
}; // End class GenericHWInterface
} // End namespace diff_drive_interface

#endif
