/*
 * send_lin_ang_commands.cpp
 *
 */
#include <csignal>
#include <iostream>
#include <ros/ros.h>
#include <rpi_motors/RPIMotorsLinAng.h>

// Service client to the RPI Motors node
ros::ServiceClient client;

// Next motor command to send
rpi_motors::RPIMotorsLinAng next_command;

// Signal handler to handle shutdowns
void ShutdownSignalHandler(const int sig) {
  std::cout << "Shutting down.  Sending 0 velocity command." << std::endl;

  // Send a 0 velocity command before shutting down
  next_command.request.linear_velocity = 0.0;
  next_command.request.angular_velocity = 0.0;

  client.call(next_command);

  ros::shutdown();
}

int main(int argc, char *argv[]) {
  // Initialize ROS
  ros::init(argc, argv, "send_lin_ang_commands");

  ros::NodeHandle nh;
  client = nh.serviceClient<rpi_motors::RPIMotorsLinAng>(
      "rpi_motor_commands_lin_ang");

  // Attach SIGINT signal handler
  signal(SIGINT, ShutdownSignalHandler);

  // Initialize next command to send
  next_command.request.linear_velocity = 0.0;
  next_command.request.angular_velocity = 0.0;

  /*
   * While this program is active maintain a REPL that allows users to send
   * commands to the RPI Motors node.
   */
  while (ros::ok()) {
    std::cout << "Please enter next desired linear/angular velocity: ";

    double lin_vel = 0.0, ang_vel = 0.0;
    std::cin >> lin_vel >> ang_vel;

    std::cout << "Sending linear: " << lin_vel << " m/s; angular: " << ang_vel
      << " rad/s" << std::endl;

    if (!client.call(next_command)) {
      std::cerr << "ERROR: failed to send command to RPI Motors node" <<
        std::endl;
    }
  }
  return 0;
}
