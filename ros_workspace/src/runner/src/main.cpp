/*
 * main.cpp
 *
 * Main program for the Robogames 2024 RoboMagellan competition.
 *
 * Bryant Pong
 * 11/27/23
 */
#include <runner/Runner.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
	ROS_INFO("Starting RoboMagellan 2024 Run");

	// ROS initialization
	ros::init(argc, argv, "runner");

  // Runner object that handles the mission
  Runner runner;

  // Load the mission coordinates
  if (!runner.LoadCoordinateFileFromParam()) {
    ROS_ERROR("%s:%d: ERROR: Unable to load coordinate file from param",
        __FUNCTION__, __LINE__);
    return 1;
  } else {
    ROS_INFO("%s:%d: Successfully loaded coordinate file from param",
        __FUNCTION__, __LINE__);
  }

  // Confirm with the user mission coordinates are correct
  const bool coords_correct = runner.ConfirmMissionCoordinates();
  if (!coords_correct) {
    ROS_INFO("%s:%d: User rejected coordinates.  Aborting run.", __FUNCTION__,
        __LINE__);
    return 1;
  } else {
    ROS_INFO("%s:%d: User accepted coordinates.  Starting run.", __FUNCTION__,
        __LINE__);
  }

  ros::Rate r(200);
	while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
	}
	return 0;
}


