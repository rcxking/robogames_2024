/*
 * runner.cpp
 *
 * Main program for the Robogames 2024 RoboMagellan competition.
 *
 * Bryant Pong
 * 11/27/23
 */
#include <coordinate_parser/CoordinateParser.h>
#include <ros/ros.h>

#include <string>

int main(int argc, char **argv) {
	ROS_INFO("Starting RoboMagellan 2024 Run");

	// ROS initialization
	ros::init(argc, argv, "runner");

	// NodeHandle to grab ROS params
	ros::NodeHandle nh("~");

	// Grab the filepath containing the mission's coordinates
	std::string filepath;
	if (nh.getParam("filepath", filepath)) {
		ROS_INFO("Will load mission coordinates from filepath: %s", filepath.c_str());
	} else {
		ROS_ERROR("Filepath %s does not exist!", filepath.c_str());
		return 1;
	}

	// Check if we're using the simulator or not
	bool use_sim;
	nh.param("use_sim", use_sim, false);

	// Parse GPS coordinates
	CoordinateParser cp;
	if (!cp.LoadCoordinateFile(filepath, use_sim)) {
		ROS_ERROR("Unable to parse coordinates from file: %s", filepath.c_str());
		return 1;
	}

	// Display mission coordinates and ask for user confirmation before starting
	cp.DisplayMissionCoordinates();
	ROS_INFO("If the coordinates are correct, enter \'Y\' to proceed");
	char input;
	scanf("%s", &input);
	if (input != 'Y') {
		// Quit
		ROS_INFO("Aborting run");
		return 0;
	} else {
		ROS_INFO("Starting run");
	}

	while (ros::ok()) {

	}
	return 0;
}


