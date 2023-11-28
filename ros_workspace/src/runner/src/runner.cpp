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

#include <cstdio>
#include <string>

int main(int argc, char **argv) {
	printf("%s:%d: Starting RoboMagellan 2024 Run\n", __FUNCTION__, __LINE__);

	// ROS initialization
	ros::init(argc, argv, "runner");

	// NodeHandle to grab ROS params
	ros::NodeHandle nh("~");

	// Grab the filepath containing the mission's coordinates
	std::string filepath;
	if (nh.getParam("filepath", filepath)) {
		printf("%s:%d: Will load mission coordinates from filepath: %s\n",
				__FUNCTION__, __LINE__, filepath.c_str());
	} else {

		fprintf(stderr, "%s:%d: ERROR: filepath does not exist!\n",
				__FUNCTION__, __LINE__);
		return 1;
	}

	// Parse GPS coordinates
	CoordinateParser cp;
	if (!cp.LoadCoordinateFile(filepath)) {
		fprintf(stderr, "%s:%d: ERROR: Unable to parse coordinates from file: %s\n",
				__FUNCTION__, __LINE__, filepath.c_str());
		return 1;
	}

	// Display mission coordinates and ask for user confirmation before starting
	cp.DisplayMissionCoordinates();
	printf("If the coordinates are correct, enter \'Y\' to proceed\n");
	char input;
	scanf("%s", &input);
	if (input != 'Y') {
		// Quit
		printf("Aborting run\n");
		return 0;
	} else {
		printf("Starting run\n");
	}

	while (ros::ok()) {

	}
	return 0;
}


