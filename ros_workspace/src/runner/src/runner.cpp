/*
 * runner.cpp
 *
 * Main program for the Robogames 2024 RoboMagellan competition.
 *
 * Bryant Pong
 * 11/27/23
 */
#include <coordinate_parser/CoordinateParser.h>
#include <geometry_msgs/Twist.h>
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

#if 0
	ROS_INFO("%s:%d: Starting sanity check", __FUNCTION__, __LINE__);

	// Sanity check: Send velocity command to simulator
	geometry_msgs::Twist vel_cmd;
	vel_cmd.linear.x = 2.0;

	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/robomagellan_2024_diff_drive_controller/cmd_vel", 1);

	ros::Rate loop_rate(10);
	while (ros::ok()) {
		ROS_INFO("%s:%d: Publishing to cmd_vel", __FUNCTION__, __LINE__);
	  vel_pub.publish(vel_cmd);
		ros::spinOnce();
		loop_rate.sleep();
	}
#endif

	//

	ros::spin();
	return 0;
}


