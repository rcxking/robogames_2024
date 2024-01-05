/*
 * pure_pursuit_controller_node.cpp
 *
 *  Created on: Jan 4, 2024
 *      Author: bryant
 */
#include <pure_pursuit_controller/PurePursuitController.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
	ROS_INFO("%s:%d: Starting Pure Pursuit Controller", __FUNCTION__, __LINE__);

	// ROS initialization
	ros::init(argc, argv, "pure_pursuit_controller");

	// Construct the pure pursuit controller
	PurePursuitController pp;

	// Listen and follow any requested plan
	pp.spin();

	return 0;
}


