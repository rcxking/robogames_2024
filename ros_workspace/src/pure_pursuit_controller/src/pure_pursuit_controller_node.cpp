/*
 * pure_pursuit_controller_node.cpp
 *
 *  Created on: Jan 4, 2024
 *      Author: bryant
 */

#include <ros/ros.h>

int main(int argc, char **argv) {
	ROS_INFO("%s:%d: Starting Pure Pursuit Controller", __FUNCTION__, __LINE__);

	// ROS initialization
	ros::init(argc, argv, "pure_pursuit_controller");
	return 0;
}


