/*
 * odometry.cpp
 *
 * Implements the odometry ROS node for the real robot.
 *
 * Bryant Pong
 * 1/10/24
 */
#include <odometry/Odometry.h>
#include <ros/ros.h>

#include <string>

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n("~");

  /*
   * Acquire ROS parameters.  These are:
   * 1) encoder_topic: Topic containing encoder ticks.
   */
  std::string encoder_topic;
  if (!n.getParam("encoder_topic", encoder_topic)) {
    ROS_ERROR("%s:%d: ERROR: Missing encoder_topic", __FUNCTION__, __LINE__);
    return 1;
  } else {
    ROS_INFO("%s:%d: encoder_topic: %s", __FUNCTION__, __LINE__,
        encoder_topic.c_str());
  }

  // Object that handles computing and publishing odometry information
  Odometry odom(encoder_topic);

  ros::Rate r(30);
  while (n.ok()) {
    odom.PublishData();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
