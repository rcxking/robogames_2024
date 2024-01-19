/*
 * Odometry.cpp
 *
 * Class implementation of the odometry module for the real robot.
 *
 *  Created on: Jan 12, 2024
 *      Author: bryant
 */
#include <arduino_connector/Encoders.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <odometry/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <cmath>

int32_t Odometry::CalculateTicksDifference(
		const int32_t prev, const int32_t next) const {
	// Using an int64_t and typecasting to int32_t factors in underflow/overflow
	const int64_t diff = next - prev;
	return static_cast<int32_t>(diff);
}

void Odometry::HandleEncodersMessage(
		const arduino_connector::Encoders::ConstPtr& msg) {
	ROS_INFO("%s:%d: Received at time %f seconds: left_encoder_ticks: %d; "
			"right_encoder_ticks: %d", __FUNCTION__, __LINE__, msg->stamp.toSec(),
			msg->left_ticks, msg->right_ticks);

	const int32_t cur_left_ticks = msg->left_ticks;
	const int32_t cur_right_ticks = msg->right_ticks;

	// Compute the difference in ticks between the last reading
	const int32_t delta_left_ticks = CalculateTicksDifference(
			last_left_encoder_ticks_, cur_left_ticks);
	const int32_t delta_right_ticks = CalculateTicksDifference(
			last_right_encoder_ticks_, cur_right_ticks);

	// Determine the distance in meters each wheel has traveled this cycle
	const double left_dist_t = delta_left_ticks * METERS_PER_TICK;
	const double right_dist_t = delta_right_ticks * METERS_PER_TICK;

	// Compute the distance the robot midpoint has traveled this cycle
	const double dist_t = (left_dist_t + right_dist_t) / 2.0;

	// Compute the change in heading this cycle
	const double delta_theta_t = (right_dist_t - left_dist_t) / (2 * WHEEL_TO_MIDPOINT);
	const double half_delta_theta_t = delta_theta_t / 2.0;

	// Update current robot's odometry pose
	const double trig_inside = prev_odom_.pose.pose.orientation.z +
			half_delta_theta_t;
	cur_odom_.pose.pose.position.x = prev_odom_.pose.pose.position.x + (dist_t * cos(trig_inside));
	cur_odom_.pose.pose.position.y = prev_odom_.pose.pose.position.y + (dist_t * sin(trig_inside));
	cur_odom_.pose.pose.orientation.z = prev_odom_.pose.pose.orientation.z + delta_theta_t;

	// Update current robot's velocities
	cur_odom_.header.stamp = ros::Time::now();
	const double delta_time = (cur_odom_.header.stamp.toSec() - prev_odom_.header.stamp.toSec());
	cur_odom_.twist.twist.linear.x = dist_t / delta_time;
	cur_odom_.twist.twist.angular.z = delta_theta_t / delta_time;

	// Update odometry data over tf
	const geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(
																								cur_odom_.pose.pose.orientation.z);
	odom_trans_.header.stamp = cur_odom_.header.stamp;
	odom_trans_.transform.translation.x = cur_odom_.pose.pose.position.x;
	odom_trans_.transform.translation.y = cur_odom_.pose.pose.position.y;
	odom_trans_.transform.translation.z = 0.0;
	odom_trans_.transform.rotation = odom_quat;

	// Update the last encoder ticks
	last_left_encoder_ticks_ = cur_left_ticks;
	last_right_encoder_ticks_ = cur_right_ticks;

	// Update the previous odometry information
	prev_odom_.header.stamp = cur_odom_.header.stamp;
	prev_odom_.pose.pose.position.x = cur_odom_.pose.pose.position.x;
	prev_odom_.pose.pose.position.y = cur_odom_.pose.pose.position.y;
	prev_odom_.pose.pose.orientation.z = cur_odom_.pose.pose.orientation.z;
}

void Odometry::PublishData() {
	odom_pub_.publish(cur_odom_);
	tf_broadcaster_.sendTransform(odom_trans_);
}