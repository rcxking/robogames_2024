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
#include <odometry/Velocities.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
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
  ROS_INFO("%s:%d: delta_left_ticks: %d; delta_right_ticks: %d", __FUNCTION__,
      __LINE__, delta_left_ticks, delta_right_ticks);

	// Determine the distance in meters each wheel has traveled this cycle
	const double left_dist_t = delta_left_ticks * METERS_PER_TICK;
	const double right_dist_t = delta_right_ticks * METERS_PER_TICK;
  ROS_INFO("%s:%d: left_dist_t: %f meters; right_dist_t: %f meters", __FUNCTION__,
      __LINE__, left_dist_t, right_dist_t);

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

	/*
	 * Update current robot's velocities.  Because of aliasing from the robot's
	 * encoders (the signal jitters), average out the readings instead of
	 * publishing the robot's instantaneous velocities.
	 */
	const double delta_time = (msg->stamp.toSec() - prev_odom_.header.stamp.toSec());
	const double new_linear_vel = dist_t / delta_time;
	const double new_angular_vel = delta_theta_t / delta_time;

	UpdateVelocityAverages(new_linear_vel, new_angular_vel);

  cur_odom_.header.stamp = msg->stamp;
	cur_odom_.twist.twist.linear.x = cur_lin_vel_avg_;
	cur_odom_.twist.twist.angular.z = cur_ang_vel_avg_;

	// Publish each side's linear velocity
	const double left_vel = left_dist_t / delta_time;
	const double right_vel = right_dist_t / delta_time;
  ROS_INFO("%s:%d: left_vel: %f m/s; right_vel: %f m/s; delta_time: %f seconds",
      __FUNCTION__, __LINE__, left_vel, right_vel, delta_time);

	odometry::Velocities vel_msg;
	vel_msg.stamp = ros::Time::now();
	vel_msg.left_velocity = left_vel;
	vel_msg.right_velocity = right_vel;
	cur_vel_pub_.publish(vel_msg);

	// Update odometry data over tf
	const geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(
																								cur_odom_.pose.pose.orientation.z);
	odom_trans_.header.stamp = msg->stamp;
	odom_trans_.transform.translation.x = cur_odom_.pose.pose.position.x;
	odom_trans_.transform.translation.y = cur_odom_.pose.pose.position.y;
	odom_trans_.transform.translation.z = 0.0;
	odom_trans_.transform.rotation = odom_quat;

	// Update the last encoder ticks
	last_left_encoder_ticks_ = cur_left_ticks;
	last_right_encoder_ticks_ = cur_right_ticks;

	// Update the previous odometry information
	prev_odom_.header.stamp = msg->stamp;
	prev_odom_.pose.pose.position.x = cur_odom_.pose.pose.position.x;
	prev_odom_.pose.pose.position.y = cur_odom_.pose.pose.position.y;
	prev_odom_.pose.pose.orientation.z = cur_odom_.pose.pose.orientation.z;
}

void Odometry::UpdateVelocityAverages(const double lin_vel,
		                                  const double ang_vel) {
	// Current position to update buffer values
	static size_t cur_pos = 0;

	const double lin_vel_to_drop = lin_vel_buffer_[cur_pos];
	const double new_lin_avg = ((vel_buffer_size_ * cur_lin_vel_avg_) -
			lin_vel_to_drop + lin_vel) / vel_buffer_size_;

	const double ang_vel_to_drop = ang_vel_buffer_[cur_pos];
	const double new_ang_avg = ((vel_buffer_size_ * cur_ang_vel_avg_) -
			ang_vel_to_drop + ang_vel) / vel_buffer_size_;

	// Update buffers with current velocities
	lin_vel_buffer_[cur_pos] = lin_vel;
	ang_vel_buffer_[cur_pos] = ang_vel;

	// Update current velocity averages
	cur_lin_vel_avg_ = new_lin_avg;
	cur_ang_vel_avg_ = new_ang_avg;

	// Update next position for next call
	cur_pos = (cur_pos + 1) % vel_buffer_size_;
}

void Odometry::PublishData() {
	odom_pub_.publish(cur_odom_);
	tf_broadcaster_.sendTransform(odom_trans_);
}
