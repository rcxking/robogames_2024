/*
 * Odometry.h
 *
 * Class definition of the odometry module for the real robot.
 *
 *  Created on: Jan 12, 2024
 *      Author: bryant
 */

#ifndef INCLUDE_ODOMETRY_ODOMETRY_H_
#define INCLUDE_ODOMETRY_ODOMETRY_H_

#include <arduino_connector/Encoders.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

#include <string>

class Odometry {
public:
	//! Default Constructor
	Odometry(const std::string& encoder_topic) :
		encoder_topic_(encoder_topic),
		last_left_encoder_ticks_(0), last_right_encoder_ticks_(0) {
		// Initialize the previous and current odometry messages
		prev_odom_.header.stamp = ros::Time::now();
		prev_odom_.header.frame_id = "odom";
		prev_odom_.child_frame_id = "base_link";
		prev_odom_.pose.pose.position.x = 0.0;
		prev_odom_.pose.pose.position.y = 0.0;
		prev_odom_.pose.pose.orientation.z = 0.0;

		cur_odom_.header.stamp = ros::Time::now();
		cur_odom_.header.frame_id = "odom";
		cur_odom_.child_frame_id = "base_link";
		cur_odom_.pose.pose.position.x = 0.0;
		cur_odom_.pose.pose.position.y = 0.0;
		cur_odom_.pose.pose.orientation.z = 0.0;
		cur_odom_.twist.twist.linear.x = 0.0;
		cur_odom_.twist.twist.angular.z = 0.0;

		// Initialize tf broadcast message
		odom_trans_.header.frame_id = "odom";
		odom_trans_.child_frame_id = "base_link";

		ros::NodeHandle nh("~");
		// Initialize encoder topic subscriber
		// TODO: Make the topic a parameter
		encoder_sub_ = nh.subscribe(encoder_topic_, 100,
				&Odometry::HandleEncodersMessage, this);

		// Initialize odometry publisher
		odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 100);

		// Initialize left/right velocity publishers
		left_vel_pub_ = nh.advertise<std_msgs::Float32>("left_vel", 100);
		right_vel_pub_ = nh.advertise<std_msgs::Float32>("right_vel", 100);
	}

	/**
	 * @brief Publishes odometry data to both ROS and tf.
	 */
	void PublishData();

	/**
	 * @brief Helper function to calculate the difference between 2 encoder tick
	 * values.
	 * @param prev The previous encoder tick value.
	 * @param next The next encoder tick value.
	 * @return int32_t Number of encoder ticks between prev and next.
	 */
	int32_t CalculateTicksDifference(const int32_t prev, const int32_t next) const;

	/**
	 * @brief Callback to update the robot's odometry information whenever new
	 * encoder information is received.
	 * @param msg The Encoders message received.
	 */
	void HandleEncodersMessage(const arduino_connector::Encoders::ConstPtr& msg);

	//! Accessors
	std::string GetEncoderTopic() const { return encoder_topic_; }
	int32_t GetLastLeftEncoderTicks() const { return last_left_encoder_ticks_; }
	int32_t GetLastRightEncoderTicks() const { return last_right_encoder_ticks_; }

	nav_msgs::Odometry GetCurrentOdom() const { return cur_odom_; }

private:
	//! Subscribe to this topic for encoder info
	std::string encoder_topic_;

	/*
	 * 1) Tick to Meters conversion. The CIMCoders provide 20 pulses per revolution,
	 * which comes out to 1 pulse equating to (pi/10) radians.  The gear train
	 * places an 11 tooth gear on the motor shaft connected to a 72 gear tooth on
	 * the drive shaft.  The wheels have a diameter of 0.15443 meters which gives a
	 * radius of 0.077215 meters.  Therefore, 1 tick is:
	 *          pi            11   2*pi*0.077215 meters
	 * 1 Tick = --  radians * -- * -------------------- = 0.0037 meters
	 *          10            72       2*pi radians
	 */
	static constexpr double METERS_PER_TICK = 0.0037;

	//! Distance from the wheel centers to the robot's midpoint (meters)
	static constexpr double WHEEL_TO_MIDPOINT = 0.28575;

	//! Last encoder ticks for each side
	int32_t last_left_encoder_ticks_, last_right_encoder_ticks_;

	//! Current and previous robot odom messages
	nav_msgs::Odometry prev_odom_, cur_odom_;

	//! Transform message
	geometry_msgs::TransformStamped odom_trans_;

	//! Subscriber to the encoder data
	ros::Subscriber encoder_sub_;

	//! Odometry topic publisher
	ros::Publisher odom_pub_;

	//! Topics to publish the left/right velocities (m/s)
	ros::Publisher left_vel_pub_, right_vel_pub_;

	//! Broadcaster to publish odometry via tf
	tf::TransformBroadcaster tf_broadcaster_;
};

#endif /* INCLUDE_ODOMETRY_ODOMETRY_H_ */
