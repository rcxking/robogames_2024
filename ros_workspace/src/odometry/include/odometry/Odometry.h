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

#include <arduino_connector/SensorStates.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <odometry/Velocities.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>

#include <cstring>
#include <string>

class Odometry {
public:
	/**
	 * Constructor
	 *
	 * @param encoder_topic String containing the topic that publishes Encoder
	 * info.
	 */
	Odometry(const std::string& encoder_topic) :
		encoder_topic_(encoder_topic),
		last_left_encoder_ticks_(0), last_right_encoder_ticks_(0),
		cur_lin_vel_avg_(0.0), cur_ang_vel_avg_(0.0),
		cur_left_vel_avg_(0.0), cur_right_vel_avg_(0.0) {
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

		vel_buffer_size_ = 20;
		lin_vel_buffer_ = new double[vel_buffer_size_];
		ang_vel_buffer_ = new double[vel_buffer_size_];
		left_vel_buffer_ = new double[vel_buffer_size_];
		right_vel_buffer_ = new double[vel_buffer_size_];
		memset(lin_vel_buffer_, 0, sizeof(double)*vel_buffer_size_);
		memset(ang_vel_buffer_, 0, sizeof(double)*vel_buffer_size_);
		memset(left_vel_buffer_, 0, sizeof(double)*vel_buffer_size_);
		memset(right_vel_buffer_, 0, sizeof(double)*vel_buffer_size_);

		// Initialize tf broadcast message
		odom_trans_.header.frame_id = "odom";
		odom_trans_.child_frame_id = "base_link";

		ros::NodeHandle nh("~");
		// Initialize encoder topic subscriber
		// TODO: Make the topic a parameter
		encoder_sub_ = nh.subscribe(encoder_topic_, 100,
				&Odometry::HandleSensorStatesMessage, this);

		// Initialize odometry publisher
		odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 100);

		// Initialize left/right velocity publisher
		cur_vel_pub_ = nh.advertise<odometry::Velocities>("current_velocities", 100);
	}

	//! Destructor
	~Odometry() {
		// Free up velocity averaging buffers
		if (lin_vel_buffer_ != nullptr) {
			delete [] lin_vel_buffer_;
			lin_vel_buffer_ = nullptr;
		}

		if (ang_vel_buffer_ != nullptr) {
			delete [] ang_vel_buffer_;
			ang_vel_buffer_ = nullptr;
		}

		if (left_vel_buffer_ != nullptr) {
			delete [] left_vel_buffer_;
			left_vel_buffer_ = nullptr;
		}

		if (right_vel_buffer_ != nullptr) {
			delete [] right_vel_buffer_;
			right_vel_buffer_ = nullptr;
		}
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
	 * @param msg The SensorStates message received.
	 */
	void HandleSensorStatesMessage(
      const arduino_connector::SensorStates::ConstPtr& msg);

	/**
	 * @brief Updates and computes the average linear/angular velocities to
	 * filter out encoder aliasing.
	 * @param lin_vel Latest linear velocity (m/s).
	 * @param ang_vel Latest angular velocity (rad/s).
	 * @param left_vel Latest left side velocity (m/s).
	 * @param right_vel Latest right side velocity (m/s).
	 */
	void UpdateVelocityAverages(const double lin_vel, const double ang_vel,
			                        const double left_vel, const double right_vel);

	//! Accessors
	std::string GetEncoderTopic() const { return encoder_topic_; }
	int32_t GetLastLeftEncoderTicks() const { return last_left_encoder_ticks_; }
	int32_t GetLastRightEncoderTicks() const { return last_right_encoder_ticks_; }

	nav_msgs::Odometry GetCurrentOdom() const { return cur_odom_; }

	size_t GetVelBufferSize() const { return vel_buffer_size_; }
	double *GetLinVelBuffer() { return lin_vel_buffer_; }
	double *GetAngVelBuffer() { return ang_vel_buffer_; }
	double *GetLeftVelBuffer() { return left_vel_buffer_; }
	double *GetRightVelBuffer() { return right_vel_buffer_; }

	double GetCurLinVelAvg() const { return cur_lin_vel_avg_; }
	double GetCurAngVelAvg() const { return cur_ang_vel_avg_; }

	double GetCurLeftVelAvg() const { return cur_left_vel_avg_; }
	double GetCurRightVelAvg() const { return cur_right_vel_avg_; }

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

	//! Buffers to help with velocity averaging
	size_t vel_buffer_size_;
	double *lin_vel_buffer_;
	double *ang_vel_buffer_;
	double *left_vel_buffer_;
	double *right_vel_buffer_;

	//! Current linear/angular velocities (m/s and rad/s)
	double cur_lin_vel_avg_, cur_ang_vel_avg_;

	//! Current left/right velocities (m/s)
	double cur_left_vel_avg_, cur_right_vel_avg_;

	//! Transform message
	geometry_msgs::TransformStamped odom_trans_;

	//! Subscriber to the encoder data
	ros::Subscriber encoder_sub_;

	//! Odometry topic publisher
	ros::Publisher odom_pub_;

	//! Topic to publish the left/right velocities (m/s)
	ros::Publisher cur_vel_pub_;

	//! Broadcaster to publish odometry via tf
	tf::TransformBroadcaster tf_broadcaster_;

	//! No Default Constructor/Copy Constructor/Assignment Operator=
	Odometry() = delete;
	Odometry(const Odometry& rhs) = delete;
	Odometry& operator=(const Odometry& rhs) = delete;
};

#endif /* INCLUDE_ODOMETRY_ODOMETRY_H_ */
