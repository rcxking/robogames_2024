/*
 * odometry_tests.cpp
 *
 * Catch2 unit tests for the Odometry module.
 *
 * Bryant Pong
 * 1/12/24
 */
#include <catch_ros/catch.hpp>
#include <odometry/Odometry.h>

#include <cmath>
#include <limits>

// Helper to compare 2 double precision variables
bool DoubleCompare(const double num1, const double num2) {
  return std::fabs(num1 - num2) <= std::numeric_limits<double>::epsilon();
}

// Odometry class construction tests
SCENARIO("An Odometry object is constructed", "[Odometry]") {
	GIVEN("An Odometry object is constructed") {
		THEN("It should be constructed without any errors/exceptions") {
			REQUIRE_NOTHROW(Odometry("/arduino_connector/encoders"));
		}
	}
	GIVEN("An Odometry object") {
		const std::string encoders_topic("/arduino_connector/encoders");
		Odometry odom(encoders_topic);
		THEN("Its member variables should be correctly instantiated") {
			REQUIRE(odom.GetEncoderTopic() == encoders_topic);

			REQUIRE(odom.GetLastLeftEncoderTicks() == 0);
			REQUIRE(odom.GetLastRightEncoderTicks() == 0);

			// Verify current pose variables (x, y, theta) are (0, 0, 0)
			const nav_msgs::Odometry current_pose = odom.GetCurrentOdom();
			REQUIRE(DoubleCompare(current_pose.pose.pose.position.x, 0.0) == true);
			REQUIRE(DoubleCompare(current_pose.pose.pose.position.y, 0.0) == true);
			REQUIRE(DoubleCompare(current_pose.pose.pose.orientation.x, 0.0) == true);
			REQUIRE(DoubleCompare(current_pose.pose.pose.orientation.y, 0.0) == true);
			REQUIRE(DoubleCompare(current_pose.pose.pose.orientation.z, 0.0) == true);
			REQUIRE(DoubleCompare(current_pose.pose.pose.orientation.w, 0.0) == true);

			// Verify lin/angular averaging buffers have a default size of 20
			REQUIRE(odom.GetVelBufferSize() == 20);
			REQUIRE(odom.GetLinVelBuffer() != nullptr);
			REQUIRE(odom.GetAngVelBuffer() != nullptr);

			// Ensure the buffers are all 0's
			const double *lin_buf = odom.GetLinVelBuffer();
			const double *ang_buf = odom.GetAngVelBuffer();
			for (size_t i = 0; i < odom.GetVelBufferSize(); ++i) {
				REQUIRE(DoubleCompare(lin_buf[i], 0.0) == true);
				REQUIRE(DoubleCompare(ang_buf[i], 0.0) == true);
			}

			// Ensure helper variables to compute velocity averages are initialized
			REQUIRE(DoubleCompare(odom.GetCurLinVelAvg(), 0.0) == true);
			REQUIRE(DoubleCompare(odom.GetCurAngVelAvg(), 0.0) == true);
		}
	}
}

// Odometry class usage tests
SCENARIO("An Odometry object is to be used", "[Odometry]") {
	GIVEN("An Odometry object") {
		Odometry odom("/arduino_connector/encoders");

		WHEN("The difference in encoder ticks are calculated") {
			/*
			 * The encoders ticks are represented by int32_t values.  We need to
			 * check for underflow/overflow.
			 */

			// Normal usage
			const int32_t prev1 = 1;
			const int32_t next1 = 10;
			const int32_t res1 = odom.CalculateTicksDifference(prev1, next1);

			const int32_t prev2 = 10;
			const int32_t next2 = 1;
			const int32_t res2 = odom.CalculateTicksDifference(prev2, next2);

			// Overflow
			const int32_t prev3 = std::numeric_limits<int32_t>::max();
			const int32_t next3 = std::numeric_limits<int32_t>::min();
			const int32_t res3 = odom.CalculateTicksDifference(prev3, next3);

			// Underflow
			const int32_t prev4 = std::numeric_limits<int32_t>::min();
			const int32_t next4 = std::numeric_limits<int32_t>::max();
			const int32_t res4 = odom.CalculateTicksDifference(prev4, next4);

			THEN("The differences are correct") {
				REQUIRE(res1 == 9);
				REQUIRE(res2 == -9);
				REQUIRE(res3 == 1);
				REQUIRE(res4 == -1);
			}
		}

		WHEN("A new linear/angular velocity is to be added") {
			THEN("The velocity averages are correct") {
				double new_lin_vel = 0.1;
				double new_ang_vel = 0.2;

				double lin_vel_sum = 0.0;
				double ang_vel_sum = 0.0;

				for (int i = 0; i < 21; ++i) {
					odom.UpdateVelocityAverages(new_lin_vel, new_ang_vel);

					/*
					 * The default buffer sizes is 20, so adding 21 entries will
					 * test out the rollover/modulo math for the running buffer.
					 * At each iteration we expect the linear/angular velocity values
					 * to be updated appropriately.
					 */
					lin_vel_sum += new_lin_vel;
					ang_vel_sum += new_ang_vel;

					// For the 21th iteration, need to subtract the original 0.1/0.2
					if (i == 20) {
						lin_vel_sum -= 0.1;
						ang_vel_sum -= 0.2;
					}

					const double expected_lin_vel_avg = lin_vel_sum / 20;
					const double expected_ang_vel_avg = ang_vel_sum / 20;

					REQUIRE(DoubleCompare(expected_lin_vel_avg,
							odom.GetCurLinVelAvg()) == true);
					REQUIRE(DoubleCompare(expected_ang_vel_avg,
							odom.GetCurAngVelAvg()) == true);

					new_lin_vel += 0.1;
					new_ang_vel += 0.1;
				}
			}
		}
	}
}
