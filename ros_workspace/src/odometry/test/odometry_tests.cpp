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
	}
}
