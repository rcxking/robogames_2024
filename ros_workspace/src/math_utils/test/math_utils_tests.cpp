/*
 * math_utils_tests.cpp
 *
 * Catch2 unit tests for the MathUtils library.
 *
 * Bryant Pong
 * 12/11/23
 */
#include <catch_ros/catch.hpp>
#include <math_utils/MathUtils.h>

#include <cmath>
#include <limits>

// Helper to compare 2 double precision variables
bool DoubleCompare(const double num1, const double num2) {
	return std::fabs(num1 - num2) <= std::numeric_limits<double>::epsilon();
}

SCENARIO("Different math utilities are called") {
	GIVEN("An angle in radians") {
		constexpr double theta = M_PI/4.;

		WHEN("The haversine is computed") {
			const double hav = Hav(theta);
			THEN("The haversine is computed correctly") {
				const double exp_hav = sin(theta/2) * sin(theta/2);
				REQUIRE(DoubleCompare(exp_hav, hav) == true);
			}
		}

		WHEN("The inverse haversine is computed") {
			const double inv_hav = InvHav(theta);
			THEN("The inverse haversine is computed correctly") {
				const double exp_inv_hav = 2 * asin(sqrt(theta));
				REQUIRE(DoubleCompare(exp_inv_hav, inv_hav) == true);
			}
		}
	}
}




