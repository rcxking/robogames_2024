/*
 * pure_pursuit_controller_tests.cpp
 *
 *  Created on: Jan 4, 2024
 *      Author: bryant
 */

#include <catch_ros/catch.hpp>
#include <pure_pursuit_controller/PurePursuitController.h>

#include <limits>

// Helper function to compare 2 floating point numbers
double DoubleCompare(const double num1, const double num2) {
	return std::fabs(num1 - num2) <= std::numeric_limits<double>::epsilon();
}

// Pure Pursuit Controller construction tests
SCENARIO("A PurePursuitController is constructed", "[PurePursuitController]") {
	GIVEN("A PurePursuitController object is constructed") {
		THEN("It is constructed without errors/exceptions") {
			REQUIRE_NOTHROW(PurePursuitController());
		}
	}

	GIVEN("A PurePursuitController is constructed") {
		THEN("Its member variables are correctly instantiated") {

		}
	}
}

// Pure Pursuit Controller usage tests
SCENARIO("A PurePursuitController is to be used", "[PurePursuitController]") {

	// Circle-line intersection tests
	GIVEN("A PurePursuitController object and several circles") {
		PurePursuitController pp;

		/*
		 * Circle is assumed to have a radius of 1 unit and centered around the
		 * origin (0, 0).
		 */
		constexpr double circle_radius = 1.0;

		/*
		 * Test cases:
		 *
		 * 1) (0 Intersections) - Line runs from (-5, 5) to (5, 5)
		 * 2) (1 Intersection) - Line runs from (-5, 1) to (5, 1)
		 * 3) (2 Intersections) - Line runs from (-5, 0) to (5, 0)
		 */
		geometry_msgs::Point p1, p2, p3, p4, p5, p6;
		p1.x = -5.0;
		p1.y = 5.0;

		p2.x = 5.0;
		p2.y = 5.0;

		p3.x = -5.0;
		p3.y = 1.0;

		p4.x = 5.0;
		p4.y = 1.0;

		p5.x = -5.0;
		p5.y = 0.0;

		p6.x = 5.0;
		p6.y = 0.0;

		WHEN("The intersections are computed between the circle and the lines") {
			geometry_msgs::Point pt1, pt2, pt3;
			const bool res1 = pp.FindCircleLineIntersections(
					p1, p2, circle_radius, &pt1);
			const bool res2 = pp.FindCircleLineIntersections(
					p3, p4, circle_radius, &pt2);
			const bool res3 = pp.FindCircleLineIntersections(
					p5, p6, circle_radius, &pt3);

			THEN("The results are correct") {
				REQUIRE(res1 == false);
				REQUIRE(res2 == true);
				REQUIRE(res3 == true);

				// pt2 should be at (0, 1)
				REQUIRE(DoubleCompare(pt2.x, 0.0) == true);
				REQUIRE(DoubleCompare(pt2.y, 1.0) == true);

				// pt3 should be at (1, 0)
				REQUIRE(DoubleCompare(pt3.x, 1.0) == true);
				REQUIRE(DoubleCompare(pt3.y, 0.0) == true);
			}
		}
	}
}
