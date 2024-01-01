/*
 * pure_pursuit_controller_tests.cpp
 *
 *  Created on: Jan 4, 2024
 *      Author: bryant
 */

#include <catch_ros/catch.hpp>
#include <pure_pursuit_controller/PurePursuitController.h>

// Pure Pursuit Controller construction tests
SCENARIO("A PurePursuitController is constructed", "[PurePursuitController]") {
	GIVEN("A PurePursuitController object is constructed") {
		THEN("It is constructed without errors/exceptions") {
			REQUIRE_NOTHROW(PurePursuitController());
		}
	}
}
