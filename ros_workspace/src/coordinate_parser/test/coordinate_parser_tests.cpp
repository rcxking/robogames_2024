/*
 * coordinate_parser_tests.cpp
 *
 * Catch2 unit tests for the CoordinateParser class.
 *
 * Bryant Pong
 * 11/24/23
 */
#include <catch_ros/catch.hpp>
#include <coordinate_parser/CoordinateParser.h>

#include <limits>
#include <string>

// Helper to compare 2 double precision variables
bool DoubleCompare(const double num1, const double num2) {
	return std::fabs(num1 - num2) <= std::numeric_limits<double>::epsilon();
}

// GPSCoordinate tests
SCENARIO("A GPSCoordinate is constructed", "[GPSCoordinate]") {
	GIVEN("A GPSCoordinate is constructed") {
		THEN("It should be constructed without any errors") {
			REQUIRE_NOTHROW(GPSCoordinate());
		}
	}

	GIVEN("A default GPSCoordinate is constructed") {
		GPSCoordinate gps;
		THEN("It should be instantiated correctly") {
			REQUIRE(DoubleCompare(gps.latitude, 0.0) == true);
			REQUIRE(DoubleCompare(gps.longitude, 0.0) == true);
		}
	}

	GIVEN("A latitude and longitude") {
		constexpr double latitude = 5.67;
		constexpr double longitude = -1.23;
		WHEN("A GPSCoordinate is constructed") {
			GPSCoordinate gps(latitude, longitude);
			THEN("It should be instantiated correctly") {
				REQUIRE(DoubleCompare(gps.latitude, latitude) == true);
				REQUIRE(DoubleCompare(gps.longitude, longitude) == true);
			}
		}
	}

	GIVEN("3 GPSCoordinates") {
		GPSCoordinate g1(1.2, 3.4);
		GPSCoordinate g2(1.2, 3.4);
		GPSCoordinate g3(0.0, 0.0);

		WHEN("The coordinates are compared with each other") {
			const bool res1 = (g1 == g2);
			const bool res2 = (g2 == g3);
			const bool res3 = (g1 == g3);

			THEN("The comparisons are correct") {
				REQUIRE(res1 == true);
				REQUIRE(res2 == false);
				REQUIRE(res3 == false);
			}
		}
	}
}

// CoordinateParser construction tests
SCENARIO("A CoordinateParser is constructed", "[CoordinateParser]") {
  GIVEN("A CoordinateParser is constructed") {
    THEN("It should be constructed without any errors") {
      REQUIRE_NOTHROW(CoordinateParser());
    }
  }

  GIVEN("A CoordinateParser is constructed") {
  	CoordinateParser cp;
  	THEN("Its member variables should be initialized correctly") {
  		REQUIRE(cp.GetWaypoints().empty() == true);
  		REQUIRE(cp.AreWaypointsSet() == false);

  		// Can't get a start/end point because waypoints aren't set
  		GPSCoordinate start, end;
  		REQUIRE(cp.GetStartPoint(&start) == false);
  		REQUIRE(cp.GetEndPoint(&end) == false);
  	}
  }

  GIVEN("A CoordinateParser is constructed") {
  	CoordinateParser cp;

  	// TODO: Pass in test file paths as ROS params
  	const std::string base_path("/home/bryant/robogames_2024/ros_workspace/src/coordinate_parser/test/");
  	const std::string nonexistent_file_path("does_not_exist.txt");
  	const std::string empty_file_path(base_path + "empty_file.txt");
  	const std::string invalid_file_path1(base_path + "invalid_file1.txt");
  	const std::string good_file_path1(base_path + "good_file1.txt");

  	WHEN("A nonexistent file path is loaded") {
  		const bool ret = cp.LoadCoordinateFile(nonexistent_file_path);
  		THEN("The file fails to load and the waypoints do not change") {
  			REQUIRE(ret == false);
  			REQUIRE(cp.GetWaypoints().empty() == true);

  			// Can't get a start/end point because waypoints aren't set
  			GPSCoordinate start, end;
  			REQUIRE(cp.GetStartPoint(&start) == false);
  			REQUIRE(cp.GetEndPoint(&end) == false);
  		}
  	}

  	WHEN("An empty file path is loaded") {
  		const bool ret = cp.LoadCoordinateFile(empty_file_path);
  		THEN("The file fails to load and the waypoints are not updated") {
  			REQUIRE(ret == false);
  			REQUIRE(cp.GetWaypoints().empty() == true);

  			// Can't get a start/end point because waypoints aren't set
  			GPSCoordinate start, end;
  			REQUIRE(cp.GetStartPoint(&start) == false);
  			REQUIRE(cp.GetEndPoint(&end) == false);
  		}
  	}

  	WHEN("An invalid file with malformed coordinates is loaded") {
  		const bool ret = cp.LoadCoordinateFile(invalid_file_path1);
  		THEN("The file fails to load and the waypoints are not updated") {
  			REQUIRE(ret == false);
  			REQUIRE(cp.GetWaypoints().empty() == true);

  			// Can't get a start/end point because waypoints aren't set
  			GPSCoordinate start, end;
  			REQUIRE(cp.GetStartPoint(&start) == false);
  			REQUIRE(cp.GetEndPoint(&end) == false);
  		}
  	}

  	WHEN("A file containing coordinates is loaded") {
  		const bool ret = cp.LoadCoordinateFile(good_file_path1);
  		THEN("The file successfully loads and the waypoints are updated") {
  			REQUIRE(ret == true);
  			REQUIRE(cp.GetWaypoints().size() == 2);

  			GPSCoordinate start, end;
  			REQUIRE(cp.GetStartPoint(&start) == true);
  			REQUIRE(cp.GetEndPoint(&end) == true);

  			REQUIRE(start == GPSCoordinate(0.0, 0.0));
  			REQUIRE(end == GPSCoordinate(1.3, 2.4));
  		}
  	}
  }
}
