/*
 * runner_tests.cpp
 *
 * Catch2 unit tests for the Runner module.
 *
 * Bryant Png
 * 2/13/24
 */
#include <catch_ros/catch.hpp>
#include <runner/Runner.h>

#include <cmath>
#include <limits>

// Helper to compare 2 double precision variables
bool DoubleCompare(const double num1, const double num2) {
  return std::fabs(num1 - num2) <= std::numeric_limits<double>::epsilon();
}

// Runner Construction tests
SCENARIO("A Runner object is to be Constructed", "[Runner]") {
  GIVEN("A Runner object is constructed") {
    THEN("It is constructed without any exception/errors") {
      REQUIRE_NOTHROW(Runner());
    }
  }

  GIVEN("A Runner object is constructed") {
    Runner runner;
    THEN("Its member variables are correctly instantiated") {
      REQUIRE(runner.MissionCoordinatesLoaded() == false);
      REQUIRE(runner.GPSReady() == false);
      REQUIRE(runner.GetCurrentGPSCoordinate() == GPSCoordinate(0.0, 0.0));
      REQUIRE(runner.GetStartingGPSCoordinate() == GPSCoordinate(0.0, 0.0));
    }
  }
}

// Runner usage tests
SCENARIO("A Runner object is to be used", "[Runner]") {
  GIVEN("A Runner object") {
    Runner runner;

    WHEN("An invalid file path is loaded") {
      const bool ret = runner.LoadCoordinateFile("");
      THEN("The file fails to load") {
        REQUIRE(ret == false);
        REQUIRE(runner.MissionCoordinatesLoaded() == false);
      }
    }

    WHEN("A valid file path is loaded") {
      // TODO: Pass test file as a parameter
      const bool ret = runner.LoadCoordinateFile("/home/bryant/robogames_2024/ros_workspace/src/runner/test/good_file.txt");
      THEN("The file successfully loads") {
        REQUIRE(ret == true);
        REQUIRE(runner.MissionCoordinatesLoaded() == true);

        /*
         * Once coordinates are successfully loaded expecting the starting GPS
         * coordinate to also be set.
         */
        REQUIRE(runner.GetStartingGPSCoordinate() == GPSCoordinate(-0.3, -2.2));
      }
    }

    WHEN("A new SensorStates message is received") {
      arduino_connector::SensorStates::Ptr msg_ptr(
          new arduino_connector::SensorStates());
      msg_ptr->latitude = 3.2;
      msg_ptr->longitude = 2.0;

      runner.HandleNewSensorStates(msg_ptr);

      THEN("The current GPS coordinate is updated appropriately") {
        const GPSCoordinate &current_coord = runner.GetCurrentGPSCoordinate();
        REQUIRE(current_coord == GPSCoordinate(3.2, 2.0));
      }
    }

    WHEN("A coordinate is transformed with no file loaded") {
      MapCoordinate transformed;
      const bool res = runner.ConvertGPSToMap(GPSCoordinate(0.0, 0.0),
          &transformed);
      THEN("The conversion fails due to no starting coordinate present") {
        REQUIRE(res == false);
      }
    }
  }
}
