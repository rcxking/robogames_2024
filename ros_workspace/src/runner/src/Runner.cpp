/*
 * Runner.cpp
 *
 * Class implementation of the object that executes the state machine for the
 * RoboMagellan competition.
 *
 * Bryant Pong
 * 2/13/24
 */
#include <arduino_connector/SensorStates.h>
#include <coordinate_parser/CoordinateParser.h>
#include <ros/ros.h>
#include <runner/Runner.h>

#include <cstdio>
#include <string>

bool Runner::LoadCoordinateFile(const std::string &filepath) {
  ROS_INFO("%s:%d: Attempting to load coordinates from: %s", __FUNCTION__,
      __LINE__, filepath.c_str());

  const bool success = cp_.LoadCoordinateFile(filepath);
  if (success) {
    ROS_INFO("%s:%d: Successfully loaded coordinates from: %s", __FUNCTION__,
        __LINE__, filepath.c_str());
    // Also set starting GPS coordinate
    if (cp_.GetStartPoint(&starting_gps_coordinate_)) {
      mission_coords_loaded_ = true;
    } else {
      ROS_ERROR("%s:%d: ERROR: Unable to get starting point", __FUNCTION__,
          __LINE__);
    }
  } else {
    ROS_ERROR("%s:%d: ERROR: Failed to load coordinates from: %s", __FUNCTION__,
        __LINE__, filepath.c_str());
    mission_coords_loaded_ = false;
  }
  return success;
}

bool Runner::LoadCoordinateFileFromParam() {
  bool success = false;

  std::string filepath;
  if (nh_.getParam("filepath", filepath)) {
    ROS_INFO("%s:%d: Will load mission coordinates from: %s", __FUNCTION__,
        __LINE__, filepath.c_str());

    // Parse mission coordinates
    if (cp_.LoadCoordinateFile(filepath)) {
      ROS_INFO("%s:%d: Successfully loaded coordinates from: %s", __FUNCTION__,
          __LINE__, filepath.c_str());
      success = true;
    } else {
      ROS_ERROR("%s:%d: ERROR: Unable to load coordinates from: %s",
          __FUNCTION__, __LINE__, filepath.c_str());
    }
  } else {
    ROS_ERROR("%s:%d: ERROR: filepath param does not exist!", __FUNCTION__,
        __LINE__);
  }
  return success;
}

bool Runner::ConfirmMissionCoordinates() const {
  // Ensure mission coordinates have been successfully loaded
  if (!cp_.AreWaypointsSet()) {
    ROS_ERROR("%s:%d: ERROR: No mission coordinates loaded!", __FUNCTION__,
        __LINE__);
    return false;
  }

  // Display mission coordinates and ask for user confirmation
  cp_.DisplayMissionCoordinates();
  ROS_INFO("%s:%d: If coordinates are correct, enter \'Y\' to proceed",
      __FUNCTION__, __LINE__);

  char input;
  scanf("%s", &input);
  return (input == 'Y');
}

void Runner::HandleNewSensorStates(
    const arduino_connector::SensorStates::ConstPtr &msg) {
  // Update the current latitude/longitude
  current_gps_coordinate_.latitude = msg->latitude;
  current_gps_coordinate_.longitude = msg->longitude;
}
