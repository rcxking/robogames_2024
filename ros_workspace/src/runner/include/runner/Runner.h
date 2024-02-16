#ifndef __RUNNER_H__
#define __RUNNER_H__

/*
 * Runner.h
 *
 * Class definition of the object that executes the state machine for the
 * RoboMagellan competition.
 *
 * Bryant Pong
 * 2/13/24
 */
#include <arduino_connector/SensorStates.h>
#include <coordinate_parser/CoordinateParser.h>
#include <ros/ros.h>

#include <string>

class MapCoordinate {
public:
  //! Default Constructor
  MapCoordinate() : x(0.0), y(0.0) {}

  //! Constructor taking in (x, y)
  MapCoordinate(const double this_x, const double this_y) :
    x(this_x), y(this_y) {}

  //! Coordinate point (x, y)
  double x, y;
};

class Runner {
public:
  //! Default Constructor
  Runner() : mission_coords_loaded_(false), gps_ready_(false), nh_("~") {
    sensor_states_sub_ = nh_.subscribe(
        "/arduino_connector/current_sensor_states", 100,
        &Runner::HandleNewSensorStates, this);
  }

  /**
   * @brief Loads the mission coordinates from the specified file.
   * @param filepath The full file path to the mission coordinates.
   * @return True if successfully loaded mission coordinates; false otherwise.
   */
  bool LoadCoordinateFile(const std::string &filepath);

  /**
   * @brief Loads the mission coordinates from the file specified by the
   * "filepath" parameter.
   * @return True if successfully loaded mission coordinates; false otherwise.
   */
  bool LoadCoordinateFileFromParam();

  /**
   * @brief Confirm with the user that the loaded mission coordinates are
   * correct.
   * @return True if user accepted mission coordinates; false otherwise.
   */
  bool ConfirmMissionCoordinates() const;

  /**
   * @brief Converts the provided GPS coordinate into a map coordinate (assumes
   * that the Earth is flat since the distances are short).
   */
  bool ConvertGPSToMap(const GPSCoordinate &gps_coord,
                       MapCoordinate *map_coord) const;

  /**
   * @brief Callback to update the current robot's state from the Arduino
   * Connector.
   * @param msg Incoming SensorStates message.
   */
  void HandleNewSensorStates(
      const arduino_connector::SensorStates::ConstPtr &msg);

  //! Accessors
  bool MissionCoordinatesLoaded() const { return mission_coords_loaded_; }
  bool GPSReady() const { return gps_ready_; }
  const GPSCoordinate &GetCurrentGPSCoordinate() const {
    return current_gps_coordinate_;
  }
  const GPSCoordinate &GetStartingGPSCoordinate() const {
    return starting_gps_coordinate_;
  }

private:
  //! Has mission coordinates been loaded?
  bool mission_coords_loaded_;

  //! Has a valid GPS location been accepted?
  bool gps_ready_;

  //! Current robot GPS location (latitude/longitude in degrees)
  GPSCoordinate current_gps_coordinate_;

  //! Starting robot GPS location (latitude/longitude in degrees)
  GPSCoordinate starting_gps_coordinate_;

  //! Object that loads and maintains the mission coordinates
  CoordinateParser cp_;

  //! Node Handle to acquire parameters and publish/subscribe to topics
  ros::NodeHandle nh_;

  //! Subscriber for SensorData
  ros::Subscriber sensor_states_sub_;
};
#endif
