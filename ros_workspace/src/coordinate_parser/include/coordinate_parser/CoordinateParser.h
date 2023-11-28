#ifndef __COORDINATE_PARSER_H__
#define __COORDINATE_PARSER_H__

/*
 * CoordinateParser.h
 *
 * Class declaration of an object that can parse GPS coordinates from an input
 * text file.
 *
 * Bryant Pong
 * 11/25/23
 */
#include <cmath>
#include <limits>
#include <string>
#include <vector>

//! Stores a GPS Coordinate (latitude/longitude)
class GPSCoordinate {
public:
	//! Default Constructor
	GPSCoordinate() :
		latitude(0.0),
		longitude(0.0) {}

	//! Constructor taking in latitude/longitude (degrees)
	GPSCoordinate(const double latit, const double longi) :
		latitude(latit),
		longitude(longi) {}

	/**
	 * @brief Checks if another GPSCoordinate is the same as this one.
	 * @param other Second GPSCoordinate to compare with this one.
	 * @return bool True if both latitude/longitude match; false if not.
	 */
	bool operator==(const GPSCoordinate &other) const {
		const bool lat_match = std::fabs(other.latitude - latitude) <=
				std::numeric_limits<double>::epsilon();
		const bool long_match = std::fabs(other.longitude - longitude) <=
				std::numeric_limits<double>::epsilon();
		return lat_match && long_match;
	}

	//! Latitude/Longitude (degrees)
	double latitude;
	double longitude;
};

class CoordinateParser {
public:
	//! Default Constructor
	CoordinateParser();

	/**
	 * @brief Attempts to load and parse the coordinates in the specified file.
	 * @param filepath String containing the full file path to load.
	 * @return bool True if file loaded successfully and waypoints_ was modified;
	 * false on failure.
	 */
	bool LoadCoordinateFile(const std::string& filepath);

	/**
	 * @brief Checks if waypoints are set.
	 * @return bool True if waypoints are set; false if not.
	 */
	bool AreWaypointsSet() const { return !waypoints_.empty(); }

	/**
	 * @brief Gets the starting point of the mission.
	 * @param start On success this pointer will be updated with mission's start.
	 * @return bool True if the starting point was acquired; false on error.
	 */
	bool GetStartPoint(GPSCoordinate *start) const;

	/**
	 * @brief Gets the ending point of the mission.
	 * @param end On success this pointer will be updated with mission's end.
	 * @return bool True if the ending point was acquired; false on error.
	 */
	bool GetEndPoint(GPSCoordinate *end) const;

	/**
	 * @brief Displays the current mission waypoints.
	 */
	void DisplayMissionCoordinates() const;

	// Accessors
	std::vector<GPSCoordinate> GetWaypoints() const { return waypoints_; }

private:
	//! Stores the parsed GPS coordinates in the order to visit
	std::vector<GPSCoordinate> waypoints_;
};
#endif
