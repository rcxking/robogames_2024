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

	// Accessors
	std::vector<GPSCoordinate> GetWaypoints() const { return waypoints_; }

private:
	//! Stores the parsed GPS coordinates in the order to visit
	std::vector<GPSCoordinate> waypoints_;
};
#endif
