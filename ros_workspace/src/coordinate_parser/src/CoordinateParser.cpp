/*
 * CoordinateParser.cpp
 *
 * Class implementation of an object that can parse GPS coordinates from an
 * input text file.
 *
 * Bryant Pong
 * 11/25/23
 */
#include <coordinate_parser/CoordinateParser.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

CoordinateParser::CoordinateParser() {}

bool CoordinateParser::LoadCoordinateFile(const std::string& filepath) {
	// Attempt to open the file
	std::ifstream input_file(filepath);
	if (input_file.is_open()) {
		// Temporary GPS coordinate storage to ensure all coordinates are valid
		std::vector<GPSCoordinate> tmp;

		// Parse the next lines
		std::string next_line;
		while (getline(input_file, next_line)) {
			// Expecting each line to be of the form "lat long"
			std::stringstream ss(next_line);
			std::string latitude, longitude;
			ss >> latitude;
			ss >> longitude;

			// Convert the next latitude and longitude into doubles
			double latitude_doub, longitude_doub;
			try {
				latitude_doub = std::stod(latitude);
				longitude_doub = std::stod(longitude);
			} catch (std::invalid_argument& ex) {
				fprintf(stderr, "%s:%d: ERROR: Could not cast either %s or %s into "
						"doubles\n", __FUNCTION__, __LINE__, latitude.c_str(),
						longitude.c_str());
				return false;
			}

			// Store next GPS coordinate
			tmp.push_back(GPSCoordinate(latitude_doub, longitude_doub));
		}
		input_file.close();

		// Ensure at least 2 waypoints (start and end) are specified
		if (tmp.size() < 2) {
			fprintf(stderr, "%s:%d: ERROR: Less than 2 waypoints specified\n",
					__FUNCTION__, __LINE__);
			return false;
		} else {
			waypoints_ = tmp;
			return true;
		}
	} else {
		fprintf(stderr, "%s:%d: ERROR: Unable to open file %s\n", __FUNCTION__,
				__LINE__, filepath.c_str());
		return false;
	}
}

bool CoordinateParser::GetStartPoint(GPSCoordinate *start) const {
	bool succ = false;

	// nullptr check
	if (start != nullptr) {
		// Ensure waypoints are set
		if (AreWaypointsSet()) {
			// The first waypoint is the starting point
			*start = waypoints_[0];
			succ = true;
		} else {
			fprintf(stderr, "%s:%d: ERROR: waypoints not set\n", __FUNCTION__,
					__LINE__);
		}
	} else {
		fprintf(stderr, "%s:%d: ERROR: start is NULL\n", __FUNCTION__, __LINE__);
	}
	return succ;
}

bool CoordinateParser::GetEndPoint(GPSCoordinate *end) const {
	bool succ = false;

	// nullptr check
	if (end != nullptr) {
		// Ensure waypoints are set
		if (AreWaypointsSet()) {
			// The last waypoint is the ending point
			*end = waypoints_[waypoints_.size() - 1];
			succ = true;
		} else {
			fprintf(stderr, "%s:%d: ERROR: waypoints not set\n", __FUNCTION__,
					__LINE__);
		}
	} else {
		fprintf(stderr, "%s:%d: ERROR: end is NULL\n", __FUNCTION__, __LINE__);
	}
	return succ;
}
