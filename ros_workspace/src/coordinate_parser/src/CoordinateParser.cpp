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
#include <ros/ros.h>

#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

CoordinateParser::CoordinateParser() {
	is_sim_run_ = false;
}

bool CoordinateParser::LoadCoordinateFile(const std::string& filepath,
		                                      const bool is_sim) {
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
				ROS_ERROR("Could not cast either %s or %s into doubles",
						latitude.c_str(), longitude.c_str());
				return false;
			}

			// Store next GPS coordinate
			tmp.push_back(GPSCoordinate(latitude_doub, longitude_doub));
		}
		input_file.close();

		// Ensure at least 2 waypoints (start and end) are specified
		if (tmp.size() < 2) {
			ROS_ERROR("Less than 2 waypoints specified");
			return false;
		} else {
			waypoints_ = tmp;
			is_sim_run_ = is_sim;
			return true;
		}
	} else {
		ROS_ERROR("Unable to open file %s", filepath.c_str());
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
			ROS_ERROR("waypoints_ not set");
		}
	} else {
		ROS_ERROR("start is NULL");
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
			ROS_ERROR("waypoints_ not set");
		}
	} else {
		ROS_ERROR("end is NULL");
	}
	return succ;
}

void CoordinateParser::DisplayMissionCoordinates() const {
	// Ensure waypoints are set before displaying mission coordinates
	if (AreWaypointsSet()) {
		if (is_sim_run_) {
			ROS_INFO("This is a simulation run");
		} else {
			ROS_INFO("This is a real robot run");
		}
		ROS_INFO("I will be following these coordinates:");
		for (size_t i = 0; i < waypoints_.size(); ++i) {
			std::stringstream next_info;
			if (i == 0) {
				next_info << "Starting ";
			} else if (i == waypoints_.size() - 1) {
				next_info << "Ending ";
			} else {
				next_info << "Intermediate ";
			}

			if (is_sim_run_) {
				next_info << "Point (meters): (" << waypoints_[i].latitude << ", " <<
						waypoints_[i].longitude << ")";
			} else {
				next_info << "Point: Latitude (deg): " << waypoints_[i].latitude <<
						", Longitude (deg): " << waypoints_[i].longitude;
			}

			// Need to use temporary variables to avoid dangling pointer
			const std::string next_info_str = next_info.str();
			const char *next_info_c_str = next_info_str.c_str();
			ROS_INFO("%s", next_info_c_str);
		}
	} else {
		// ERROR: need to set waypoints before displaying mission
		ROS_ERROR("waypoints_ not set");
	}
}
