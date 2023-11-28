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

#include <cstring>
#include <iostream>
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
				std::cerr << "ERROR: Could not cast either: " << latitude << " or "
						<< longitude << " into doubles" << std::endl;
				return false;
			}

			// Store next GPS coordinate
			tmp.push_back(GPSCoordinate(latitude_doub, longitude_doub));
		}
		input_file.close();

		// Ensure at least 2 waypoints (start and end) are specified
		if (tmp.size() < 2) {
			std::cerr << "ERROR: Less than 2 waypoints specified" << std::endl;
			return false;
		} else {
			waypoints_ = tmp;
			return true;
		}
	} else {
		std::cerr << "ERROR: Unable to open file: " << filepath.c_str() << std::endl;
		return false;
	}
}


