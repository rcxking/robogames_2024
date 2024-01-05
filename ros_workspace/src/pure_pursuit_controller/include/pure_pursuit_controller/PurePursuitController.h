/*
 * PurePursuitController.h
 *
 * Class declaration for the Pure Pursuit Controller.
 *
 * Bryant Pong
 * 1/4/24
 */
#ifndef __PURE_PURSUIT_CONTROLLER_H__
#define __PURE_PURSUIT_CONTROLLER_H__

#include <geometry_msgs/Point.h>

class PurePursuitController {
public:
	//! Default Constructor
	PurePursuitController();

	/**
	 * @brief Finds the intersections between a circle and a line segment.
	 * The circle is assumed to be centered at the origin (0, 0).
	 * @param pt1 Starting point of the line
	 * @param pt2 Ending point of the line
	 * @param r radius of the circle
	 * @param lookahead_pt Intersection on the line segment towards pt2.  This gives the
	 * "lookahead point" to steer the robot.
	 * @return bool True if lookahead_pt was found; false if not.
	 */
	bool FindCircleLineIntersections(const geometry_msgs::Point &pt1,
			                             const geometry_msgs::Point &pt2,
																	 const double r,
																	 geometry_msgs::Point *lookahead_pt) const;

private:
};
#endif
