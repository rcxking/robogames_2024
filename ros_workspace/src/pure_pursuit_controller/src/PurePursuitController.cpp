/*
 * PurePursuit.cpp
 *
 *  Created on: Jan 4, 2024
 *      Author: bryant
 */
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <pure_pursuit_controller/PurePursuitController.h>
#include <ros/ros.h>

#include <cmath>

PurePursuitController::PurePursuitController() {
}

bool PurePursuitController::FindCircleLineIntersections(
																	 const geometry_msgs::Point &pt1,
			                             const geometry_msgs::Point &pt2,
																	 const double r,
																	 geometry_msgs::Point *lookahead_pt) const {
	bool lookahead_found = false;

	// nullptr checks
	if (lookahead_pt != nullptr) {
		// Extract (x, y) from each point
		const double x1 = pt1.x;
		const double x2 = pt2.x;
		const double y1 = pt1.y;
		const double y2 = pt2.y;

		// X/Y differences
		const double dx = x2 - x1;
		const double dy = y2 - y1;
		const double dr2 = (dx*dx) + (dy*dy);

		// Determinant of pt1 and pt2
		const double det = (x1*y2) - (x2*y1);

		// Discriminant
		const double discrim = (r*r*dr2) - (det*det);

		/*
		 * The discriminant tells how many intersections there are.
		 *
		 *        | < 0: 0 intersections
		 * If det | = 0: 1 intersection (tangent)
		 *        | > 0: 2 intersections
		 */
		if (discrim >= 0.0) {
			// Apply restrictions to return the point within the current segment

			// Dot products
			const double d1 = (x1*x1) + (y1*y1);
			const double d2 = (x2*x2) + (y2*y2);
			const double dd = d2 - d1;

			const double sqrt_term = std::sqrt(discrim);
			lookahead_pt->x = ((det*dy) + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
			lookahead_pt->y = (-det * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
			lookahead_found = true;
		} else {
			ROS_INFO("%s:%d: Determinant negative; no intersections", __FUNCTION__,
					__LINE__);
		}
	} else {
		ROS_ERROR("%s:%d: ERROR: lookahead_pt is NULL", __FUNCTION__, __LINE__);
	}
	return lookahead_found;
}

geometry_msgs::Twist PurePursuitController::ComputeVelocityCommands(
															const geometry_msgs::Pose &pose,
															const geometry_msgs::Twist &velocity) const {

	// Set the final velocity command
	geometry_msgs::Twist cmd_vel;
	return cmd_vel;
}



