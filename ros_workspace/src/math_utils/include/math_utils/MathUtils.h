/*
 * MathUtils.h
 *
 * Extra math functions not defined in math.h.
 *
 * Bryant Pong
 * 12/11/23
 */

#ifndef INCLUDE_MATH_UTILS_MATHUTILS_H_
#define INCLUDE_MATH_UTILS_MATHUTILS_H_

namespace MathUtils {

/**
 * @brief Haversine function.  Name is shorthand for "half-versed sine".
 * @param theta Angle in RADIANS.
 * @return Haversine in RADIANS.
 */
double Hav(const double theta);

/**
 * @brief Inverse Haversine function.
 * @param theta Angle in RADIANS.
 * @return Inverse Haversine in RADIANS.
 */
double InvHav(const double theta);
}
#endif /* INCLUDE_MATH_UTILS_MATHUTILS_H_ */
