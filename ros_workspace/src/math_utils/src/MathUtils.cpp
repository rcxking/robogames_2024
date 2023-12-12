/*
 * MathUtils.cpp
 *
 * Implementation of extra math functions.
 *
 * Bryant Pong
 * 12/11/23
 */
#include <cmath>

double Hav(const double theta) {
	const double half_hav = sin(theta/2);
	return half_hav * half_hav;
}

double InvHav(const double theta) {
	return 2 * asin(sqrt(theta));
}
