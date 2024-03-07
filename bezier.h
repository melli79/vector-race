//
// Created by Melchior Grützmann on 2024-03-04.
//

#ifndef BEZIER_H
#define BEZIER_H

#include "geometry.h"
#include <vector>

/**
 * \brief Bézier curve through Points.
 * \param ps Supporting Points of the Bézier curve
 * \param t position parameter (0 -> ps[0], 1 -> ps.last)
 * \return the interpolated Point on the curve.
 */
Point bezier(const std::vector<Point>& ps, double t=0.5);

struct Position {
    double t, d;
};

inline int sgn(double x) {
    if (x<0)
        return -1;
    if (x>0)
        return 1;
    return 0;
}

Position findPosition(std::vector<Point> const& route, Point const& p);

double computeLength(std::vector<Point> const& route, double end);

#endif //BEZIER_H
