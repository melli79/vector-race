//
// Created by Melchior Grützmann on 2024-03-04.
//

#include "geometry.h"

Point Point::translate(const Vector& v) const {
    return { x + v.x,  y + v.y };
}

Vector Point::operator -(const Point& p) const {
    return { x-p.x, y-p.y };
}

Point Point::interpolate(double t, const Point& p) const {
    return Point(*this).translate((p-*this)*t);
}

Point Point::scale(double f) const {
    return ORIGIN.translate((*this-ORIGIN)*f);
}

std::vector<Point> Circle::intersect(const Circle& c2) const {
    Vector v = c2.center - center;
    double d = sqrt(v.norm2());
    if (d < epsilon || d < abs(radius - c2.radius) || radius+c2.radius < d)
        return {};
    double l = (sqr(d) + sqr(radius) -sqr(c2.radius)) / d;
    double h2 = sqr(radius) -sqr(l);
    if (h2 < 3*epsilon*(radius*abs(l)+1.0)) {
        auto p = Point(center);
        return { p.translate(v.normed()*l) };
    }
    const auto e = v*(1/d);
    const auto b = center.translate(e*l);
    const auto p = e.perp() * sqrt(h2);
    return { b.translate(p), b.translate(-p) };
}
