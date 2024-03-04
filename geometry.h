//
// Created by Melchior Grützmann on 2024-03-04.
//

#ifndef POINT_H
#define POINT_H
#include <vector>

const double epsilon = 1e-16;
inline double sqr(double x) {
    return x*x;
}

struct Vector;

struct Point {
    double x, y;

    const static Point ORIGIN;

    bool operator <(const Point& p) const {
        return y<p.y || y==p.y&&x<p.x;
    }

    [[nodiscard]]
    Point translate(const Vector& v) const;

    [[nodiscard]]
    Vector operator -(const Point& p) const;

    [[nodiscard]]
    Point interpolate(double t, const Point& p) const;

    [[nodiscard]]
    Point scale(double f) const;
};


struct Vector {
    double x, y;
    const static Vector ZERO;

    Vector operator+(const Vector& v) const {
        return  { x+v.x, y+v.y };
    }

    Vector operator*(double f) const {
        return { x*f, y*f };
    }

    Vector& operator+=(const Vector& v) {
        x += v.x;  y+= v.y;
        return *this;
    }

    [[nodiscard]]
    double norm2() const {
        return x*x +y*y;
    }

    [[nodiscard]]
    Vector normed() const {
        double f = 1/sqrt(norm2());
        return { x*f, y*f };
    }

    [[nodiscard]]
    Vector perp() const {
        return { -y, x };
    }

    [[nodiscard]]
    Vector operator-() const {
        return { -x, -y };
    }

    [[nodiscard]]
    Vector rotate(double angle) const {
        return rotate(cos(angle), sin(angle));
    }

    [[nodiscard]]
    Vector rotate(double c, double s) const {
        return {c*x-s*y, s*x+c*y };
    }

    [[nodiscard]]
    double dot(Vector v) const {
        return x*v.x +y*v.y;
    }

    [[nodiscard]]
    Vector operator-(Vector const& v) const {
        return { x-v.x, y-v.y };
    }
};


struct Circle {
    Point center;
    double radius; // >0
    [[nodiscard]]
    std::vector<Point> intersect(const Circle& circle) const;
};


#endif //POINT_H
