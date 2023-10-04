#include "QuadTree/Point.h"
#include <cmath>

Point::Point(double x, double y, void* data) : x(x), y(y), userData(data) {}

double Point::sqDistanceFrom(const Point& other) const {
    const double dx = other.x - x;
    const double dy = other.y - y;
    return dx * dx + dy * dy;
}

double Point::distanceFrom(const Point& other) const {
    return std::sqrt(sqDistanceFrom(other));
}
