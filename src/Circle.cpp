#include "Circle.h"
#include <cmath>

Circle::Circle(double x, double y, double r) : x(x), y(y), r(r) {
    rSquared = r * r;
}

bool Circle::contains(const Point& point) const {
    double d = (point.x - x) * (point.x - x) + (point.y - y) * (point.y - y);
    return d <= rSquared;
}

bool Circle::intersects(const Rectangle& range) const {
    double xDist = std::abs(range.x - x);
    double yDist = std::abs(range.y - y);
    double w = range.w / 2;
    double h = range.h / 2;

    double edges = std::pow((xDist - w), 2) + std::pow((yDist - h), 2);

    if (xDist > (r + w) || yDist > (r + h)) {
        return false;
    }

    if (xDist <= w || yDist <= h) {
        return true;
    }

    return edges <= rSquared;
}
