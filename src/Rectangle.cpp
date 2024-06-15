#include "Rectangle.h"
#include <cmath>

Rectangle::Rectangle(double x, double y, double w, double h, void* data)
    : x(x), y(y), w(w), h(h), left(x - w / 2), right(x + w / 2), top(y - h / 2), bottom(y + h / 2) {}

bool Rectangle::contains(const Point& point) const {
    return left <= point.x && point.x <= right && top <= point.y && point.y <= bottom;
}

bool Rectangle::intersects(const Rectangle& range) const {
    return !(right < range.left || range.right < left || bottom < range.top || range.bottom < top);
}

Rectangle Rectangle::subdivide(const std::string& quadrant) const {
    if (quadrant == "ne") {
        return Rectangle(x + w / 4, y - h / 4, w / 2, h / 2);
    } else if (quadrant == "nw") {
        return Rectangle(x - w / 4, y - h / 4, w / 2, h / 2);
    } else if (quadrant == "se") {
        return Rectangle(x + w / 4, y + h / 4, w / 2, h / 2);
    } else if (quadrant == "sw") {
        return Rectangle(x - w / 4, y + h / 4, w / 2, h / 2);
    }
    // Return an empty rectangle if quadrant is invalid
    return Rectangle(0.0, 0.0, 0.0, 0.0);
}

double Rectangle::xDistanceFrom(const Point& point) const {
    if (left <= point.x && point.x <= right) {
        return 0.0;
    }
    return std::min(std::abs(point.x - left), std::abs(point.x - right));
}

double Rectangle::yDistanceFrom(const Point& point) const {
    if (top <= point.y && point.y <= bottom) {
        return 0.0;
    }
    return std::min(std::abs(point.y - top), std::abs(point.y - bottom));
}

double Rectangle::sqDistanceFrom(const Point& point) const {
    const double dx = xDistanceFrom(point);
    const double dy = yDistanceFrom(point);
    return dx * dx + dy * dy;
}

double Rectangle::distanceFrom(const Point& point) const {
    return std::sqrt(sqDistanceFrom(point));
}
