#pragma once

#include <string>
#include "Point.h"

class Rectangle {
public:
    double x;
    double y;
    double w;
    double h;
    double left;
    double right;
    double top;
    double bottom;

    Rectangle(double x, double y, double w, double h, void* data = nullptr);
    bool contains(const Point& point) const;
    bool intersects(const Rectangle& range) const;
    Rectangle subdivide(const std::string& quadrant) const;
    double xDistanceFrom(const Point& point) const;
    double yDistanceFrom(const Point& point) const;
    double sqDistanceFrom(const Point& point) const;
    double distanceFrom(const Point& point) const;
};
