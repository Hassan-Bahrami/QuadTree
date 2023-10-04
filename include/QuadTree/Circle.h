#pragma once
#include "Point.h"
#include "Rectangle.h"

class Circle {
public:
    double x;
    double y;
    double r;
    double rSquared;

    Circle(double x, double y, double r);
    bool contains(const Point& point) const;
    bool intersects(const Rectangle& range) const;
};
