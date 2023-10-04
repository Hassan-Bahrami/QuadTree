#pragma once

class Point {
public:
    double x;
    double y;
    void* userData; // Assuming you will use void* for arbitrary user data

    Point(double x, double y, void* data = nullptr);
    double sqDistanceFrom(const Point& other) const;
    double distanceFrom(const Point& other) const;
};