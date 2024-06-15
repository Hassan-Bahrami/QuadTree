#ifndef QUADTREE_H
#define QUADTREE_H

#include "Point.h"
#include "Rectangle.h"
#include "Circle.h"
#include <vector>
#include <algorithm>
#include <cmath>

class QuadTree {
public:
    QuadTree(const Rectangle& boundary, int capacity = DEFAULT_CAPACITY, int depth = 0);

    void clear();
    bool insert(const Point& point);
    std::vector<Point> query(const Rectangle& range) const;
    void deleteInRange(const Rectangle& range);
    std::vector<Point> closest(const Point& searchPoint, int maxCount = 1, double maxDistance = INFINITY) const;
    void forEach(void (*fn)(const Point&));
    QuadTree filter(bool (*fn)(const Point&)) const;
    QuadTree merge(const QuadTree& other, int capacity) const;
    
    std::vector<Point> kNearestWrapper(const Point& searchPoint, int maxCount, double maxDistance) const {
        double sqMaxDistance = maxDistance * maxDistance;
        double furthestSqDistance = INFINITY;
        std::vector<Point> found;
        kNearest(searchPoint, maxCount, sqMaxDistance, furthestSqDistance, found);
        return found;
    }

    int length() const;

private:
    static const int DEFAULT_CAPACITY = 8;
    static const int MAX_DEPTH = 8;

    Rectangle boundary;
    int capacity;
    std::vector<Point> points;
    bool divided;
    int depth;

    std::vector<QuadTree> children;

    void subdivide();
    bool contains(const Point& point) const;
    bool intersects(const Rectangle& range) const;
    double sqDistanceFrom(const Point& point) const;
    double distanceFrom(const Point& point) const;
    void kNearest(const Point& searchPoint, int maxCount, double sqMaxDistance, double& furthestSqDistance, std::vector<Point>& found) const;
};

#endif // QUADTREE_H
