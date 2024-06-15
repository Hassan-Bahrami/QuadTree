#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include "QuadTree.h"

// Function to generate random points
std::vector<Point> generateRandomPoints(int numPoints, int minX, int maxX, int minY, int maxY) {
    std::vector<Point> points;
    points.reserve(numPoints);
    for (int i = 0; i < numPoints; ++i) {
        int x = minX + rand() % (maxX - minX + 1);
        int y = minY + rand() % (maxY - minY + 1);
        points.emplace_back(x, y);
    }
    return points;
}

double calculateDistance(const Point& p1, const Point& p2) {
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

int main() {
    srand(static_cast<unsigned>(time(0))); // Seed random number generator

    // Create a boundary rectangle for the QuadTree
    Rectangle boundary(0, 0, 1000, 1000);

    // Create a QuadTree with a capacity of 4
    QuadTree quadtree(boundary, 4);

    // Generate random points
    int numPoints = 1000;
    std::vector<Point> randomPoints = generateRandomPoints(numPoints, -500, 500, -500, 500);

    // Insert random points into the QuadTree
    for (const Point& point : randomPoints) {
        quadtree.insert(point);
    }

    // Define the search point
    Point searchPoint(30, 30);

    // Specify the maximum number of nearest neighbors to find
    int maxCount = 5;

    // Specify the maximum distance within which to find neighbors
    double maxDistance = 100;

    // Find the k-nearest neighbors
    std::vector<Point> nearestNeighbors = quadtree.kNearestWrapper(searchPoint, maxCount, maxDistance);

    // Print the results with distances
    std::cout << "Closest points to (" << searchPoint.x << ", " << searchPoint.y << "):\n";
    for (const Point& neighbor : nearestNeighbors) {
        double distance = calculateDistance(searchPoint, neighbor);
        std::cout << "(" << neighbor.x << ", " << neighbor.y << ") - Distance: " << distance << "\n";
    }

    return 0;
}
