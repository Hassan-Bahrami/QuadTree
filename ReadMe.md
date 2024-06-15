# QuadTree Project
This project implements a QuadTree data structure in C++ for spatial indexing and nearest neighbor search.

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)

## Introduction

A QuadTree is a tree data structure used to partition a 2D space into regions (quadrants), recursively subdividing it into four quadrants or nodes until each node contains zero or a specified number of points. It is particularly useful for spatial indexing, collision detection, and nearest neighbor search.

This project provides a basic implementation of a QuadTree with the following functionalities:
- Insertion of points
- Querying points within a range
- Deleting points within a range
- Finding k-nearest neighbors to a given point

## Features

- **Insertion**: Insert points into the QuadTree structure.
- **Query**: Retrieve points within a specified rectangular range.
- **Deletion**: Remove points within a specified rectangular range.
- **Nearest Neighbor Search**: Find k-nearest neighbors to a given point.

## Dependencies

This project depends on:
- C++17 compiler
- CMake (for building the project)

## Installation

1. Clone the repository:
   ```sh
   git clone git@github.com:Hassan-Bahrami/QuadTree.git
   cd QuadTree
   ```
2. Build the project using CMake:
  ```sh
  mkdir build
  cd build
  cmake ..
  make
  ```
## Usage
To use the QuadTree in your C++ project, include the necessary headers and link against the QuadTree library:
```sh
#include <iostream>
#include "QuadTree/QuadTree.h"

int main() {
    // Example usage
    // Create a boundary rectangle for the QuadTree
    Rectangle boundary(0, 0, 100, 100);

    // Create a QuadTree with a capacity of 4
    QuadTree quadtree(boundary, 4);

    // Insert points
    quadtree.insert(Point(20, 0));
    quadtree.insert(Point(40, 0));
    quadtree.insert(Point(-30, 0));
    quadtree.insert(Point(-40, 0));

    // Define the search point
    Point searchPoint(30, 30);

    // Find the k-nearest neighbors
    std::vector<Point> nearestNeighbors = quadtree.kNearestWrapper(searchPoint, 2, 40);

    // Print the results
    std::cout << "Closest points to (" << searchPoint.x << ", " << searchPoint.y << "):\n";
    for (const Point& neighbor : nearestNeighbors) {
        std::cout << "(" << neighbor.x << ", " << neighbor.y << ")\n";
    }

    return 0;
}
```
