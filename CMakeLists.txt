cmake_minimum_required(VERSION 3.10)

# Project name and version
project(QuadTree)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable
add_executable(QuadTree
    src/main.cpp
    src/QuadTree.cpp
    src/Point.cpp
    src/Rectangle.cpp
    src/Circle.cpp
)

# Include directory for the headers
target_include_directories(QuadTree PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

