#include <iostream>
#include <Eigen/Core>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <Eigen/Geometry> // Include for rotation
#include <igl/opengl/glfw/Viewer.h>

// int main(int argc, char *argv[])
// {
    // // Create a libigl Viewer
    // igl::opengl::glfw::Viewer viewer;
    // //  Load the mesh from the exported OBJ file
    // Eigen::MatrixXd Plane_V;  // Vertices
    // Eigen::MatrixXi Plane_F; // Faces
    // igl::readOBJ("/home/hassan/Projects/ISTA/SurfaceDevelopibility/models/curvedPlane.obj", Plane_V, Plane_F);
    // // Set the loaded mesh as the viewer's mesh
    // viewer.data().set_mesh(Plane_V, Plane_F);

    // // Find the bounding box of the mesh
    // Eigen::Vector3d minCoords = Plane_V.colwise().minCoeff();
    // Eigen::Vector3d maxCoords = Plane_V.colwise().maxCoeff();

    // // Print minCoords and maxCoords
    // std::cout << "minCoords: " << minCoords.transpose() << std::endl;
    // std::cout << "maxCoords: " << maxCoords.transpose() << std::endl;

    // // Define the number of rows and columns for the grid
    // int num_rows = 10;
    // int num_cols = 10;

    // // Compute the spacing between grid lines
    // double grid_spacing_x = (maxCoords.x() - minCoords.x()) / num_cols;
    // double grid_spacing_y = (maxCoords.z() - minCoords.z()) / num_rows;

    // // Generate grid vertices
    // Eigen::MatrixXd gridVertices((num_rows + 1) * (num_cols + 1), 3);
    // // Generate grid faces
    // Eigen::MatrixXi gridFaces(num_rows * num_cols, 4);

    // // Populate gridVertices with coordinates
    // for (int i = 0; i <= num_rows; i++) {
    //     for (int j = 0; j <= num_cols; j++) {
    //         double x = minCoords.x() + j * grid_spacing_x;
    //         double y = minCoords.z() + i * grid_spacing_y;
    //         double z = 0.0; // Set the z-coordinate as needed
    //         gridVertices.row(i * (num_cols + 1) + j) << x, z, y;
    //     }
    // }

    // // Print gridVertices
    // std::cout << "gridVertices:" << std::endl;
    // for (int i = 0; i < gridVertices.rows(); i++) {
    //     std::cout << gridVertices.row(i) << std::endl;
    // }

    // viewer.data().add_points(gridVertices, Eigen::RowVector3d(0.0, 1.0, 0.0));
    // // Define the point you want to test (change these coordinates)
    // Eigen::RowVector3d point_to_test(0.0, 0.0, 0.0);
    // Eigen::RowVector3d ray_origin = point_to_test;
    // viewer.data().add_points(point_to_test, Eigen::RowVector3d(1.0, 0.0, 0.0));

    // // Print the dimensions of matrix V
    // std::cout << "Dimensions of V: " << Plane_V.rows() << " x " << Plane_V.cols() << std::endl;

    // // Print the dimensions of matrix F
    // std::cout << "Dimensions of F: " << Plane_F.rows() << " x " << Plane_F.cols() << std::endl;

    // // Print the dimensions of matrix V and F
    // std::cout << "Grid Vertices: " << gridVertices.rows() << " x " << gridVertices.cols() << std::endl;
    // std::cout << "Grid Faces: " << gridFaces.rows() << " x " << gridFaces.cols() << std::endl;

    // viewer.launch();


    // igl::embree::EmbreeIntersector tree;
    // // Ensure that Plane_V and Plane_F have consistent types
    // Eigen::MatrixXf floatPlane_V = Plane_V.cast<float>();
    // Eigen::MatrixXf floatPlane_F = Plane_F.cast<float>();
    // tree.init(floatPlane_V, Plane_F);

    // Eigen::Vector3d origin(0.0, 0.0, 0.0);  // Replace with your arbitrary point
    // Eigen::Vector3d direction(1.0, 1.0, 0.0); // Replace with your desired direction

    // Eigen::Vector3f origin_f = origin.cast<float>(); // Cast origin to float
    // Eigen::Vector3f direction_f = direction.cast<float>(); // Cast direction to float

    // igl::Hit hit;
    // if (tree.intersectRay(origin_f, direction_f, hit))
    // {
    //     // The point is inside the mesh
    //     std::cout << "Point is inside. Intersection point: (" << hit.u << ", " << hit.v << ", " << hit.t << ")" << std::endl;
    // }
    // else
    // {
    //     // The point is outside the mesh
    //     std::cout << "Point is outside." << std::endl;
    // }
//     return 0;
// }

#include <iostream>
#include <vector>
#include "QuadTree/QuadTree.h" // Include your QuadTree header

int main() {
    // Create a boundary rectangle for the QuadTree
    Rectangle boundary(0, 0, 100, 100);

    // Create a QuadTree with a capacity of 4
    QuadTree quadtree(boundary, 4);

    // Insert some points into the QuadTree
    quadtree.insert(Point(20, 0));
    quadtree.insert(Point(40, 0));
    quadtree.insert(Point(-30, 0));
    quadtree.insert(Point(-40, 0));

    // Define the search point
    Point searchPoint(30, 30);

    // Specify the maximum number of nearest neighbors to find
    int maxCount = 2;

    // Specify the maximum distance within which to find neighbors
    double maxDistance = INFINITY; // To include all points

    // Find the k-nearest neighbors
    std::vector<Point> nearestNeighbors = quadtree.kNearestWrapper(searchPoint, maxCount, maxDistance);

    // Print the results
    std::cout << "Closest points to (" << searchPoint.x << ", " << searchPoint.y << "):\n";
    for (const Point& neighbor : nearestNeighbors) {
        std::cout << "(" << neighbor.x << ", " << neighbor.y << ")\n";
    }

    return 0;
}
