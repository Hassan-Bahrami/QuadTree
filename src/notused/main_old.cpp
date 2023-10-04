#include <igl/boundary_loop.h>
#include <igl/harmonic.h>
#include <igl/map_vertices_to_circle.h>
#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>


int main(int argc, char* argv[])
{
    // Define the number of vertices in each dimension
    int num_vertices_x = 5;
    int num_vertices_y = 5;

    // Create a rectangular grid
    Eigen::MatrixXd V(num_vertices_x * num_vertices_y, 3);
    Eigen::MatrixXi F((num_vertices_x - 1) * (num_vertices_y - 1) * 2, 3);

    // Fill V with the vertex positions
    for (int y = 0; y < num_vertices_y; y++)
    {
        for (int x = 0; x < num_vertices_x; x++)
        {
            int vertex_index = y * num_vertices_x + x;
            V(vertex_index, 0) = x;
            V(vertex_index, 1) = y;
            V(vertex_index, 2) = 0.0; // Set z-coordinate to 0 for a 2D mesh
        }
    }

    // Fill F with the face indices (assuming a regular grid)
    for (int y = 0; y < num_vertices_y - 1; y++)
    {
        for (int x = 0; x < num_vertices_x - 1; x++)
        {
            int face_index = (y * (num_vertices_x - 1) + x) * 2;
            F(face_index, 0) = y * num_vertices_x + x;
            F(face_index, 1) = y * num_vertices_x + x + 1;
            F(face_index, 2) = (y + 1) * num_vertices_x + x + 1;

            F(face_index + 1, 0) = y * num_vertices_x + x;
            F(face_index + 1, 1) = (y + 1) * num_vertices_x + x + 1;
            F(face_index + 1, 2) = (y + 1) * num_vertices_x + x;
        }
    }

    // Fix the four corners of the rectangle
    V.row(0) = Eigen::Vector3d(0.0, 0.0, 0.0);
    V.row(num_vertices_x - 1) = Eigen::Vector3d(num_vertices_x - 1, 0.0, 0.0);
    V.row((num_vertices_y - 1) * num_vertices_x) = Eigen::Vector3d(0.0, num_vertices_y - 1, 0.0);
    V.row(num_vertices_x * num_vertices_y - 1) = Eigen::Vector3d(num_vertices_x - 1, num_vertices_y - 1, 0.0);

    // Apply the ARAP deformation by pulling up the center vertex (V(2, 2))
    int center_vertex_index = 2 * num_vertices_x + 2;
    V.row(center_vertex_index) = Eigen::Vector3d(2.0, 2.0, 1.0); // Pulling up the center vertex in the z-direction

    // Set up points for visualization
    Eigen::MatrixXd fixed_points(4, 3);
    fixed_points << V.row(0), V.row(num_vertices_x - 1), V.row((num_vertices_y - 1) * num_vertices_x), V.row(num_vertices_x * num_vertices_y - 1);

    // Set the positions of the points (corners) in red (RGB: 1, 0, 0)
    Eigen::MatrixXd point_colors(4, 3);
    point_colors << Eigen::RowVector3d(1.0, 0.0, 0.0),
                    Eigen::RowVector3d(1.0, 0.0, 0.0),
                    Eigen::RowVector3d(1.0, 0.0, 0.0),
                    Eigen::RowVector3d(1.0, 0.0, 0.0);

    // Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().add_points(fixed_points, point_colors); // Add points at the fixed corners
    viewer.launch();

    return 0;
}


// #include <Eigen/Core>
// #include <Eigen/Dense>
// #include <Eigen/Sparse>
// #include <Eigen/Geometry>
// #include <igl/adjacency_list.h>
// #include <igl/cotmatrix.h>
// #include <igl/boundary_loop.h>
// #include <igl/min_quad_with_fixed.h>
// #include <igl/opengl/glfw/Viewer.h>
// #include "MyARAP.cpp"
// #include <iostream>
// #include <vector>

// // ARAPMeshes class definition (same as before)
// // ... (Refer to the ARAPMeshes class implementation provided above)

// int main(int argc, char *argv[])
// {
//     // Define the number of vertices in each dimension
//     int num_vertices_x = 5;
//     int num_vertices_y = 5;

//     // Create a rectangular grid
//     Eigen::MatrixXd V(num_vertices_x * num_vertices_y, 3);
//     Eigen::MatrixXi F((num_vertices_x - 1) * (num_vertices_y - 1) * 2, 3);

//     // Fill V with the vertex positions
//     for (int y = 0; y < num_vertices_y; y++)
//     {
//         for (int x = 0; x < num_vertices_x; x++)
//         {
//             int vertex_index = y * num_vertices_x + x;
//             V(vertex_index, 0) = x;
//             V(vertex_index, 1) = y;
//             V(vertex_index, 2) = 0.0; // Set z-coordinate to 0 for a 2D mesh
//         }
//     }

//     // Fill F with the face indices (assuming a regular grid)
//     for (int y = 0; y < num_vertices_y - 1; y++)
//     {
//         for (int x = 0; x < num_vertices_x - 1; x++)
//         {
//             int face_index = (y * (num_vertices_x - 1) + x) * 2;
//             F(face_index, 0) = y * num_vertices_x + x;
//             F(face_index, 1) = y * num_vertices_x + x + 1;
//             F(face_index, 2) = (y + 1) * num_vertices_x + x + 1;

//             F(face_index + 1, 0) = y * num_vertices_x + x;
//             F(face_index + 1, 1) = (y + 1) * num_vertices_x + x + 1;
//             F(face_index + 1, 2) = (y + 1) * num_vertices_x + x;
//         }
//     }

//     // Fix the four corners of the rectangle
//     V.row(0) = Eigen::Vector3d(0.0, 0.0, 0.0);
//     V.row(num_vertices_x - 1) = Eigen::Vector3d(num_vertices_x - 1, 0.0, 0.0);
//     V.row((num_vertices_y - 1) * num_vertices_x) = Eigen::Vector3d(0.0, num_vertices_y - 1, 0.0);
//     V.row(num_vertices_x * num_vertices_y - 1) = Eigen::Vector3d(num_vertices_x - 1, num_vertices_y - 1, 0.0);

//     // Apply deformation by pulling up the center vertex (V(2, 2))
//     int center_vertex_index = 2 * num_vertices_x + 2;
//     V.row(center_vertex_index) = Eigen::Vector3d(2.0, 2.0, 1.0); // Pulling up the center vertex in the z-direction

//     // // Create an instance of ARAPMeshes and pass the mesh data
//     // ARAPMeshes arap_mesh(V, F);

//     // // Solve for the deformation
//     // std::vector<int> handle_verts = {center_vertex_index}; // The center vertex is the handle vertex
//     // Eigen::MatrixXd handle_verts_pos(1, 3);
//     // handle_verts_pos.row(0) = V.row(center_vertex_index);
//     // Eigen::MatrixXd deformed_verts = arap_mesh.solve({}, handle_verts, handle_verts_pos, 0, 10); // 10 iterations

//     // Visualization using igl::viewer::Viewer
//     igl::opengl::glfw::Viewer viewer;
//     // viewer.data().set_mesh(deformed_verts, F);

//     // Set colors for the fixed and handle vertices
//     Eigen::MatrixXd colors(V.rows(), 3);
//     colors.row(0) << 1, 0, 0;                      // Fixed vertex 1 (red)
//     colors.row(num_vertices_x - 1) << 1, 0, 0;     // Fixed vertex 2 (red)
//     colors.row((num_vertices_y - 1) * num_vertices_x) << 1, 0, 0;                     // Fixed vertex 3 (red)
//     colors.row(num_vertices_x * num_vertices_y - 1) << 1, 0, 0; // Fixed vertex 4 (red)
//     colors.row(center_vertex_index) << 0, 1, 0;                       // Handle vertex (green)
//     viewer.data().set_colors(colors);

//     viewer.launch();

//     return 0;
// }
