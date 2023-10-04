#include <unordered_set>
#include <set>
#include <queue>

#include "MotorCycleGraph/MotorCycle.h"

// Define a hash function for Vector4i
struct Vector4iHash {
    std::size_t operator()(const Eigen::Vector4i& vec) const {
        std::size_t seed = 0;
        for (int i = 0; i < 4; ++i) {
            seed ^= std::hash<int>()(vec[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};
struct Vector2iHash {
    std::size_t operator()(const Eigen::Vector2i& vec) const {
        std::size_t seed = 0;
        for (int i = 0; i < 2; ++i) {
            seed ^= std::hash<int>()(vec[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

bool MotorcycleGraph::quadContainsEdge(const Eigen::Vector4i& quad, const Eigen::Vector2i& edge) {
    for (int i = 0; i < 4; ++i) {
        int v1 = quad[i];
        int v2 = quad[(i + 1) % 4];

        if ((v1 == edge.x() && v2 == edge.y()) || (v2 == edge.x() && v1 == edge.y())) {
            return true;
        }
    }
    return false;
}


void MotorcycleGraph::MotorCycle_init(const std::vector<Eigen::Vector3d>& V,
                                    const std::vector<Eigen::Vector4i>& Q) {

    // Set boundaryVertices and boundaryEdges to empty vectors
    this->boundaryVertices.clear();
    this->boundaryEdges.clear();
    
    // Extract edges from faces
    for (const Eigen::Vector4i& quad : Q) {
        this->E.emplace_back(quad[0], quad[1]);
        this->E.emplace_back(quad[1], quad[2]);
        this->E.emplace_back(quad[2], quad[3]);
        this->E.emplace_back(quad[3], quad[0]);
    }

    // Initialize vertexFaceCount
    std::unordered_map<int, int> vertexFaceCount;

    // count number of incident faces to a vertex
    for (size_t i = 0; i < Q.size(); ++i) {
        const Eigen::Vector4i& quad = Q[i];
        for (int j = 0; j < 4; ++j) {
            int vertexId = quad[j];
            if (vertexFaceCount.find(vertexId) != vertexFaceCount.end()) {
                vertexFaceCount[vertexId]++;
            } else {
                vertexFaceCount[vertexId] = 1;
            }
        }
    }

    // Calculate the number of incident edges for each vertex
    for (auto& entry : vertexFaceCount) {
        if (entry.second == 1) {
            // Corner vertex with 1 incident face
            entry.second = 2;
        } else {
            // All other vertices
            entry.second = 2 * entry.second - entry.second;
        }
    }

    for (auto& entry : vertexFaceCount) {
        std::cout << "vertex ID: " << entry.first << " incident edges: " << entry.second << std::endl;
    }

    // // Iterate through each face
    // for (size_t i = 0; i < Q.size(); ++i) {
    //     const Eigen::Vector4i& quad = Q[i];
    //     std::cout << quad[0] << ", " << quad[1] << ", " << quad[2] << ", " << quad[3] << std::endl;
    // }
    // for (int vertexId = 0; vertexId < 31; vertexId++){
    //     std::cout << "vertexr " << vertexId << ": " << vertexFaceCount[vertexId] <<std::endl;
    // }
    

    // // Print the contents of vertexFaceCount
    // for (const auto& entry : vertexFaceCount) {
    //     int vertexId = entry.first;
    //     int count = entry.second;
    //     std::cout << "Vertex " << vertexId << " has " << count << " incident faces" << std::endl;
    // }

    // Print the number of vertices with more than 4 incident faces
    int verticesWithMoreThan4Faces = 0;
    for (const auto& entry : vertexFaceCount) {
        if (entry.second > 4) {
            verticesWithMoreThan4Faces++;
            this->extraordinaryVertices.push_back(entry.first);

        }
    }
    std::cout << "Vertices with more than 4 incident faces: " << verticesWithMoreThan4Faces << std::endl;

     // Print the number of vertices with more than 4 incident faces
    int verticesWithLessThan4Faces = 0;
    for (const auto& entry : vertexFaceCount) {
        if (entry.second < 4) {
            verticesWithLessThan4Faces++;
            this->extraordinaryVertices.push_back(entry.first);

        }
    }
    std::cout << "Vertices with less than 4 incident faces: " << verticesWithLessThan4Faces << std::endl;
    // // Find boundary edges (edges with one incident face)
    // std::unordered_set<Eigen::Vector2i, Vector2iHash> uniqueEdges;
    // for (const Eigen::Vector4i& quad : Q) {
    //     uniqueEdges.insert(Eigen::Vector2i(quad[0], quad[1]));
    //     uniqueEdges.insert(Eigen::Vector2i(quad[1], quad[2]));
    //     uniqueEdges.insert(Eigen::Vector2i(quad[2], quad[3]));
    //     uniqueEdges.insert(Eigen::Vector2i(quad[3], quad[0]));
    // }

    // for (const Eigen::Vector2i& edge : edges) {
    //     int count = 0;
    //     for (const Eigen::Vector4i& quad : Q) {
    //         if (quadContainsEdge(quad, edge)) {
    //             count++;
    //         }
    //     }
    //     if (count == 1) {
    //         boundaryEdges.push_back(edge);
    //     }
    // }

    // // Printing the boundary edges
    // for (const Eigen::Vector2i& edge : boundaryEdges) {
    // std::cout << "(" << edge.x() << ", " << edge.y() << ")" << std::endl;
    // }

    // // Find boundary vertices (vertices connected to boundary edges)
    // std::unordered_set<int> boundaryVertexSet;
    // for (const Eigen::Vector2i& edge : boundaryEdges) {
    //     boundaryVertexSet.insert(edge.x());
    //     boundaryVertexSet.insert(edge.y());
    // }

    // // Check if boundary vertices have exactly 4 incident faces
    // for (int vertex : boundaryVertexSet) {
    //     if (vertexFaceCount[vertex] == 4) {
    //         extraordinaryVertices.push_back(vertex);
    //     }
    // }

    // // Count the number of boundary vertices with exactly 4 incident faces
    // int boundaryVerticesWith4Faces = 0;
    // for (int vertex : boundaryVertexSet) {
    //     if (vertexFaceCount[vertex] == 4) {
    //         boundaryVerticesWith4Faces++;
    //     }
    // }
    // std::cout << "Boundary vertices with exactly 4 incident faces: " << boundaryVerticesWith4Faces << std::endl;
// && std::find(boundaryVertices.begin(), boundaryVertices.end(), vertexId) == boundaryVertices.end()
    // // Find vertices with less than 4 incident edges
    // for (const Eigen::Vector3d& vertex : V) {
    //     int vertexId = static_cast<int>(&vertex - &V[0]);
    //     if (vertexFaceCount[vertexId] < 4) {
    //         this->extraordinaryVertices.push_back(vertexId);
    //     }
    // }
}


MotorcycleGraph::MotorcycleGraph(const QuadMesh& quadmesh) : mesh(quadmesh) {
    this->V = convertVertices(quadmesh.GetVertices());
    this->Q = convertQuads(quadmesh.GetQuads());

    // Initialize edges, extraordinaryVertices, boundaryVertices, and boundaryEdges using MotorCycle_init
    MotorCycle_init(this->V, this->Q);
    submeshes.clear();
}


std::vector<Eigen::Vector2i> MotorcycleGraph::getIncidentEdges(int vertex) {
    std::vector<Eigen::Vector2i> incidentEdges;

    for (const Eigen::Vector4i& face : Q) {
        if (vertex == face[0] || vertex == face[1] || vertex == face[2] || vertex == face[3]) {
            int idx = -1;

            for (int i = 0; i < 4; ++i) {
                if (vertex == face[i]) {
                    idx = i;
                    break;
                }
            }

            if (idx != -1) {
                // Clockwise orientation
                Eigen::Vector2i edgeClockwise(face[idx], face[(idx + 1) % 4]);
                Eigen::Vector2i edgeClockwiseReversed(edgeClockwise[1], edgeClockwise[0]);
                if (std::find(incidentEdges.begin(), incidentEdges.end(), edgeClockwise) == incidentEdges.end() &&
                    std::find(incidentEdges.begin(), incidentEdges.end(), edgeClockwiseReversed) == incidentEdges.end()) {
                    incidentEdges.push_back(edgeClockwise);
                }

                // Counter-clockwise orientation
                Eigen::Vector2i edgeCounterClockwise(face[idx], face[(idx - 1) % 4]);
                Eigen::Vector2i edgeCounterClockwiseReversed(edgeCounterClockwise[1], edgeCounterClockwise[0]);
                if (std::find(incidentEdges.begin(), incidentEdges.end(), edgeCounterClockwise) == incidentEdges.end() &&
                    std::find(incidentEdges.begin(), incidentEdges.end(), edgeCounterClockwiseReversed) == incidentEdges.end()) {
                    incidentEdges.push_back(edgeCounterClockwise);
                }
            }
        }
    }

    return incidentEdges;
}

Eigen::Vector4i MotorcycleGraph::findFaceContainingEdge(const Eigen::Vector2i& edge) {
    int v1 = edge[0];
    int v2 = edge[1];
    
    for (const Eigen::Vector4i& face : Q) {
        if (std::find(face.data(), face.data() + 4, v1) != face.data() + 4 && std::find(face.data(), face.data() + 4, v2) != face.data() + 4) {
            int idx_v1 = std::distance(face.data(), std::find(face.data(), face.data() + 4, v1));
            int idx_v2 = std::distance(face.data(), std::find(face.data(), face.data() + 4, v2));
            
            if ((idx_v1 + 1) % 4 == idx_v2) {
                return face;
            }
        }
    }
    
    return Eigen::Vector4i::Zero(); // Return an empty face if no such face is found
}

int MotorcycleGraph::findEdgeIndexInFace(const Eigen::Vector4i& face, const Eigen::Vector2i& edge) {
    for (int i = 0; i < 4; ++i) {
        if (face[i] == edge[0] && face[(i + 1) % 4] == edge[1]) {
            return i;
        }
    }
    
    return -1; // Return -1 if edge is not found in the face
}

std::pair<Eigen::Vector2i, Eigen::Vector4i> MotorcycleGraph::getOppositeEdgeTopo(const Particle& particle) {
    Eigen::Vector4i currentFace = findFaceContainingEdge(particle.getEdge());
    int idxCurrentEdge = findEdgeIndexInFace(currentFace, particle.getEdge());
    Eigen::Vector2i nextEdge(currentFace[(idxCurrentEdge + 1) % 4], currentFace[(idxCurrentEdge + 2) % 4]);
    Eigen::Vector2i reverseOfNextEdge(nextEdge[1], nextEdge[0]);
    Eigen::Vector4i newFace = findFaceContainingEdge(reverseOfNextEdge);
    int idxReverseOfNextEdge = findEdgeIndexInFace(newFace, reverseOfNextEdge);
    Eigen::Vector2i oppositeEdge(newFace[(idxReverseOfNextEdge + 1) % 4], newFace[(idxReverseOfNextEdge + 2) % 4]);

    return std::make_pair(oppositeEdge, newFace);
}

void MotorcycleGraph::extractSubmeshes() {

    std::unordered_set<Eigen::Vector2i, Vector2iHash> submeshBoundariesSet;
    std::unordered_set<Eigen::Vector4i, Vector4iHash> visitedFaces;

    // Lambda function to find neighboring faces of a given face
    auto findNeighbors = [&](const Eigen::Vector4i& face) {
        std::vector<Eigen::Vector4i> neighbors;
        for (const Eigen::Vector4i& neighborFace : Q) {
            if (neighborFace != face) {
                int sharedVertices = 0;
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        if (face[i] == neighborFace[j]) {
                            sharedVertices++;
                        }
                    }
                    if (sharedVertices == 2) {
                        break;
                    }
                }
                if (sharedVertices == 2) {
                    neighbors.push_back(neighborFace);
                }
            }
        }
        return neighbors;
    };

    for (const Eigen::Vector4i& startFace : Q) {
        if (visitedFaces.find(startFace) != visitedFaces.end()) {
            continue;  // Skip if the face has already been visited
        }

        std::vector<Eigen::Vector4i> submesh;
        // std::vector<Eigen::Vector2i> submeshBoundaries;
        std::queue<Eigen::Vector4i> queue;
        queue.push(startFace);

        while (!queue.empty()) {
            Eigen::Vector4i currentFace = queue.front();
            queue.pop();

            if (visitedFaces.find(currentFace) == visitedFaces.end()) {
                visitedFaces.insert(currentFace);
                submesh.push_back(currentFace);

                // Check edges and update submesh boundaries
                for (int i = 0; i < 4; ++i) {
                    Eigen::Vector2i edge(currentFace[i], currentFace[(i + 1) % 4]);
                    if (std::find(motorcycleEdges.begin(), motorcycleEdges.end(), edge) != motorcycleEdges.end()) {
                        submeshBoundariesSet.insert(edge);
                    }
                }

                // Find neighboring faces using the lambda function
                std::vector<Eigen::Vector4i> neighbors = findNeighbors(currentFace);

                // Add unvisited neighbors to the queue
                for (const Eigen::Vector4i& neighbor : neighbors) {
                    if (visitedFaces.find(neighbor) == visitedFaces.end()) {
                        queue.push(neighbor);
                    }
                }
            }
        }

        this->submeshes.push_back(submesh);
    }
}

// Assuming you have a QuadMesh object mesh with vertices and quads
std::vector<Eigen::Vector3d> MotorcycleGraph::convertVertices(const std::vector<Vertex>& vertices) {
    std::vector<Eigen::Vector3d> convertedVertices;
    convertedVertices.reserve(vertices.size());
    
    for (const Vertex& vertex : vertices) {
        Eigen::Vector3d position(vertex.position.x, vertex.position.y, vertex.position.z);
        convertedVertices.push_back(position);
    }
    
    return convertedVertices;
}

std::vector<Eigen::Vector4i> MotorcycleGraph::convertQuads(const std::vector<Quad>& quads) {
    std::vector<Eigen::Vector4i> convertedQuads;
    convertedQuads.reserve(quads.size());
    
    for (const Quad& quad : quads) {
        Eigen::Vector4i quadIndices(quad.vertexIndices[0], quad.vertexIndices[1], quad.vertexIndices[2], quad.vertexIndices[3]);
        convertedQuads.push_back(quadIndices);
    }
    
    return convertedQuads;
}

// Implementation to return extraordinaryVertices
const std::vector<int>& MotorcycleGraph::getEOV() const {
    return extraordinaryVertices;
}
