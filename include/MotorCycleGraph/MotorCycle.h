#ifndef MOTORCYCLE_H
#define MOTORCYCLE_H

#include <Eigen/Dense>
#include <vector>
#include <map>

#include "Particle.h"
#include "viewer/QuadMesh.h"


class MotorcycleGraph {
public:
    MotorcycleGraph(const QuadMesh& quadmesh);

    void constructMotorcycleGraph();
    void extractSubmeshes();
    void visualizeMotorcycleEdges();
    const std::vector<Eigen::Vector2i>& getMotorcycleEdges() const;
    const std::vector<int>& getEOV() const;

private:
    QuadMesh mesh;
    std::vector<Eigen::Vector3d> V;
    std::vector<Eigen::Vector4i> Q;
    std::vector<Eigen::Vector2i> E;
    std::vector<int> extraordinaryVertices;
    std::vector<int> boundaryVertices;
    std::vector<Eigen::Vector2i> boundaryEdges;
    std::map<int, Eigen::Vector3d> H;
    std::vector<Particle> particles;
    std::vector<Eigen::Vector2i> motorcycleEdges;
    std::vector<std::vector<Eigen::Vector4i>> submeshes;


    std::vector<Eigen::Vector2i> getIncidentEdges(int vertex);
    bool areOppositeTravelers(const Particle& particle1, const Particle& particle2);
    bool arePerpendicular(const Particle& particle1, const Particle& particle2);
    Eigen::Vector4i findFaceContainingEdge(const Eigen::Vector2i& edge);
    int findEdgeIndexInFace(const Eigen::Vector4i& face, const Eigen::Vector2i& edge);
    std::pair<Eigen::Vector2i, Eigen::Vector4i> getOppositeEdgeTopo(const Particle& particle);
    std::vector<Eigen::Vector3d> convertVertices(const std::vector<Vertex>& vertices);
    std::vector<Eigen::Vector4i> convertQuads(const std::vector<Quad>& quads);
    bool quadContainsEdge(const Eigen::Vector4i& quad, const Eigen::Vector2i& edge);
    void MotorCycle_init(const std::vector<Eigen::Vector3d>& V,
                     const std::vector<Eigen::Vector4i>& Q);
};

#endif // MOTORCYCLEGRAPH_H
