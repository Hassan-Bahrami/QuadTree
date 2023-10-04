#ifndef PARTICLE_H
#define PARTICLE_H

#include <Eigen/Core>
#include <vector>

class Particle {
public:
    Particle(const Eigen::Vector2i& edge, const int& vertex, const Eigen::Vector4i& face);
    bool atInteriorVertex(const std::vector<int>& extraordinaryVertices);
    void moveTo(const Eigen::Vector2i& newEdge, const Eigen::Vector4i& newFace) ;
    const std::vector<Eigen::Vector2i>& getTraveledEdges() const;  // Note the 'const' qualifier;
    bool meetsAnotherParticlesTrack(const std::vector<Particle>& particles);
    bool meetsBoundaryVertex(const std::vector<int>& boundaryVertices);
    bool meetsMultipleParticles(const std::vector<Particle>& particles);
    const Eigen::Vector2i getEdge() const;

private:
    int index;
    static int particleCounter;
    Eigen::Vector2i edge;
    int vertex;
    Eigen::Vector4i face;
    std::vector<Eigen::Vector2i> traveledEdges;
    std::vector<Particle> metParticles;

    void meetingCounter(const std::vector<Particle>& particles);
};

#endif // PARTICLE_H
