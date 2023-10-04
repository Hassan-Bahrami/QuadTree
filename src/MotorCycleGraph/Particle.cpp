#include "MotorCycleGraph/Particle.h"


bool Particle::atInteriorVertex(const std::vector<int>& extraordinaryVertices) {
    // Get the index of the destination vertex (v2)
    int v2 = this->edge[1];

    // Determine if the destination vertex (v2) is an ordinary (interior) vertex
    bool isInterior = true;
    for (int extraordinaryVertex : extraordinaryVertices) {
        if (v2 == extraordinaryVertex) {
            isInterior = false;
            break;
        }
    }

    return isInterior;
}

void Particle::moveTo(const Eigen::Vector2i& newEdge, const Eigen::Vector4i& newFace) {
    // Update the current vertex to the destination vertex of the current edge
    this->vertex = this->edge[1];

    // Update the current edge to the opposite edge
    this->edge = newEdge;

    // Update the current face to the opposite face
    this->face = newFace;

    // Append the new edge to the list of traveled edges
    traveledEdges.push_back(newEdge);
}

const std::vector<Eigen::Vector2i>& Particle::getTraveledEdges() const {  // Note the 'const' qualifier
    return traveledEdges;
}

bool Particle::meetsAnotherParticlesTrack(const std::vector<Particle>& particles) {
    Eigen::Vector2i currentEdge = this->edge;
    int currentVertex = this->vertex;

    for (const Particle& otherParticle : particles) {
        if (&otherParticle != this) {
            const std::vector<Eigen::Vector2i>& otherEdges = otherParticle.getTraveledEdges();
            for (const Eigen::Vector2i& otherEdge : otherEdges) {
                if (otherEdge[0] == currentEdge[1]) {
                    return true;  // Intersection found
                }
            }
        }
    }

    return false;  // No intersection found
}

bool Particle::meetsBoundaryVertex(const std::vector<int>& boundaryVertices) {
    // Get the vertices of the current edge
    int v2 = this->edge[1];

    // Check if either vertex v1 or v2 is a boundary vertex
    if (std::find(boundaryVertices.begin(), boundaryVertices.end(), v2) != boundaryVertices.end()) {
        return true;  // Intersection with a boundary vertex found
    }

    return false;  // No intersection with a boundary vertex found
}

bool Particle::meetsMultipleParticles(const std::vector<Particle>& particles) {
    this->meetingCounter(particles);

    if (metParticles.size() >= 1) {
        return true;
    }

    return false;
}

void Particle::meetingCounter(const std::vector<Particle>& particles) {
    metParticles.clear();  // Clear the metParticles vector
    Eigen::Vector2i currentEdge = edge;

    for (const Particle& otherParticle : particles) {
        if (&otherParticle != this && otherParticle.edge[1] == currentEdge[1]) {
            metParticles.push_back(otherParticle);
        }
    }
}

const Eigen::Vector2i Particle::getEdge() const {
    return this->edge;
}
