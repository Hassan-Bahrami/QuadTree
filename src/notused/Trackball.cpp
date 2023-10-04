#include "Trackball.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <iostream>

Trackball::Trackball(float width, float height) : isDragging(false), rotationMatrix(1.0f), accumulatedRotationMatrix(1.0f) {
    // Initialize the rotation matrices to identity matrices
    rotationMatrix = glm::mat4(1.0f);
    accumulatedRotationMatrix = glm::mat4(1.0f);

    // Calculate the radius of the trackball based on the window size
    float minDim = glm::min(width, height);
    trackballRadius = minDim / 2.0f;
}

void Trackball::OnMouseDown(float x, float y) {
    // Start dragging and store the initial click position
    isDragging = true;
    startVector = ScreenToSphere(x, y);
}

void Trackball::OnMouseMove(float x, float y) {
    if (isDragging) {
        // Calculate the current position of the mouse on the sphere
        endVector = ScreenToSphere(x, y);

        // Calculate the rotation axis and angle
        glm::vec3 rotationAxis = glm::cross(startVector, endVector);
        float dotProduct = glm::dot(startVector, endVector);
        float rotationAngle = glm::acos(glm::min(1.0f, dotProduct)) * 2.0f;

        // Create the rotation matrix
        glm::mat4 currentRotation = glm::rotate(glm::mat4(1.0f), rotationAngle, rotationAxis);
        rotationMatrix = currentRotation * rotationMatrix;

        // Update the starting vector for the next drag
        startVector = endVector;
    }
}

void Trackball::OnMouseUp() {
    isDragging = false;
    // Call the UpdateRotation function when the mouse is released
    UpdateRotation();
}

bool Trackball::IsTracking() {
    return isDragging;
}

void Trackball::StartTracking() {
    isDragging = true;
}

void Trackball::StopTracking() {
    isDragging = false;
}

void Trackball::UpdateRotation() {
    // Accumulate the current rotation into the accumulated rotation matrix
    accumulatedRotationMatrix = rotationMatrix * accumulatedRotationMatrix;

    // Reset the current rotation matrix
    rotationMatrix = glm::mat4(1.0f);
}

glm::mat4 Trackball::GetRotationMatrix() {
    // Return the accumulated rotation matrix
    return accumulatedRotationMatrix;
}

glm::vec3 Trackball::ScreenToSphere(float x, float y) {
    // Normalize screen coordinates to the range [-1, 1]
    float normalizedX = (2.0f * x) / trackballRadius - 1.0f;
    float normalizedY = 1.0f - (2.0f * y) / trackballRadius;

    // Calculate the squared length from the origin
    float lengthSquared = normalizedX * normalizedX + normalizedY * normalizedY;

    // Check if the point is inside or outside the sphere
    if (lengthSquared <= 1.0f) {
        // Inside the sphere
        float z = glm::sqrt(1.0f - lengthSquared);
        return glm::vec3(normalizedX, normalizedY, z);
    } else {
        // Outside the sphere, project the point onto the sphere's surface
        glm::vec3 result = glm::normalize(glm::vec3(normalizedX, normalizedY, 0.0f));
        return result;
    }
}
