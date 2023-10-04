#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Trackball {
public:
    Trackball(float width, float height);

    // Call this function when the mouse button is pressed
    void OnMouseDown(float x, float y);

    // Call this function when the mouse is dragged
    void OnMouseMove(float x, float y);

    // Call this function when the mouse button is released
    void OnMouseUp();

    // Get the rotation matrix to apply to your object
    glm::mat4 GetRotationMatrix();

    // Start tracking
    void StartTracking();

    // Stop tracking
    void StopTracking();

    // Check if tracking is active
    bool IsTracking();

    // Update rotation
    void UpdateRotation();

private:
    bool isDragging;
    glm::vec3 startVector;
    glm::vec3 endVector;
    glm::mat4 rotationMatrix;
    float trackballRadius;
    glm::mat4 accumulatedRotationMatrix;

    glm::vec3 ScreenToSphere(float x, float y);
};
