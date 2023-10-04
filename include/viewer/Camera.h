#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Camera {
public:
    Camera();
    Camera(glm::vec3 position, glm::vec3 front, glm::vec3 up, float aspect_ratio);

    glm::mat4 const projection();
    glm::mat4 const view();

    void set_position(glm::vec3 position);
    void set_front(glm::vec3 front);
    void set_up(glm::vec3 up);
    void set_aspect(float aspect_ratio);

    // User control
    float speed = 10;
    float sensitivity = 1;
    float scroll_sensitivity = 1;

protected:
    glm::vec3 position, front, up, right;
    glm::vec3 world_up = glm::vec3(0, 1, 0);
    float yaw = -90, pitch = 0, zoom = 45;
    float near = 0.1f, far = 5000.0f;
    float aspect_ratio = 16.0f / 9.0f;

    void update_vectors();
    void compute_view();
    void compute_projection();

private:
    glm::mat4 _projection, _view;
};

class FPSCamera : public Camera {
public:
    enum CameraMovement {
        FORWARD, BACKWARD, LEFT, RIGHT, UP, DOWN
    };

    FPSCamera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 10.0f), glm::vec3 front = glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), float aspect_ratio = 4.0f / 3.0f, float speed = 10.0f, float sensitivity = 0.1f, float scroll_sensitivity = 1.0f);


    void move(CameraMovement movement, float dt);
    void mouse_control(float xoff, float yoff);
    void scroll_control(float yoff);
};

class OrbitCamera : public Camera {
public:
    OrbitCamera();
    OrbitCamera(float radius);
    OrbitCamera(float radius, glm::vec3 position, glm::vec3 front, glm::vec3 up, float aspect_ratio);

    void rotate(float dtheta, float dphi);
    void zoom(float dradius);

private:
    float _theta = 0, _phi = 90, _radius = 10;
    float speed = 10, sensitivity = 1, scroll_sensitivity = 1;

    void update_vectors();
};
