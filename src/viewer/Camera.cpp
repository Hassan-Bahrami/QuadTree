#include "viewer/Camera.h"
#include <algorithm>

Camera::Camera() {
    position = glm::vec3(0, 0, 10);
    front = glm::vec3(0, 0, -1);
    up = glm::vec3(0, 1, 0);

    update_vectors();
    compute_projection();
}

Camera::Camera(glm::vec3 position, glm::vec3 front, glm::vec3 up, float aspect_ratio)
    : position(position), front(front), up(up), aspect_ratio(aspect_ratio) {
    update_vectors();
    compute_projection();
}

glm::mat4 const Camera::projection() {
    return _projection;
}

glm::mat4 const Camera::view() {
    return _view;
}

void Camera::set_position(glm::vec3 position) {
    this->position = position;
    compute_view();
}

void Camera::set_front(glm::vec3 front) {
    this->front = front;
    compute_view();
}

void Camera::set_up(glm::vec3 up) {
    this->up = up;
    compute_view();
}

void Camera::set_aspect(float aspect_ratio) {
    this->aspect_ratio = aspect_ratio;
    compute_projection();
}

void Camera::update_vectors() {
    front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    front.y = sin(glm::radians(pitch));
    front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    front = glm::normalize(front);

    right = glm::normalize(glm::cross(front, world_up));
    up = glm::normalize(glm::cross(right, front));

    compute_view();
}

void Camera::compute_view() {
    _view = glm::lookAt(position, position + front, up);
}

void Camera::compute_projection() {
    _projection = glm::perspective(glm::radians(zoom), aspect_ratio, near, far);
}

FPSCamera::FPSCamera(glm::vec3 position, glm::vec3 front, glm::vec3 up, float aspect_ratio, float speed, float sensitivity, float scroll_sensitivity)
    : Camera(position, front, up, aspect_ratio) {
    this->speed = speed;
    this->sensitivity = sensitivity;
    this->scroll_sensitivity = scroll_sensitivity;

    float zoom = 45.0f; // Initial zoom level
    float horizontalAngle = 0.0f; // Horizontal rotation angle
    float verticalAngle = 0.0f;   // Vertical rotation angle

}

void FPSCamera::move(CameraMovement movement, float dt) {
    float velocity = speed * dt;
    switch (movement) {
    case CameraMovement::FORWARD:
        position += front * velocity;
        this->set_position(position);
        compute_projection();
        break;
    case CameraMovement::BACKWARD:
        position -= front * velocity;
        this->set_position(position);
        compute_projection();
        break;
    case CameraMovement::LEFT:
        position -= right * velocity;
        this->set_position(position);
        compute_projection();
        break;
    case CameraMovement::RIGHT:
        position += right * velocity;
        this->set_position(position);
        compute_projection();
        break;
    case CameraMovement::UP:
        position += world_up * velocity;
        this->set_position(position);
        compute_projection();
        break;
    case CameraMovement::DOWN:
        position -= world_up * velocity;
        this->set_position(position);
        compute_projection();
        break;
    }
}

void FPSCamera::mouse_control(float xoff, float yoff) {
    xoff *= sensitivity;
    yoff *= sensitivity;

    yaw += xoff;
    pitch += yoff;

    pitch = std::min(89.0f, std::max(-89.0f, pitch));

    update_vectors();
}

void FPSCamera::scroll_control(float yoff) {
    zoom -= yoff * scroll_sensitivity;
    zoom = std::min(90.0f, std::max(1.0f, zoom));
    compute_projection();
}

OrbitCamera::OrbitCamera() : Camera() {
    update_vectors();
}

OrbitCamera::OrbitCamera(float radius) : _radius(radius) {
    update_vectors();
    compute_projection();
}

OrbitCamera::OrbitCamera(float radius, glm::vec3 position, glm::vec3 front, glm::vec3 up, float aspect_ratio)
    : _radius(radius), Camera(position, front, up, aspect_ratio) {
    update_vectors();
    compute_projection();
}

void OrbitCamera::rotate(float dtheta, float dphi) {
    float _dtheta = dtheta * sensitivity;
    float _dphi = dphi * sensitivity;

    _phi += _dphi;

    if (_phi > 180) _phi = 180;
    if (_phi < 0) _phi = 0;

    _theta -= _dtheta;
    if (_theta > 360) _theta -= 360;
    if (_theta < 0) _theta += 360;

    if ((_phi > 0 && _phi < 180) || (_phi < -180 && _phi > -360)) {
        world_up = glm::vec3(0, 1, 0);
    } else {
        world_up = glm::vec3(sin(glm::radians(_theta)), 0, cos(glm::radians(_theta)));
    }

    update_vectors();
}

void OrbitCamera::zoom(float dradius) {
    if (std::abs(dradius) > 0.001f) {
        _radius -= dradius * scroll_sensitivity;
        _radius = std::max(near, _radius);
        update_vectors();
    }
}

void OrbitCamera::update_vectors() {
    float x = sin(glm::radians(_phi)) * sin(glm::radians(_theta));
    float y = cos(glm::radians(_phi));
    float z = sin(glm::radians(_phi)) * cos(glm::radians(_theta));

    front = -glm::vec3(x, y, z);
    position = _radius * glm::vec3(x, y, z);
    right = glm::normalize(glm::cross(front, world_up));
    up = glm::normalize(glm::cross(right, front));

    compute_view();
}
