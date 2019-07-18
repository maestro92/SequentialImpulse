#pragma once

struct CameraTransform
{
    glm::vec3 position;
};

struct GameInput
{
    glm::vec2 prevMousePoint;
    glm::vec3 MousePoint;
    float dt_s;
};

