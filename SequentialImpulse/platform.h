#pragma once

// https://stackoverflow.com/questions/15575054/what-does-ll-mean
// LL is long long
#define Kilobytes(value) ((value)*1024LL)
#define Megabytes(value) (Kilobytes(value)*1024LL)
#define Gigabytes(value) (Megabytes(value)*1024LL)

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

struct GameMemory
{
    uint64 memoryStorageSize;
    void* memoryStorage;

    uint64 debugStorageSize;
    void* debugStorage;
};
