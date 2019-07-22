#pragma once

// https://stackoverflow.com/questions/15575054/what-does-ll-mean
// LL is long long
#define Kilobytes(value) ((value)*1024LL)
#define Megabytes(value) (Kilobytes(value)*1024LL)
#define Gigabytes(value) (Megabytes(value)*1024LL)



enum GameInputMouseButton
{
    LEFT,
    MIDDLE,
    RIGHT,

    NUM_GAME_INPUT_MOUSE_BUTTON
};
    

// halfTransition is either up->down or down->up
// so a full click up->down->up is 2 half transitions
struct GameInputButtonState
{
    int halfTransitionCount;
    bool endedDown;
};


// we store the states of keyboard buttons (ones you care about)
// and the mouse buttons in GameInput
struct GameInput
{
    GameInputButtonState mouseButtons[NUM_GAME_INPUT_MOUSE_BUTTON];
    glm::vec3 mousePosition;
    float dt_s;
};

struct GameMemory
{
    uint64 memoryStorageSize;
    void* memoryStorage;

    uint64 debugStorageSize;
    void* debugStorage;
};
