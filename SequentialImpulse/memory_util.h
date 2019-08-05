#pragma once
/*

MemoryStorage

so the platform layer gives a memory pointer and a memory storage
the game layer interprets it as multiple memory sections



anytime anyone wants to use something
from GameMemoryStorage, you use it from MemorySection

     ___________________
    |                   |
    | SimMemorySection  |
    |                   |
    |                   |
    |___________________|
    |                   |
    |                   |
    |                   |
    |                   |
    |   AssetSection    |
    |                   |
    |                   |
    |                   |
    |                   |
    |                   |
    |___________________|
    |                   |
    |___________________|
*/

typedef size_t memoryIndex;

struct MemorySection
{
    memoryIndex size;
    uint8_t* base;
    memoryIndex used;
    // used is always of memory alignment sized, 
    // meaning base + used is always gonna point at a memory alignment
};


#define PushSize(memorySection, size)   PushSize_(memorySection, size)
#define PushArray(memorySection, count, type)   (type*)PushSize_(memorySection, (count) * sizeof(type))


const int DEFAULT_MEMORY_SECTION_ALIGNMENT = 4;


memoryIndex GetBytesToNextMemoryAligment(MemorySection* memorySection, memoryIndex alignment)
{
    memoryIndex bytesNeeded = 0;

    memoryIndex curMemoryPointer = (memoryIndex)(memorySection->base + memorySection->used);
    memoryIndex alignmentMask = alignment - 1;

    // want to check if we have bits set in the last few bits
    if (curMemoryPointer & alignmentMask)
    {
        bytesNeeded = alignment - (curMemoryPointer & alignmentMask);
    }

    return bytesNeeded;
}

// user requests for requestedSize, but it will also use up the bytes to the next alignment
memoryIndex GetEffectiveSizeForRequestedSize(MemorySection* memorySection, memoryIndex requestedSize)
{
    return requestedSize + GetBytesToNextMemoryAligment(memorySection, DEFAULT_MEMORY_SECTION_ALIGNMENT);
}

void ResetMemoryAddress(void* address, memoryIndex size)
{
    uint8_t* ptr = (uint8_t*)address;
    while (size--)
    {
        *ptr++ = 0;
    }
}

void* PushSize_(MemorySection* memorySection, memoryIndex requestedSize)
{
    memoryIndex effectiveSize = GetEffectiveSizeForRequestedSize(memorySection, requestedSize);

    // memorySection->used will always be at a memory alignment
    assert((memorySection->used + effectiveSize) <= memorySection->size);


    void* result = memorySection->base + memorySection->used;
    memorySection->used += effectiveSize;

    assert(effectiveSize >= requestedSize);

    ResetMemoryAddress(result, effectiveSize);

    return result;
}


/*
assume alignment is 4, that means want memory addresses that is like:

memory mask will be 0111

    0x00000000
    0x00000004
    0x00000008
    0x0000000C
    0x00000010
    0x00000014

example: if our memory address is 0x00000000, our alignment offset is 0
if the memory address is 0x00000003, we want 1
if the memory address is 0x00000002, we want 2
*/



void InitializeMemorySection(MemorySection* memorySection, void* base, memoryIndex size)
{
    memorySection->size = size;
    memorySection->base = (uint8_t*)base;
    memorySection->used = 0;
}

void InitalizeSubMemorySection(MemorySection* memorySection, MemorySection* masterMemorySection, memoryIndex size)
{
    memorySection->size = size;
    memorySection->base = (uint8_t*)PushSize_(masterMemorySection, size);
    memorySection->used = 0;
}

memoryIndex GetRemainingMemorySize(MemorySection* memorySection)
{
    return memorySection->size - (memorySection->used + GetBytesToNextMemoryAligment(memorySection, DEFAULT_MEMORY_SECTION_ALIGNMENT));
}