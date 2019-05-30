#pragma once

namespace Physics
{

    enum PhysBodyType
    {
        PB_OBB = 0,
        PB_PLANE = 1
    };

    struct ContactInfo
    {
        glm::vec3 normal;

        int numContactPoints;
        glm::vec3 contactPoints[16];
    };

    // considering using an AABB to do an early out?
    // oriented bounding boxes
    struct OBB
    {
        glm::vec3 center;
        glm::vec3 axes[3];
        glm::vec3 halfEdges;
    };

    // page 55
    // imagine the point-normal representation of plane
    // d = dot(normal, p);
    struct Plane
    {
        glm::vec3 normal;
        float d; // d = dot(normal, p)
    };


    struct PhysBody
    {
        PhysBodyType type;
        OBB obb;
        Plane plane;

        /*
        apparently entity.h copy constructor is disabled if we have union
        so we have to figure out another way to solve it if we REALLY REALLY want union here
        union
        {
            OBB obb;
            Plane plane;
        };
        */

    };
}