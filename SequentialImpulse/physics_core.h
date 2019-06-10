#pragma once

namespace Physics
{

    enum PhysBodyType
    {
        PB_OBB = 0,
        PB_PLANE = 1
    };

    struct PhysBodyTransform
    {
        glm::vec3 position;
        glm::mat4 orientation;
    };

    // assuming all contact points have the same normal. 
    // for this application, this may be true
    // for for advance complicated convex shape, this wont be
    struct CollisionData
    {
        glm::vec3 normal;

        int numContactPoints;
        glm::vec3 contactPoints[16];

        float penetration;

        CollisionData()
        {
            normal = glm::vec3(0.0);
            numContactPoints = 0;
            penetration = 0;
        }
    };

    struct ContactPoint
    {
        glm::vec3 position;
        glm::vec3 normal;
        float penetration;
        glm::vec3 relativeContactPositions[2];


        ContactPoint()
        {
            position = glm::vec3(0.0);
            normal = glm::vec3(0.0);
            penetration = 0;
            relativeContactPositions[0] = glm::vec3(0.0);
            relativeContactPositions[1] = glm::vec3(0.0);
        }
    };



    struct Sphere
    {
        glm::vec3 center;
        float radius;
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


    // you can think of d as a offset from the origin to the plane the plane
    // so using the point-normal representation of plane
    // if p is a point on the plane, and we treat it like center point of the plane (since the plane is infinite)
    // then, d = dot(normal, p) is the offset distance from the plane to the origin 
    
    
    // however, do note that, this distance is in the direction of plane.normal
    // so its essentially the closest distance from the plane to the origin


    struct Plane
    {
        glm::vec3 normal;
        float offset; // d = dot(normal, p)
    };


    struct PhysBody
    {
        PhysBodyType type;
        OBB obb;
        Plane plane;
        Sphere sphere;

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