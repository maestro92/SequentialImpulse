#pragma once

namespace Physics
{
    enum PhysBodyFlags
    {
        // TODO(casey): Does it make more sense to have the flag be for _non_ colliding entities?
        // TODO(casey): Collides and ZSupported probably can be removed now/soon
        PhysBodyFlag_Collides = (1 << 0),
        PhysBodyFlag_Static = (1 << 1),
    };



    enum PhysBodyShape
    {
        PB_OBB = 0,
        PB_PLANE = 1,
        PB_CIRCLE = 2
    };

    
    struct PhysBodyTransform
    {
        glm::vec3 position;
        glm::vec3 axes[3];
    };
    

    struct LineSegment
    {
        glm::vec3 v0;
        glm::vec3 v1;
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

    // the point-normal representation of plane
    struct Plane
    {
        glm::vec3 normal;
     //   float offset; // d = dot(normal, p)
        glm::vec3 point;
    };


    struct PhysBodyShapeData
    {
        PhysBodyShape shape;
        union
        {
            OBB obb;
            Plane plane;
            Sphere sphere;
        };
    };

    struct PhysBody
    {
        int id;
        PhysBodyShapeData shapeData;
        unsigned int flags;
        float mass;
        float invMass;
        glm::vec3 position;
        glm::vec3 velocity;
        glm::vec3 scale;
        glm::quat orientation;
        glm::mat4 orientationMat;

        glm::vec3 angularVelocity;

        float velocityDamping;
        float angularDamping;

        // this is in local coordinate
        glm::mat3 inertiaTensor;

        // this is in world cooridnates
        glm::mat3 inverseInertiaTensor;

        glm::vec3 forceAccum;
        glm::vec3 torqueAccum;

        bool isAwake;


        void Init()
        {
            position = glm::vec3(0.0, 0.0, 0.0);
            velocity = glm::vec3(0.0, 0.0, 0.0);
            scale = glm::vec3(1.0, 1.0, 1.0);
            orientation = glm::quat(1.0, 0.0, 0.0, 0.0);
            orientationMat = glm::mat4(1.0);
            angularVelocity = glm::vec3(0.0, 0.0, 0.0);
            flags = 0;

            forceAccum = glm::vec3(0.0);
            torqueAccum = glm::vec3(0.0);
        }

#if 0
        inline void Entity::setOrientation(glm::mat4 rot)
        {
            /*
            m_xAxis = glm::vec3(rot[0][0], rot[0][1], rot[0][2]);
            m_yAxis = glm::vec3(rot[1][0], rot[1][1], rot[1][2]);
            m_zAxis = glm::vec3(rot[2][0], rot[2][1], rot[2][2]);

            float temp[16] = { rot[0][0], rot[0][1], rot[0][2], 0.0,
                              rot[1][0], rot[1][1], rot[1][2], 0.0,
                              rot[2][0], rot[2][1], rot[2][2], 0.0,
                              0.0,       0.0,       0.0,       1.0 };
                              */
            orientationMat = glm::make_mat4(temp);
        }
#endif



        void addRotation(glm::vec3 angularVelocity, float dtSec)
        {
            glm::quat q(0, angularVelocity.x * dtSec, angularVelocity.y * dtSec, angularVelocity.z * dtSec);

            q = q * orientation;

            orientation.w += q.w * 0.5;
            orientation.x += q.x * 0.5;
            orientation.y += q.y * 0.5;
            orientation.z += q.z * 0.5;

            orientation = glm::normalize(orientation);

            if (orientation.w == 1)
            {
                if (abs(orientation.x) < 0.001)
                {
                    orientation.x = 0;
                }
                if (abs(orientation.y) < 0.001)
                {
                    orientation.y = 0;
                }
                if (abs(orientation.z) < 0.001)
                {
                    orientation.z = 0;
                }

            }

            //            orientation = normalizeQuat(orientation);

            orientation = normalizeQuat(orientation);
            SyncOrientationMat();
        }

        glm::quat normalizeQuat(glm::quat q)
        {
            q = glm::normalize(q);

            if (q.w == 1)
            {
                if (abs(q.x) < 0.001)
                {
                    q.x = 0;
                }
                if (abs(q.y) < 0.001)
                {
                    q.y = 0;
                }
                if (abs(q.z) < 0.001)
                {
                    q.z = 0;
                }
            }
            return q;
        }

        void SyncOrientationMat()
        {
            orientationMat = glm::toMat4(orientation);
        }

        // not really needed in 2D, but we will keep this code for completeness
        // still need to verify whether this is correct
        void transformInertiaTensor()
        {
            // convert this to world coordinates
            // the world 3 axes is just (1,0,0), (0,1,0), and (0,0,1)

            glm::mat3 orientation3x3 = glm::mat3(orientationMat);
            //    utl::debug("orientation ", orientation);
            //    utl::debug("orientation3x3 ", orientation3x3);
            //    utl::debug("inertiaTensor ", inertiaTensor);

            inverseInertiaTensor = glm::inverse(orientation3x3) * inertiaTensor;
            // then inverse it 
            inverseInertiaTensor = glm::inverse(inverseInertiaTensor);

            //    utl::debug("inverseInertiaTensor ", inverseInertiaTensor);
        }

        void addForce(glm::vec3 f, bool awakesEntity)
        {
            forceAccum += f;
            if (awakesEntity)
            {
                isAwake = true;
            }
        }

        /*
        void SetAwake(bool awake)
        {
            if (awake)
            {
                physBody.isAwake = awake;
                motion = sleepEpislon * 2.0f;
            }
            else
            {
                physBody.isAwake = false;
                velocity = glm::vec3(0, 0, 0);
                angularVelocity = glm::vec3(0, 0, 0);
            }
        }
        */

        void addTorqueFromForce(glm::vec3 f, glm::vec3 vecToForce, bool awakesEntity)
        {
            torqueAccum += glm::cross(vecToForce, f);
            if (awakesEntity)
            {
                isAwake = true;
            }
            utl::debug("    torqueAccum is ", torqueAccum);
        }

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