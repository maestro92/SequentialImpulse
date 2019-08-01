#pragma once

namespace Physics
{

    const float LINEAR_SLOP = 0.005f;

    enum PhysBodyFlags
    {
        // TODO(casey): Does it make more sense to have the flag be for _non_ colliding entities?
        // TODO(casey): Collides and ZSupported probably can be removed now/soon
        PhysBodyFlag_Collides = (1 << 0),
        PhysBodyFlag_Static = (1 << 1),
        PhysBodyFlag_Awake = (1 << 2),
        PhysBodyFlag_FixedRotation = (1 << 3),


    };

    enum PhysBodyShape
    {
        PB_OBB = 0,
        PB_PLANE = 1,
        PB_SPHERE = 2
    };


    enum JointType
    {
        RESOLUTE_JOINT = 0,
        MOUSE_JOINT = 1,
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

        void InitAsSphere(float radius)
        {
            shape = Physics::PhysBodyShape::PB_SPHERE;
            sphere.center = glm::vec3(0.0);
            sphere.radius = radius;
        }

        void InitAsPlane()
        {

        }

        void InitAsOBB(glm::vec3 halfDim)
        {
            shape = Physics::PhysBodyShape::PB_OBB;
            obb.center = glm::vec3(0, 0, 0);
            obb.axes[0] = glm::vec3(1, 0, 0);
            obb.axes[1] = glm::vec3(0, 1, 0);
            obb.axes[2] = glm::vec3(0, 0, 1);
            obb.halfEdges = halfDim;
        }
    };



    struct PhysBodyDef
    {
        PhysBodyShape shape;
        glm::vec3 halfDim;
        float mass;
        glm::vec3 pos;
        glm::mat4 rot;
        bool hasJoint;
        unsigned int flags;
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
        bool hasJoint;

        float dormantTimer;

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

        void SetAwake(bool awake)
        {
            if (awake)
            {
                flags |= PhysBodyFlag_Awake;
                dormantTimer = 0.0f;
            }
            else
            {
                flags &= ~PhysBodyFlag_Awake;
                dormantTimer = 0.0f;
                
                velocity = glm::vec3(0.0);
                angularVelocity = glm::vec3(0.0);

                forceAccum = glm::vec3(0.0);
                torqueAccum = glm::vec3(0.0);
            }
        }


        glm::mat3 GetSolidSphereInertiaTensor(float mass, float radius)
        {
            float I = mass * radius * radius * 2.0f / 5.0f;

            return glm::mat3(I, 0, 0,
                             0, I, 0,
                             0, 0, I);
        }


        glm::mat3 GetBoxInertiaTensor(float mass, glm::vec3 halfDim)
        {
            float dx = halfDim.x;
            float dy = halfDim.y;
            float dz = halfDim.z;

            float oneOver12 = 1 / 12.0f;
            float m00 = oneOver12 * mass * (dy * dy + dz * dz);
            float m11 = oneOver12 * mass * (dx * dx + dz * dz);
            float m22 = oneOver12 * mass * (dx * dx + dy * dy);

            return glm::mat3(m00, 0, 0,
                0, m11, 0,
                0, 0, m22);
        }

        bool IsStatic()
        {
            return (flags & Physics::PhysBodyFlag_Static) == Physics::PhysBodyFlag_Static;
        }

        bool DoesCollides()
        {
            return (flags & Physics::PhysBodyFlag_Collides) == Physics::PhysBodyFlag_Collides;
        }

        bool HasFixedRotation()
        {
            return  (flags & Physics::PhysBodyFlag_FixedRotation) == Physics::PhysBodyFlag_FixedRotation;
        }

        void initFromPhysBodyDef(PhysBodyDef def)
        {
            Init();
            flags = def.flags;
            mass = def.mass;
            invMass = 1.0f / mass;
            position = def.pos;
            glm::mat4 om = def.rot;
            hasJoint = def.hasJoint;
            //        isAwake = true;

            orientation = glm::toQuat(om);
            SyncOrientationMat();
            scale = def.halfDim;

            if (def.shape == PB_OBB)
            {
                if (!HasFixedRotation())
                {
                    glm::vec3 fullDim = def.halfDim * 2.0f;
                    inertiaTensor = GetBoxInertiaTensor(mass, fullDim);
                    transformInertiaTensor();
                }
                else
                {
                    inertiaTensor = glm::mat3(0.0);
                    inverseInertiaTensor = glm::mat3(0.0);
                }
                shapeData.InitAsOBB(def.halfDim);
            }
            else if (def.shape == PB_SPHERE)
            {
                assert(def.halfDim.x == def.halfDim.y);
                // when in 3D, we need           
                // assert(def.halfDim.y == def.halfDim.z);

                if (!HasFixedRotation())
                {
                    inertiaTensor = GetSolidSphereInertiaTensor(mass, def.halfDim.x);
                    transformInertiaTensor();
                }
                else
                {
                    inertiaTensor = glm::mat3(0.0);
                    inverseInertiaTensor = glm::mat3(0.0);
                }
                shapeData.InitAsSphere(def.halfDim.x);
            }
            else
            {
                assert(0);
            }
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

        void addRotation(glm::vec3 rot)
        {
            glm::quat q(0, rot.x, rot.y, rot.z);

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

        void addRotation(glm::vec3 angularVelocity, float dtSec)
        {
            addRotation(angularVelocity * dtSec);
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
     //           isAwake = true;
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
      //          isAwake = true;
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