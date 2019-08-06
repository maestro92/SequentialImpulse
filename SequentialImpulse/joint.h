#pragma once
#include "physics_core.h"
#include "utility_math.h"

namespace Physics
{
    // for the two bodies just involved in the the contact Which we just resolved, 
    // if they were invovled in other contacts, we have to recompute the closing velocities
    
    // 1000 * 5 mass * 0.016
    float MAX_JOINT_IMPULSE = 80;

    struct Joint
    {
        JointType type;
        PhysBody* a;
        PhysBody* b;

        glm::vec3 aLocalAnchor;
        glm::vec3 bLocalAnchor;

        // mouse joint target pos;
        glm::vec3 targetPos; // absolute position;

        glm::vec3 rA;
        glm::vec3 rB;

        glm::vec3 impulse;
        bool ignoreCollision;
    };

    void WarmStart(Joint& joint)
    {
        switch (joint.type)
        {
        case MOUSE_JOINT:
            // cheat with some damping
            joint.a->angularVelocity *= 0.98f;

            joint.a->velocity -= joint.impulse * joint.a->invMass;
            joint.a->angularVelocity -= joint.a->inverseInertiaTensor * glm::cross(joint.rA, joint.impulse);
            break;

        case RESOLUTE_JOINT:
            joint.a->velocity -= joint.impulse * joint.a->invMass;
            joint.a->angularVelocity -= joint.a->inverseInertiaTensor * glm::cross(joint.rA, joint.impulse);

            if (!(joint.b->flags & Physics::PhysBodyFlag_Static))
            {
                joint.b->velocity += joint.impulse * joint.b->invMass;
                joint.b->angularVelocity += joint.b->inverseInertiaTensor * glm::cross(joint.rB, joint.impulse);
            }
            break;
        }
    }

    void InitAndWarmStartJointVelocityConstraints(Physics::Joint* joint)
    {
        if (joint->a != NULL)
        {
            joint->rA = glm::mat3(joint->a->orientationMat) * joint->aLocalAnchor;
        }
        if (joint->b != NULL)
        {
            joint->rB = glm::mat3(joint->b->orientationMat) * joint->bLocalAnchor;
        }

        WarmStart(*joint);
    }





    void SolveResoluteJointVelocityConstraints(Joint& joint)
    {
        float imA = joint.a->invMass;
        float iiA = joint.a->inverseInertiaTensor[2][2];

        float imB = 0;
        float iiB = 0;
        if (!(joint.b->flags & Physics::PhysBodyFlag_Static))
        {
            imB = joint.b->invMass;
            iiB = joint.b->inverseInertiaTensor[2][2];
        }


        float col00 = imA + iiA * joint.rA.y * joint.rA.y + 
                      imB + iiB * joint.rB.y * joint.rB.y;

        float col01 = -iiA * joint.rA.x * joint.rA.y 
                      -iiB * joint.rB.x * joint.rB.y;


        float col10 = col01;
        float col11 = imA + iiA * joint.rA.x * joint.rA.x + 
                      imB + iiB * joint.rB.x * joint.rB.x;



        float temp[9] = { col00, col01, 0,
                            col10, col11, 0,
                            0.0,  0.0,  0.0 };
        glm::mat3 A = glm::make_mat3(temp);


        // in this case, this is equivalent to CDot, which is J * V_i = 
        glm::vec3 b = -glm::cross(joint.a->angularVelocity, joint.rA);
        b -= joint.a->velocity;
        if (!(joint.b->flags & Physics::PhysBodyFlag_Static))
        {
            b += glm::cross(joint.b->angularVelocity, joint.rB);
            b += joint.b->velocity;
        }

        glm::vec3 impulse = utl::solve22(A, -b);

        joint.impulse += impulse;

        joint.a->velocity -= impulse * joint.a->invMass;
        joint.a->angularVelocity -= joint.a->inverseInertiaTensor * glm::cross(joint.rA, impulse);

        if (!(joint.b->flags & Physics::PhysBodyFlag_Static))
        {
            joint.b->velocity += impulse * joint.b->invMass;
            joint.b->angularVelocity += joint.b->inverseInertiaTensor * glm::cross(joint.rB, impulse);
        }
    }

    bool SolveResoluteJointPositionConstraints(Joint& joint)
    {
        float positionError = 0.0f;

        glm::vec3 rA = glm::mat3(joint.a->orientationMat) * joint.aLocalAnchor;
        glm::vec3 rB = glm::mat3(joint.b->orientationMat) * joint.bLocalAnchor;

        float imA = joint.a->invMass;
        float iiA = joint.a->inverseInertiaTensor[2][2];

        float imB = 0;
        float iiB = 0;
        if (!(joint.b->flags & Physics::PhysBodyFlag_Static))
        {
            imB = joint.b->invMass;
            iiB = joint.b->inverseInertiaTensor[2][2];
        }

        // the constraint
        glm::vec3 C = joint.b->position + rB - joint.a->position - rA;

        positionError = glm::length(C);

        float col00 = joint.a->invMass + iiA * rA.y * rA.y + 
                      joint.b->invMass + iiB * rB.y * rB.y;
        float col01 = -iiA * rA.x * rA.y 
                      -iiB * rB.x * rB.y;

        float col10 = col01;
        float col11 = joint.a->invMass + iiA * rA.x * rA.x + 
                      joint.b->invMass + iiB * rB.x * rB.x;



        float temp[9] = { col00, col01, 0,
                            col10, col11, 0,
                            0.0,  0.0,  0.0 };
        glm::mat3 A = glm::make_mat3(temp);


        glm::vec3 impulse = -utl::solve22(A, C);


        joint.a->position -= impulse * joint.a->invMass;
        glm::vec3 rotation = -joint.a->inverseInertiaTensor * glm::cross(rA, impulse);
        joint.a->addRotation(rotation, 1.0);

        if (!(joint.b->flags & Physics::PhysBodyFlag_Static))
        {
            joint.b->position += impulse * joint.b->invMass;
            rotation = joint.b->inverseInertiaTensor * glm::cross(rB, impulse);
            joint.b->addRotation(rotation, 1.0);
        }
        return positionError <= LINEAR_SLOP;
    }


    bool print = false;

    void SolveMouseJointVelocityConstraints(Joint& joint, float dt_s)
    {
        float imA = joint.a->invMass;
        float iiA = joint.a->inverseInertiaTensor[2][2];

        // designer selects frequency and damping ratio
        // frequency and period T = `1/f.  so the 
        float frequency = 5;
        float omega = 2.0f * 3.14f * frequency;
        float dampingRatio = 0.7;

        // we compute spring-damper coefficients 
        float c = 2.0f * joint.a->mass * dampingRatio * omega;
        float k = joint.a->mass * (omega * omega);


        // we then compute softness paramters
        float gamma = dt_s * (c + dt_s * k);
        gamma = 1.0f / gamma;
        float beta = dt_s * k * gamma;

        float col00 = imA + iiA * joint.rA.y * joint.rA.y + gamma;
        float col01 = -iiA * joint.rA.x * joint.rA.y;
        float col10 = col01;
        float col11 = imA + iiA * joint.rA.x * joint.rA.x + gamma;

        // left handside
        float temp[9] = { col00, col01, 0,
                            col10, col11, 0,
                            0.0,  0.0,  0.0 };
        glm::mat3 A = glm::make_mat3(temp);


        // right hand side
        glm::vec3 jv = -glm::cross(joint.a->angularVelocity, joint.rA);
        jv -= joint.a->velocity;


        // very important, that we go from posA to posB
        glm::vec3 displacement = joint.targetPos - (joint.a->position + joint.rA);
        

        if (print)
        {

            utl::debug("            joint.a->position", joint.a->position);
            utl::debug("            joint.rA.", joint.rA);
            utl::debug("            joint.targetPos", joint.targetPos);

            utl::debug("            displacement", displacement);
        }

        displacement *= beta;

        /*
        utl::debug("            beta", beta);
        utl::debug("            displacement", displacement);
        */

        glm::vec3 softness = glm::vec3(0.0);// gamma * joint.impulse;

        glm::vec3 b = -jv - displacement - softness;



        glm::vec3 dImpulse = utl::solve22(A, b);

    

        if (print)
        {
            utl::debug("            A", A);
            utl::debug("            b", b);
        }

        glm::vec3 oldImpulse = joint.impulse;
        joint.impulse += dImpulse;

        // TODO: clamping
        if (glm::dot(joint.impulse, joint.impulse) > (MAX_JOINT_IMPULSE * MAX_JOINT_IMPULSE))
        {
            joint.impulse *= MAX_JOINT_IMPULSE / glm::length(joint.impulse);
        }

        dImpulse = joint.impulse - oldImpulse;

        joint.a->velocity -= dImpulse * joint.a->invMass;
        joint.a->angularVelocity -= joint.a->inverseInertiaTensor * glm::cross(joint.rA, dImpulse);
    }

    void SolveJointVelocityConstraints(Joint& joint, float dt_s)
    {
        switch (joint.type)
        {
        case MOUSE_JOINT:
            SolveMouseJointVelocityConstraints(joint, dt_s);
            break;
        case RESOLUTE_JOINT:
            SolveResoluteJointVelocityConstraints(joint);
            break;
        }
    }

    bool SolveJointPositionConstraints(Joint& joint)
    {
        switch (joint.type)
        {
            case MOUSE_JOINT:
                return true;

            case RESOLUTE_JOINT:
                return SolveResoluteJointPositionConstraints(joint);            
        }
        return true;
    }
}