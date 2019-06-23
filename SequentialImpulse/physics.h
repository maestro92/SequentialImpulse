#ifndef PHYSICS_H_
#define PHYSICS_H_

#include "entity.h"
#include "physics_core.h"

namespace Physics
{

    struct ContactPoint
    {
        glm::vec3 position;
        glm::vec3 normal;
        float penetration;   // positive if they are separate, negative if they penetrate
        glm::mat4 worldToContact;

        // this is in local contact coordinates
    //    glm::vec3 closingVelocity;
        float desiredSeparatingVelocity;
        glm::vec3 relativeContactPositions[2];

        // when we are doing sequentual impulse, we need to our impulse history
        // this actually causes jitter
        // refer to GDC2006, Erin Cattor Fast and Simple Physics using Sequential Impulses
        float normalImpulse;

        ContactPoint()
        {
            position = glm::vec3(0.0);
            normal = glm::vec3(0.0);
            penetration = 0;
            relativeContactPositions[0] = glm::vec3(0.0);
            relativeContactPositions[1] = glm::vec3(0.0);

            normalImpulse = 0;
        }


        glm::mat3 WorldToContact()
        {
            return glm::mat3(worldToContact);
        }

        glm::mat3 ContactToWorld()
        {
            return glm::mat3(glm::inverse(worldToContact));
        }


        void PrintDebug()
        {
            cout << "########## priting contactpoint " << endl;
            utl::debug("position", position);
            utl::debug("normal", normal);
            utl::debug("penetration", penetration);
            utl::debug("relativeContactPositions[0]", relativeContactPositions[0]);
            utl::debug("relativeContactPositions[1]", relativeContactPositions[1]);
        }
    };


    // assuming all contact points have the same normal. 
    // for this application, this may be true
    // for for advance complicated convex shape, this wont be
    struct CollisionData
    {
        //    glm::vec3 normal;

        int numContactPoints;
        ContactPoint contactPoints[16];
        PhysBody* a;
        PhysBody* b;
        //    glm::vec3 contactPoints[16];

     //   float penetration;

        CollisionData()
        {
            //    normal = glm::vec3(0.0);
            numContactPoints = 0;
            //    penetration = 0;
        }
    };

    /*
    for detailed explanation, see the graph on page 162, figure 5.16
    we compute projection of the most positive vertices onto the plane normal vector.
                ri = (Vi - C) . n

    most positive vertex is, which is: 
                vi = C + axes[0] * halfEdges[0] + axes[1] * halfEdges[1] + axes[2] * halfEdges[2]

    the thing is that it may not be where the most "positive" vertex is intersecting the plane,
    but it could any one of the four vertices. But we are taking absolute values, so we are just
    taking the distance from the center to its most positive vertex and take that distance,
    project it onto the normal and see if its shorter then the distance from the box center to the plane 
    */



    // box2D does 
    /*
    int TestOBBOBB(OBB a, glm::vec4 aTransform, OBB b, glm::vec4 bTransform)
    {

    }
    */

    float restitution = 0.5;

    glm::mat3 GetBoxInertiaTensor(float mass, float xDim, float yDim, float zDim)
    {
        float dx = xDim;
        float dy = yDim;
        float dz = zDim;

        float oneOver12 = 1 / 12.0f;
        float m00 = oneOver12 * mass * (dy * dy + dz * dz);
        float m11 = oneOver12 * mass * (dx * dx + dz * dz);
        float m22 = oneOver12 * mass * (dx * dx + dy * dy);

        return glm::mat3( m00, 0, 0,
                          0, m11, 0,
                          0, 0, m22);
    }


    void GetOBBInertiaTensor()
    {

    }


    void GetSphereInertiaTensor()
    {

    }


    bool testPointInsideOBB2D(glm::vec3 point, OBB b, glm::mat4 rot, glm::vec3 bCenter)
    {
        glm::vec3 realAxes[3] = { glm::vec3(rot * glm::vec4(b.axes[0], 0)),
                                  glm::vec3(rot * glm::vec4(b.axes[1], 0)),
                                  glm::vec3(rot * glm::vec4(b.axes[2], 0))};

        glm::vec3 d = point - bCenter;

        // just need to do 2
        for (int i = 0; i < 2; i++)
        {
            float dist = glm::dot(d, realAxes[i]);
        
            if (dist > b.halfEdges[i] || dist < -b.halfEdges[i])
                return false;        
        }

        return true; 
        // or you can do Casey_s method, which is to do edge tests, see if a point is inside the edge
        // [not sure if this is true, but edge tests seems to be more general purpose?
        // i can handle parallelograms?
        // OBB after is just an rotated AABB
    }



    void TestSphereSphere(CollisionData& contact)
    {
        // to be done.
    }


    // we prefer to use piont-face contacts if we can.
    // 3 possibility:
    // 1.   point-face contact  
    // 2.   edge face contact, then we return two contact point
    // 3.   face face contact, we return four contact point

    // we can find the set of contacts by simply checking each vertex of the box one by one
    // and generating a contact if it lies below the plane.

    
    glm::vec3 GetBoxVertexOffset(glm::vec3 axes[3], glm::vec3 halfEdges, glm::vec3 dir)
    {
        return axes[0] * halfEdges[0] * dir.x + 
                axes[1] * halfEdges[1] * dir.y + 
                axes[2] * halfEdges[2] * dir.z;
    }
    



    int TestOBBPlane(OBB b, PhysBody* bTransform,
        Plane p, PhysBody* pTransform)
    {
        glm::vec3 realAxes[3] = { glm::vec3(bTransform->orientation * glm::vec4(b.axes[0], 0)),
            glm::vec3(bTransform->orientation * glm::vec4(b.axes[1], 0)),
            glm::vec3(bTransform->orientation * glm::vec4(b.axes[2], 0)) };

        float r = b.halfEdges[0] * abs(glm::dot(p.normal, realAxes[0])) +
            b.halfEdges[1] * abs(glm::dot(p.normal, realAxes[1])) +
            b.halfEdges[2] * abs(glm::dot(p.normal, realAxes[2]));

        // distance from box center to the plane
        float s = glm::dot(p.normal, bTransform->position) - p.offset;

        return abs(s) <= r;
    };



    void GetOBBPlaneContacts(OBB b, PhysBody* bBody,
                            Plane p, PhysBody* pBody, CollisionData& contact)
    {
        if (!TestOBBPlane(b, bBody, p, pBody))
        {
            return;
        }

        static glm::vec3 dirs[4] = { glm::vec3(1,1,0),
                                   glm::vec3(-1,1,0),
                                   glm::vec3(1,-1,0),
                                   glm::vec3(-1,-1,0) };

        glm::vec3 realAxes[3] = { glm::vec3(bBody->orientation * glm::vec4(b.axes[0], 0)),
                                glm::vec3(bBody->orientation * glm::vec4(b.axes[1], 0)),
                                glm::vec3(bBody->orientation * glm::vec4(b.axes[2], 0)) };

   //     cout << "in here " << endl;
        
//        cout << "bTransform.position " << bTransform.position << endl;

  //      utl::debug("bTransform.position", bTransform.position);

        for (int i = 0; i < 4; i++)
        {
            /*
            cout << ">>>>>>>>>>> point" << endl;
            utl::debug("        realAxes[0]", realAxes[0]);
            utl::debug("        realAxes[1]", realAxes[1]);
            utl::debug("        realAxes[2]", realAxes[2]);


            utl::debug("        b.halfEdges", b.halfEdges);
            utl::debug("        dirs", dirs[i]);

            glm::vec3 vertexOffset = GetBoxVertexOffset(realAxes, b.halfEdges, dirs[i]);

            utl::debug("        vertexOffset", vertexOffset);
            */

            glm::vec3 vertexPos = bBody->position + b.center + GetBoxVertexOffset(realAxes, b.halfEdges, dirs[i]);

            /*
            utl::debug("        bTransform.position", bTransform.position);
            utl::debug("        vertexPos", vertexPos);
            */
            
            // compute vertex distance from the plane

            // need to change penetration to be negative, cuz in academic papers, they use that convention
            float dist = p.offset - glm::dot(vertexPos, p.normal);

            // every contact point having the same penetration is the wrong assumption
            // fix it!
            // normal is from b to a
            if (dist >= 0)
            {
                ContactPoint* cp = &contact.contactPoints[contact.numContactPoints];

                cp->position = vertexPos + 0.5f * dist * p.normal;
                cp->normal = p.normal;
                cp->penetration = dist;
                contact.numContactPoints++;

                // do I need to remember the two bodies between each contact?

            }
        }
        

    };







    // 

    /*
    1. we work in a set of coordinates that are relative to the contact
        this makes the math a lot simpler. 
          
        This is because we arent interested in the linear and angular velocity of the whole object
        we will be first resolving velocity of contact points, then through the 
        contact points, we will change the velocity and rotation of the object

        the idea is that the physics model we are simulating is, the change in motion of both objects
        in a collision is caused by the forces generated at the collision point by compression and 
        deformation. Because we are representing the whole collision event as a single moment in time,
        we are gonna do it through impulses, rather then forces


    2. GOAL: compute what impulse we need to apply after the collision
           
        for frictionless contacts, the only impulses generated at the contact are applied along the contact normal
        so we have linear and angular

        the linear part, we use an impulse

        for the angular part, you get an impulsive torque from the impulse

        and equation 9.5 tells us the velocity of a point (again, here we are concerned with the contact point,
        not the object)
                    
        so we a have a set of equations that goes from 
        impulse, via the impulsive torque it generates, the angular velocity that the torque causes, then finally
        the linear velocity that results.


        so three steps

        torqueFromImpulsve 
        angularVelocityFromTorque
        velocityFromAngualrVelocity

        transform this velocity into contact coordinates


        
        so for each object in the collision, we can find the change in velocity of the contact point 
        for contacts with two objects, we have four values,
        the velocity caused by the linear motion and by angular motion.

        we will add the resulting values together to get an overall change in velocity per unit impulse.




        we are not intersted in the linear and angular velocity of the whole object
        we are only interested in the separating velocity of the contact points







            the idea is to calculate impulse
            we have current relative velocity
            we calcualte the desired new separating velocity
            impulse = vel - vel2;
    
    
    
    */


    /*
    our current steps are 

        add gravity and drag
 
        Create contacts

        integrate


    so in the create contact step, if we were to remove velocity induced from acceleration
    we need to use last frame's acceleration. because this current frame's collision
    is caused by last frame's acceleration and velocity.
    */
#define DEBUGGING 1





    float computeEffectiveMass(ContactPoint& cp, PhysBody* a, PhysBody* b)
    {
        float totalInvMass = a->invMass;
        glm::vec3 rxn = glm::cross(cp.relativeContactPositions[0], cp.normal);
        glm::vec3 rotation = a->inverseInertiaTensor * rxn;
        totalInvMass += glm::dot(rxn, rotation);

       /*        
        utl::debug("       cp.relativeContactPositions[0] ", cp.relativeContactPositions[0]);
        utl::debug("       rotation ", rotation);
        utl::debug("       totalInvMass ", totalInvMass);
        */




        if (b != NULL)
        {
            totalInvMass += b->invMass;
            rxn = glm::cross(cp.relativeContactPositions[1], -cp.normal);
            rotation = b->inverseInertiaTensor * rxn;
            totalInvMass += glm::dot(rxn, rotation);            
        }

        return 1 / totalInvMass;
    }





    float ComputeRelativeVelocity(ContactPoint& cp, PhysBody* a, PhysBody* b)
    {
        glm::vec3 relativeVelocity = glm::cross(a->angularVelocity, cp.relativeContactPositions[0]);
        relativeVelocity += a->velocity;

        if (b != NULL)
        {
            relativeVelocity += glm::cross(b->angularVelocity, cp.relativeContactPositions[1]);
            relativeVelocity += b->velocity;
        }

        return glm::dot(cp.normal, relativeVelocity);
    }


    // for the two bodies just involved in the the contact Which we just resolved, 
    // if they were invovled in other contacts, we have to recompute the closing velocities
    void SolveVelocityConstraints(ContactPoint& cp, PhysBody* a, PhysBody* b, float dt_s, int j)
    {
        /*
        glm::vec3 relativeVelocity = glm::cross(a->angularVelocity, cp.relativeContactPositions[0]);
        relativeVelocity += a->velocity;

        if (b != NULL)
        {
            relativeVelocity += glm::cross(b->angularVelocity, cp.relativeContactPositions[1]);
            relativeVelocity += b->velocity;
        }

        float relV = glm::dot(cp.normal, relativeVelocity);
        */
        float dv = cp.desiredSeparatingVelocity - ComputeRelativeVelocity(cp, a, b);

        /*
        if (flag)
        {
            utl::debug(">>>>>>>>> here", cp.position);
            utl::debug("        a->angularVelocity", a->angularVelocity);
            utl::debug("        a->velocity", a->velocity);
            cout << "       relV " << relV << endl;
            cout << "       newRelV " << newRelV << endl;
            cout << "       dv " << dv << endl;
        }
        */
        // this is the lambda for the impulse
        float effectiveMass = computeEffectiveMass(cp, a, b);
        float lambda = effectiveMass * dv;

    //    if (flag)
     /* 
        {
            cout << "##########" << j << endl;
            cout << "       effectiveMass " << effectiveMass << endl;

            cout << "       lambda " << lambda << endl;
        }
        */

        float newImpulse = std::max(cp.normalImpulse + lambda, 0.0f);
        lambda = newImpulse - cp.normalImpulse;
        cp.normalImpulse = newImpulse;


    //    if (flag) 
        {
    //        cout << "       newImpulse " << newImpulse << endl << endl;
        }

        glm::vec3 newImpulseVec = lambda * cp.normal;



    //    utl::debug("         before a->velocity", a->velocity);

        a->velocity += newImpulseVec * a->invMass;
        a->angularVelocity += a->inverseInertiaTensor * glm::cross(cp.relativeContactPositions[0], newImpulseVec);

    //    utl::debug("         after a->velocity", a->velocity);

        if (b != NULL)
        {
            b->velocity = b->velocity - newImpulseVec * b->invMass;
            b->angularVelocity += b->inverseInertiaTensor * glm::cross(cp.relativeContactPositions[1], newImpulseVec);
        }
    }



    /*
    void CheckAwakeState(Entity* a, Entity* b)
    {
        if (b == NULL)
            return;

        if (!a->isAwake)
        {
            a->SetAwake(true);
        }

        if (!b->isAwake)
        {
            b->SetAwake(true);
        }
    }
    */

    // equation 9.5 is crucial
    // q_vel = angular_velocity x (q_pos - object_origin) + object_velo [9.5]


    void PrepareContactPoints(CollisionData& contact, PhysBody* a, PhysBody* b)
    {
        float INELASTIC_COLLISION_THRESHOLD = 1.0;
        for (int i = 0; i < contact.numContactPoints; i++)
        {
            ContactPoint& cp = contact.contactPoints[i];
            cp.relativeContactPositions[0] = cp.position - a->position;

            if (b != NULL)
            {
                cp.relativeContactPositions[1] = cp.position - b->position;
            }

            // needs to be after relativeContactPositions are set
            float vRel = ComputeRelativeVelocity(cp, a, b);

            if (vRel > -INELASTIC_COLLISION_THRESHOLD)
            {
                cp.desiredSeparatingVelocity = 0;
            }
            else
            {
                cp.desiredSeparatingVelocity = -restitution * vRel;
            }
        
        }
    }

    void ResolveVelocity(CollisionData& contact, PhysBody* a, PhysBody* b, float dt_s)
    {
       /*
#if DEBUGGING
        cout << "########## resolving " << contact.numContactPoints << " contact points" << endl;

        for (int i = 0; i < contact.numContactPoints; i++)
        {
            cout << "           penetration at start up " << contact.contactPoints[i].penetration << endl;
        }
#endif
*/
    //    cout << "########## newTick " << endl;
        int velocityIterations = 4;
        for (int i = 0; i < velocityIterations; i++)
        {
            for (int j = 0; j < contact.numContactPoints; j++)
            {
                SolveVelocityConstraints(contact.contactPoints[j], a, b, dt_s, j);
            }
        }
    }


    void ResolvePosition(CollisionData& contact, PhysBody* a, PhysBody* b, float dt_s)
    {
        float baumgarte = 0.2;
        float linearSlop = 0.005f;
        float maxLinearCorrection = 0.2f;

        int positionIterations = 3;
        for (int i = 0; i < positionIterations; i++)
        {
            float largestPenetration = 0.0f;
            for (int j = 0; j < contact.numContactPoints; j++)
            {                
                ContactPoint cp = contact.contactPoints[j];

                // we track the largest penetration
                largestPenetration = min(largestPenetration, cp.penetration);

                // even at max, we only want to resolve cp.penetrate - linearSlop of penetration
                float positionCorrection = baumgarte * (cp.penetration - linearSlop);
                positionCorrection = max(0.0f, positionCorrection);
                positionCorrection = min(maxLinearCorrection, positionCorrection);

                /*
                cout << ">>>>>>> j " << j << endl;
                cout << "       cp.penetration " << cp.penetration << endl;
                cout << "       positionCorrection " << positionCorrection << endl;
                */

                float effectiveMass = computeEffectiveMass(cp, a, b);

                float impulsePerInvMass = positionCorrection / effectiveMass;

                glm::vec3 impulsePerInvMassVec = impulsePerInvMass * cp.normal;

                // utl::debug("        impulsePerInvMassVec ", impulsePerInvMassVec);
                // cout << "positionCorrection " << positionCorrection << endl;


                a->position += a->invMass * impulsePerInvMassVec;
                glm::vec3 rotation = glm::cross(cp.relativeContactPositions[0], impulsePerInvMassVec);
                a->addRotation(rotation, 1.0);

                if (b != NULL)
                {
                    b->position += b->invMass * impulsePerInvMassVec;
                    rotation = glm::cross(cp.relativeContactPositions[1], impulsePerInvMassVec);
                    b->addRotation(rotation, 1.0);
                }
            }

            if (largestPenetration < 3 * linearSlop)
            {
                break;
            }
        }
    }


    void GenerateContactInfo(PhysBody* a, PhysBody* b, CollisionData& contact)
    {

        if (a->shapeData.shape == PhysBodyShape::PB_OBB && b->shapeData.shape == PhysBodyShape::PB_PLANE)
        {
            glm::vec3 aCenter = a->position + a->shapeData.obb.center;

            contact.a = a;
            contact.b = NULL;

            GetOBBPlaneContacts(a->shapeData.obb, a, b->shapeData.plane, b, contact);
        }
    };
};

#endif