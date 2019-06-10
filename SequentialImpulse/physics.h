#ifndef PHYSICS_H_
#define PHYSICS_H_

#include "entity.h"
#include "physics_core.h"

namespace Physics
{
 

    // separating-axis test
    // only the axis parallel to the plane normal n need to be tested 
    


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
    



    int TestOBBPlane(OBB b, PhysBodyTransform bTransform,
        Plane p, PhysBodyTransform pTransform)
    {
        glm::vec3 realAxes[3] = { glm::vec3(bTransform.orientation * glm::vec4(b.axes[0], 0)),
            glm::vec3(bTransform.orientation * glm::vec4(b.axes[1], 0)),
            glm::vec3(bTransform.orientation * glm::vec4(b.axes[2], 0)) };

        float r = b.halfEdges[0] * abs(glm::dot(p.normal, realAxes[0])) +
            b.halfEdges[1] * abs(glm::dot(p.normal, realAxes[1])) +
            b.halfEdges[2] * abs(glm::dot(p.normal, realAxes[2]));

        // distance from box center to the plane
        float s = glm::dot(p.normal, bTransform.position) - p.offset;

        return abs(s) <= r;
    };



    void GetOBBPlaneContacts(OBB b, PhysBodyTransform bTransform,
        Plane p, PhysBodyTransform pTransform, CollisionData& contact)
    {
        if (!TestOBBPlane(b, bTransform, p, pTransform))
        {
            return;
        }

        static glm::vec3 dirs[4] = { glm::vec3(1,1,0),
                                   glm::vec3(-1,1,0),
                                   glm::vec3(1,-1,0),
                                   glm::vec3(-1,-1,0) };

        glm::vec3 realAxes[3] = { glm::vec3(bTransform.orientation * glm::vec4(b.axes[0], 0)),
                                glm::vec3(bTransform.orientation * glm::vec4(b.axes[1], 0)),
                                glm::vec3(bTransform.orientation * glm::vec4(b.axes[2], 0)) };

   //     cout << "in here " << endl;
        int contactIndex = 0;

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

            glm::vec3 vertexPos = bTransform.position + b.center + GetBoxVertexOffset(realAxes, b.halfEdges, dirs[i]);

            /*
            utl::debug("        bTransform.position", bTransform.position);
            utl::debug("        vertexPos", vertexPos);
            */
            
            // compute vertex distance from the plane

            float dist = p.offset - glm::dot(vertexPos, p.normal);

            // normal is from b to a
            if (dist >= 0)
            {
                contact.contactPoints[contactIndex] = vertexPos + 0.5f * dist * p.normal;
                contact.numContactPoints++;
                contact.normal = p.normal;
                contact.penetration = dist;

                // do I need to remember the two bodies between each contact?

            }
        }
        

    };







    int TestSphereOBB()
    {
        return 0;
    }













    // impulsve for lienar motion
    // impulsive torque for angular motion

    // w is angular velocity
    // dw = I^-1 * impulsive torque


    float CalculateSeparatingVelocity(Entity* a, Entity* b, glm::vec3 contactNormal)
    {
        glm::vec3 relativeVelocity = a->velocity;
        if (!b->flags & EntityFlag_Static)
        {
            relativeVelocity -= b->velocity;
        }

        return glm::dot(relativeVelocity, contactNormal);
    }

    // note separatingVelocity, newSeparatingVelocity, deltaVelocity are all along contact normal
    void ResolveVelocityLinearOnly(CollisionData& contact, Entity* a, Entity* b)
    {
        float separatingVelocity = CalculateSeparatingVelocity(a, b, contact.normal);

        if (separatingVelocity > 0)
        {
            // no impulse required
            return;
        }

        float newSeparatingVelocity = -separatingVelocity * 0.5f;
                
        float deltaVelocity = newSeparatingVelocity - separatingVelocity;

        float totalInverseMass = a->invMass;
        if (!b->flags & EntityFlag_Static)
        {
            totalInverseMass += b->invMass;
        }

        if (totalInverseMass <= 0)
            return;

        float impulse = deltaVelocity / totalInverseMass;
        glm::vec3 impulsePerInvMass = contact.normal * impulse;

        a->velocity = a->velocity + impulsePerInvMass * a->invMass;

        if (!(b->flags & EntityFlag_Static))
        {
            b->velocity = b->velocity - impulsePerInvMass * b->invMass;
        }

        return;
    }

    void ResolveInterpenetrationLinearOnly(CollisionData& contact, Entity* a, Entity* b)
    {
        if (contact.penetration < 0)
        {
            return;
        }

        float totalInverseMass = a->invMass;
        if (!b->flags & EntityFlag_Static)
        {
            totalInverseMass += b->invMass;
        }

        if (totalInverseMass <= 0)
            return;

        glm::vec3 movePerInvMass = contact.normal * (contact.penetration / totalInverseMass);

        // do we want to resolve position or combine this into the velocity?
        a->position = a->position + movePerInvMass * a->invMass;
        if (!b->flags & EntityFlag_Static)
        {
            b->position = b->position - movePerInvMass * b->invMass;
        }
    }

    /*
    14.2.1
    inverting a matrix is complex yet, if your matrix is only rotation, and it doesnt have translational
    also no skewing or scaling, since our rotation matrix is orthonormal, then the transpose is the same as 
    the inverse
    */

    glm::mat4 CreateContactCoordinateBasis(CollisionData& contact)
    {
        // page 311
        // since we are in 2d, our z axis is just pointing out of the world
        glm::vec3 xAxis = contact.normal;
        glm::vec3 zAxis = glm::vec3(0, 0, 1);
        glm::vec3 yAxis = glm::cross(zAxis, xAxis);
    
        // world to contact local matrix
        return utl::axes2GLMMat(xAxis, yAxis, zAxis);    
    }










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




    float CalculateDeltaVelocity(glm::vec3 contactPoint, glm::vec3 normal,
        Entity* a, Entity* b, glm::mat4 worldToContact)
    {
        glm::vec3 relativeContactPointA = contactPoint - a->position;

        // velocity induced from angular part
        glm::vec3 closingVelocity = glm::cross(a->angularVelocity, relativeContactPointA);
        closingVelocity += a->velocity;

        if (b != NULL)
        {
            glm::vec3 relativeContactPointB = contactPoint - b->position;
            closingVelocity = glm::cross(b->angularVelocity, relativeContactPointB);
            closingVelocity += b->velocity;
        }

        glm::mat3 worldToContact3x3 = glm::mat3(worldToContact);

        glm::vec3 closingVelocityInContactCoordinate = worldToContact3x3 * closingVelocity;

        return -closingVelocityInContactCoordinate.x * (1 + 0.5);
    }


    // we will do the same thing in ResolveVelocityLinearOnly
    // we calcualte the desired separating velocity, deltaV
    // then calcualte the impulse for each contact point 

    // TODO: accout for physbody offset
    glm::vec3 CalculateFrictionlessImpulse(ContactPoint& cp, glm::mat4 worldToContact, Entity* a, Entity* b)
    {
        glm::vec3 deltaVelocityPerUnitImpulseWorldCoordinate = glm::cross(cp.relativeContactPositions[0], cp.normal);
        deltaVelocityPerUnitImpulseWorldCoordinate = a->inverseInertiaTensor * deltaVelocityPerUnitImpulseWorldCoordinate;
        deltaVelocityPerUnitImpulseWorldCoordinate = glm::cross(deltaVelocityPerUnitImpulseWorldCoordinate, cp.relativeContactPositions[0]);

        float deltaVelocityPerUnitImpulse = glm::dot(deltaVelocityPerUnitImpulseWorldCoordinate, cp.normal);

        deltaVelocityPerUnitImpulse += a->invMass;

        if (b != NULL)
        {
            // ######### should I use -normal here?
            deltaVelocityPerUnitImpulseWorldCoordinate = glm::cross(cp.relativeContactPositions[1], -cp.normal);
            deltaVelocityPerUnitImpulseWorldCoordinate = b->inverseInertiaTensor * deltaVelocityPerUnitImpulseWorldCoordinate;
            deltaVelocityPerUnitImpulseWorldCoordinate = glm::cross(deltaVelocityPerUnitImpulseWorldCoordinate, cp.relativeContactPositions[1]);

            deltaVelocityPerUnitImpulse += glm::dot(deltaVelocityPerUnitImpulseWorldCoordinate, -cp.normal);
            deltaVelocityPerUnitImpulse += b->invMass;
        }

        
        // compute desired separating velocity
        float deltaVelocity = CalculateDeltaVelocity(cp.position, cp.normal, a, b, worldToContact);

        glm::vec3 impulse(deltaVelocity / deltaVelocityPerUnitImpulse, 0, 0);


        impulse = glm::vec3(glm::inverse(worldToContact) * glm::vec4(impulse.x, impulse.y, impulse.z, 0));
        return impulse;
    }





    void ResolveVelocityForConactPoint(ContactPoint& cp, glm::mat4 worldToContact, Entity* a, Entity* b)
    {
        glm::vec3 impulse = CalculateFrictionlessImpulse(cp, worldToContact, a, b);

        glm::vec3 velocityChange = impulse * a->invMass;
        glm::vec3 impulsiveTorque = glm::cross(cp.relativeContactPositions[0], impulse);

        glm::vec3 angularVelocityChange = a->inverseInertiaTensor * impulsiveTorque;

        a->velocity += velocityChange;
        a->angularVelocity += angularVelocityChange;


        utl::debug("impulse ", impulse);
        utl::debug("impulsiveTorque ", impulsiveTorque);
        utl::debug("velocityChange ", velocityChange);
        utl::debug("angularVelocityChange ", angularVelocityChange);

        if (b != NULL)
        {

        }

    }

    // move both objects in the direction of the contact normal until they are no longer 
    // interpenetrating. The movement is both linear and augular
    // penetration depth of the contact is the total amount of movement we need to resolve
    // our goal is to find the proportion of this movement that will be contributed by linear and angular motion
    // for each object.
    void ResolveInterpenetrationForContactPoint(ContactPoint& cp, glm::mat4 worldToContact, Entity* a, Entity* b)
    {
        Entity* bodies[2] = { a, b };

        float linearInertia[2];
        float angularInertia[2];

        
        float totalInertia = 0;
        for (int i = 0; i < 2; i++)
        {
            if (bodies[i])
            {
                glm::vec3 impulsiveTorque = glm::cross(cp.relativeContactPositions[i], cp.normal);
                glm::vec3 angularChange = a->inverseInertiaTensor * impulsiveTorque;
                glm::vec3 angularInducedVelocity = glm::cross(angularChange, cp.relativeContactPositions[i]);

                angularInertia[i] = glm::dot(angularInducedVelocity, cp.normal);
                linearInertia[i] = bodies[i]->invMass;

                totalInertia += linearInertia[i] + angularInertia[i];
            }
        }


        float inverseTotalInertia = 1 / totalInertia;
        float linearMove[2];
        float angularMove[2];
        int sign = 0;
        for (int i = 0; i < 2; i++)
        {
            if (bodies[i])
            {
                sign = 1 - 2 * i;
                linearMove[i] = sign * cp.penetration * linearInertia[i] * inverseTotalInertia;
                angularMove[i] = sign * cp.penetration * angularInertia[i] * inverseTotalInertia;
            
            
                bodies[i]->position += linearMove[i] * cp.normal;


                glm::vec3 impulsiveTorque = glm::cross(cp.relativeContactPositions[i], cp.normal);
                glm::vec3 angularChangePerUnitImpulseTorque = bodies[i]->inverseInertiaTensor * impulsiveTorque;



                bodies[i]->updateOrientation(, 1.0f);

            }
        }

        
    }


    void PrepareContactPoint(ContactPoint& cp, glm::vec3 contactPoint, glm::vec3 normal, float penetration,
                            Entity* a, Entity* b)
    {
        cp.position = contactPoint;
        cp.normal = normal;
        cp.penetration = penetration;

        cp.relativeContactPositions[0] = contactPoint - a->position;

        if (b != NULL)
        {
            cp.relativeContactPositions[1] = contactPoint - b->position;
        }
    }



    // equation 9.5 is crucial
    // q_vel = angular_velocity x (q_pos - object_origin) + object_velo [9.5]

    void Resolve(CollisionData& contact, Entity* a, Entity* b)
    {

        // this is assuming all contacts have the same basis
        glm::mat4 worldToContact = CreateContactCoordinateBasis(contact);

        for (int i = 0; i < contact.numContactPoints; i++)
        {
            ContactPoint cp = {};
            PrepareContactPoint(cp, contact.contactPoints[i], contact.normal, contact.penetration, a, b);

            ResolveVelocityForConactPoint(cp, worldToContact, a, b);
            ResolveInterpenetrationForContactPoint(cp, worldToContact, a, b);
        }
    }



    void GenerateContactInfo(Entity a, Entity b, CollisionData& contact)
    {        
        if (a.entityType == EntityType::Floor && b.entityType == EntityType::Box)
        {
            glm::vec3 bCenter = b.position + b.physBody.obb.center;
            PhysBodyTransform bTransform = { bCenter , b.orientationMat};
            PhysBodyTransform aTransform = { a.position, a.orientationMat };

            GetOBBPlaneContacts(b.physBody.obb, bTransform, a.physBody.plane, aTransform, contact);
        }
        else if (a.entityType == EntityType::Box && b.entityType == EntityType::Floor)
        {
            glm::vec3 aCenter = a.position + a.physBody.obb.center;
            PhysBodyTransform aTransform = { aCenter , a.orientationMat };
            PhysBodyTransform bTransform = { b.position , b.orientationMat };

            GetOBBPlaneContacts(a.physBody.obb, aTransform, b.physBody.plane, bTransform, contact);
        }
    };

};

#endif