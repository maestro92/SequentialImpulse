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

    float restitution = 0.0;

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
    void ResolveVelocityLinearOnly(ContactPoint& cp, Entity* a, Entity* b)
    {
        float separatingVelocity = CalculateSeparatingVelocity(a, b, cp.normal);

        if (separatingVelocity > 0)
        {
            // no impulse required
            return;
        }

        float newSeparatingVelocity = -separatingVelocity;// *0.5f;
                
        float deltaVelocity = newSeparatingVelocity - separatingVelocity;

        float totalInverseMass = a->invMass;
        if (!b->flags & EntityFlag_Static)
        {
            totalInverseMass += b->invMass;
        }

        if (totalInverseMass <= 0)
            return;

        float impulse = deltaVelocity / totalInverseMass;
        glm::vec3 impulsePerInvMass = cp.normal * impulse;

        a->velocity = a->velocity + impulsePerInvMass * a->invMass;

        if (!(b->flags & EntityFlag_Static))
        {
            b->velocity = b->velocity - impulsePerInvMass * b->invMass;
        }

        return;
    }

    void ResolveInterpenetrationLinearOnly(ContactPoint& cp, Entity* a, Entity* b)
    {
        if (cp.penetration < 0)
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

        glm::vec3 movePerInvMass = cp.normal * (cp.penetration / totalInverseMass);

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

    glm::mat4 CreateContactCoordinateBasis(ContactPoint& cp)
    {
        // page 311
        // since we are in 2d, our z axis is just pointing out of the world
        glm::vec3 xAxis = cp.normal;
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


    /*
    our current steps are 

        add gravity and drag
 
        Create contacts

        integrate


    so in the create contact step, if we were to remove velocity induced from acceleration
    we need to use last frame's acceleration. because this current frame's collision
    is caused by last frame's acceleration and velocity.
    */
#define DEBUGGING 0

    glm::vec3 computeClosingVelocityInContactCoordinates(ContactPoint& cp, Entity* a, Entity* b)
    {
        // velocity induced from angular part
        glm::vec3 closingVelocity = glm::cross(a->angularVelocity, cp.relativeContactPositions[0]);

     //   utl::debug("               velocity from rotation ", closingVelocity);


        closingVelocity += a->velocity;
     //   utl::debug("               velocity ", a->velocity);

        if (b != NULL)
        {
            closingVelocity = glm::cross(b->angularVelocity, cp.relativeContactPositions[1]);
            closingVelocity += b->velocity;
        }
#if DEBUGGING
        utl::debug("               worldToContact3x3 ", cp.worldToContact);
#endif
        glm::mat3 worldToContact3x3 = glm::mat3(cp.worldToContact);

#if DEBUGGING
        utl::debug("               a->velocity ", a->velocity);
        utl::debug("               worldToContact3x3 ", worldToContact3x3);
#endif

        return worldToContact3x3 * closingVelocity;
    }




    float CalculateDesiredDeltaVelocity(ContactPoint& cp, Entity* a, Entity* b, float dt_s)
    {
        // utl::debug("               worldToContact3x3 ", worldToContact3x3);
        // utl::debug("               closingVelocityInContactCoordinate ", closingVelocityInContactCoordinate);
        const static float velocityLimit = 0.25;

        // calculate the acceleration induced velocity
        float velocityFromAcc = glm::dot(a->acceleration, cp.normal) * dt_s;

        if (b != NULL)
        {
            velocityFromAcc -= glm::dot(b->acceleration, cp.normal) * dt_s;
        }

        float usedRestitution = restitution;
        if (abs(cp.closingVelocity.x) < velocityLimit)
        {
            usedRestitution = 0;
        }
#if DEBUGGING
        utl::debug("               cp.closingVelocity ", cp.closingVelocity.x);
        utl::debug("               usedRestitution ", usedRestitution);
        utl::debug("               velocityFromAcc ", velocityFromAcc);

        float result = -cp.closingVelocity.x - usedRestitution * (cp.closingVelocity.x - velocityFromAcc);
        utl::debug("               final delta Velo ", result);
#endif
        return -cp.closingVelocity.x - usedRestitution * (cp.closingVelocity.x - velocityFromAcc);
    }


    // we will do the same thing in ResolveVelocityLinearOnly
    // we calcualte the desired separating velocity, deltaV
    // then calcualte the impulse for each contact point 

    // TODO: accout for physbody offset
    glm::vec3 CalculateFrictionlessImpulse(ContactPoint& cp, Entity* a, Entity* b, float dt_s)
    {
        glm::vec3 velocityFromRotPerUnitImpulse = glm::cross(cp.relativeContactPositions[0], cp.normal);
        velocityFromRotPerUnitImpulse = a->inverseInertiaTensor * velocityFromRotPerUnitImpulse;
        velocityFromRotPerUnitImpulse = glm::cross(velocityFromRotPerUnitImpulse, cp.relativeContactPositions[0]);

        float deltaVelocityPerUnitImpulse = glm::dot(velocityFromRotPerUnitImpulse, cp.normal);
    //    utl::debug("               rot ", deltaVelocityPerUnitImpulse);
        deltaVelocityPerUnitImpulse += a->invMass;
    //    utl::debug("               linear ", a->invMass);
        if (b != NULL)
        {
            // ######### should I use -normal here?
            velocityFromRotPerUnitImpulse = glm::cross(cp.relativeContactPositions[1], -cp.normal);
            velocityFromRotPerUnitImpulse = b->inverseInertiaTensor * velocityFromRotPerUnitImpulse;
            velocityFromRotPerUnitImpulse = glm::cross(velocityFromRotPerUnitImpulse, cp.relativeContactPositions[1]);

            deltaVelocityPerUnitImpulse += glm::dot(velocityFromRotPerUnitImpulse, -cp.normal);
            deltaVelocityPerUnitImpulse += b->invMass;
        }

        
        // compute desired separating velocity
    //    float deltaVelocity = CalculateDesiredDeltaVelocity(cp, a, b, dt_s);
    //    utl::debug("               closingVelocity ", cp.closingVelocity);
    //    utl::debug("               deltaVelocity ", cp.desiredDeltaVelocity);
        glm::vec3 impulse(cp.desiredDeltaVelocity / deltaVelocityPerUnitImpulse, 0, 0);


        impulse = glm::vec3(glm::inverse(cp.worldToContact) * glm::vec4(impulse.x, impulse.y, impulse.z, 0));
        return impulse;
    }





    void ResolveVelocityForContactPoint(ContactPoint& cp, Entity* a, Entity* b, glm::vec3 linearChange[2], glm::vec3 angularChange[2], float dt_s)
    {
        glm::vec3 impulse = CalculateFrictionlessImpulse(cp, a, b, dt_s);
#if DEBUGGING
        utl::debug("                    impulse ", impulse);
        utl::debug("                    a->velocity ", a->velocity);
        utl::debug("                    a->angularVelocity ", a->angularVelocity);
#endif       

        glm::vec3 velocityChange = impulse * a->invMass;

        glm::vec3 impulsiveTorque = glm::cross(cp.relativeContactPositions[0], impulse);
        glm::vec3 angularVelocityChange = a->inverseInertiaTensor * impulsiveTorque;

        linearChange[0] = velocityChange;
        angularChange[0] = angularVelocityChange;
        a->velocity += velocityChange;
        a->angularVelocity += angularVelocityChange;

        
#if DEBUGGING
        utl::debug("                        impulsiveTorque ", impulsiveTorque);
        utl::debug("                        velocityChange ", velocityChange);
        utl::debug("                        angularVelocityChange ", angularVelocityChange);
        utl::debug("                        after ", 0);
        utl::debug("                        a->velocity ", a->velocity);
        utl::debug("                        a->angularVelocity ", a->angularVelocity);
#endif       

        if (b != NULL)
        {

        }

    }

    // move both objects in the direction of the contact normal until they are no longer 
    // interpenetrating. The movement is both linear and augular
    // penetration depth of the contact is the total amount of movement we need to resolve
    // our goal is to find the proportion of this movement that will be contributed by linear and angular motion
    // for each object.
    void ResolveInterpenetrationForContactPoint(ContactPoint& cp, Entity* a, Entity* b,
                                                glm::vec3 linearChange[2], glm::vec3 angularChange[2])
    {
        Entity* bodies[2] = { a, b };

        float linearInertia[2];
        float angularInertia[2];

    //    utl::debug("            @@@@@@ cp Penetration", cp.penetration);
        float totalInertia = 0;
        for (int i = 0; i < 2; i++)
        {
            if (bodies[i])
            {
                glm::vec3 impulsiveTorque = glm::cross(cp.relativeContactPositions[i], cp.normal);
                glm::vec3 angularChange = a->inverseInertiaTensor * impulsiveTorque;
                glm::vec3 angularInducedVelocity = glm::cross(angularChange, cp.relativeContactPositions[i]);
#if DEBUGGING                
                utl::debug("                cp.relativeContactPositions[i] ", cp.relativeContactPositions[i]);
                utl::debug("                impulsiveTorque ", impulsiveTorque);
                utl::debug("                angularChange[i] ", angularChange);
                utl::debug("                angularInducedVelocity[i] ", angularInducedVelocity);
#endif                
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
#if DEBUGGING
                cout << "               linearInertia[i] " << linearInertia[i] << endl;
                cout << "               angularInertia[i] " << angularInertia[i] << endl;
#endif

                linearChange[i] = cp.normal * linearMove[i];
                bodies[i]->position += linearChange[i];

                if (angularMove[i] != 0)
                {
                    glm::vec3 impulsiveTorque = glm::cross(cp.relativeContactPositions[i], cp.normal);
                    glm::vec3 angularChangePerUnitImpulseTorque = bodies[i]->inverseInertiaTensor * impulsiveTorque;

#if DEBUGGING                    
                    utl::debug("                impulsiveTorque", impulsiveTorque);
                    utl::debug("                angularChangePerUnitImpulseTorque", angularChangePerUnitImpulseTorque);
                    utl::debug("                cp.relativeContactPositions[i]", cp.relativeContactPositions[i]);

                    utl::debug("                angularMove", angularMove[i]);
                    utl::debug("                angularInertia", angularInertia[i]);
#endif
                    // this is the impulses needed to cover angularMove amount of distance
                    float impulseNeeded = angularMove[i] / angularInertia[i];

                    glm::vec3 rotation = impulseNeeded * angularChangePerUnitImpulseTorque;


#if DEBUGGING
                    utl::debug("                rotation", rotation);
#endif

                    if (glm::length(rotation) > 0.01)
                    {
                        angularChange[i] = rotation;
                        bodies[i]->updateOrientation(rotation, 1.0f);
                    }
                    else
                    {
                        angularChange[i] = glm::vec3(0,0,0);
                    }
#if DEBUGGING
                    utl::debug("                bodies[i] orientation", bodies[i]->orientation);
                    utl::debug("                bodies[i] rot", bodies[i]->orientationMat);
#endif
                }
            }
        }

        
    }


    void PrepareContactPoint(ContactPoint& cp, Entity* a, Entity* b, float dt_s)
    {
        if (abs(cp.penetration) < 0.0001)
        {
            cp.penetration = 0;
        }

        cp.worldToContact = CreateContactCoordinateBasis(cp);
       
        cp.relativeContactPositions[0] = cp.position - a->position;

        if (b != NULL)
        {
            cp.relativeContactPositions[1] = cp.position - b->position;
        }
#if DEBUGGING
        utl::debug("a.velocity", a->velocity);
#endif


        cp.closingVelocity = computeClosingVelocityInContactCoordinates(cp, a, b);


#if DEBUGGING
        utl::debug("cp.closingVelocity", cp.closingVelocity);
        utl::debug("cp.worldToContact", cp.worldToContact);
#endif


        cp.desiredDeltaVelocity = CalculateDesiredDeltaVelocity(cp, a, b, dt_s);
    }

    // to calcualte the new penetration value, we calculate the new position of the relative contact point
    // for each object, based on the linear and angular movements we applied.

    // i am assuming all the contact points come from entity a or entity b
    // not sure if this is the right assumption?
    void UpdatePenetrations(ContactPoint* cp, int numContactPoints, Entity* a, Entity* b, 
                            glm::vec3 linearChange[2], glm::vec3 angularChange[2])
    {
        Entity* bodies[2] = { a, b };
        for (int j = 0; j < 2; j++)
        {
            if (bodies)
            {
                int sign = j ? 1 : -1;

                glm::vec3 deltaPosition = linearChange[j] + glm::cross(angularChange[j], cp->relativeContactPositions[j]);
                cp->penetration += sign * glm::dot(deltaPosition, cp->normal);
            }
        }
    }



    void UpdateContactVelocityForContactPoint(ContactPoint& cp, Entity* a, Entity* b, 
                                                glm::vec3 linearChange[2], glm::vec3 angularChange[2], float dt_s)
    {
#if DEBUGGING
        utl::debug("        UpdateContactVelocityForContactPoint", cp.position);

        utl::debug("        UpdateContactVelocityForContactPoint", cp.closingVelocity);

        glm::vec3 worldClosingVel = cp.ContactToWorld() * cp.closingVelocity;
        utl::debug("        world coord", worldClosingVel);
#endif
        Entity* bodies[2] = { a, b };
        for (int i = 0; i < 2; i++)
        {
            if (bodies[i])
            {
                int sign = 1 - 2 * i;
                // change in contact velocity
                glm::vec3 deltaVelocity = linearChange[i] + glm::cross(angularChange[i], cp.relativeContactPositions[i]);

#if DEBUGGING
                utl::debug("                closingVelocity", cp.closingVelocity);
                utl::debug("                deltaVelocity", deltaVelocity);
                utl::debug("                linearChange[i]", linearChange[i]);
                //glm::mat3 m = glm::mat3(glm::inverse(cp.worldToContact));

                utl::debug("                angularChange[i]", angularChange[i]);
                utl::debug("                relativeContactPositions", cp.relativeContactPositions[i]);
                utl::debug("                velocity from angularChange[i]", glm::cross(angularChange[i], cp.relativeContactPositions[i]));
#endif
                glm::vec3 deltaVelocityContactCoord = glm::mat3(cp.worldToContact) * deltaVelocity;
      
#if DEBUGGING

                utl::debug("                deltaVelocityContactCoord", deltaVelocityContactCoord);
#endif
                cp.closingVelocity += sign * glm::mat3(cp.worldToContact) * deltaVelocity;

#if DEBUGGING
                utl::debug("                closingVelocity", cp.closingVelocity);
#endif

                glm::vec3 worldClosingVel2 = cp.ContactToWorld() * cp.closingVelocity;

#if DEBUGGING
                utl::debug("                world coord2", worldClosingVel2);
#endif

                cp.desiredDeltaVelocity = CalculateDesiredDeltaVelocity(cp, a, b, dt_s);
            }
        }
    }

    // for the two bodies just involved in the the contact Which we just resolved, 
    // if they were invovled in other contacts, we have to recompute the closing velocities
    void UpdateContactVelocities(ContactPoint* contactPoints, int numContactPoints, ContactPoint* contactPointJustResolved, 
                                Entity* a, Entity* b, glm::vec3 linearChange[2], glm::vec3 angularChange[2], float dt_s)
    {
        for (int i = 0; i < numContactPoints; i++)
        {
            UpdateContactVelocityForContactPoint(contactPoints[i], a, b, linearChange, angularChange, dt_s);

            // recompute cp desired deltaVelocity

        }
        
        /*
        for (int j = 0; j < 2; j++)
        {
            if (bodies)
            {
                int sign = j ? 1 : -1;

                glm::vec3 deltaPosition = linearChange[j] + glm::cross(angularChange[j], cp->relativeContactPositions[j]);
                cp->penetration += sign * glm::dot(deltaPosition, cp->normal);

                if (abs(cp->penetration) < 0.0001)
                {
                    cp->penetration = 0;
                }
            }
        }
        */
    }


    ContactPoint* FindContactWithMostPenetration(ContactPoint* contactPoints, int numContactPoints)
    {
        ContactPoint* worstContact = NULL;
        float worstPenetration = 0.01;
        // find worst contact
        for (int j = 0; j < numContactPoints; j++)
        {
            if (contactPoints[j].penetration > worstPenetration)
            {
                worstContact = &contactPoints[j];
                worstPenetration = contactPoints[j].penetration;
            }
        }

        return worstContact;
    }


    ContactPoint* FindContactWithMostDesiredVelocityChange(ContactPoint* contactPoints, int numContactPoints)
    {
        ContactPoint* worstContact = NULL;
        float max = -0.01;
        // find worst contact
        for (int j = 0; j < numContactPoints; j++)
        {
            if (contactPoints[j].desiredDeltaVelocity < max)
            {
                worstContact = &contactPoints[j];
                max = contactPoints[j].desiredDeltaVelocity;
            }
        }

        return worstContact;
    }



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


    // equation 9.5 is crucial
    // q_vel = angular_velocity x (q_pos - object_origin) + object_velo [9.5]

    void Resolve(CollisionData& contact, Entity* a, Entity* b, float dt_s)
    {

        // this is assuming all contacts have the same basis
        for (int i = 0; i < contact.numContactPoints; i++)
        {
            PrepareContactPoint(contact.contactPoints[i], a, b, dt_s);
        }
        
#if DEBUGGING
        cout << "########## resolving " << contact.numContactPoints << " contact points" << endl;

        for (int i = 0; i < contact.numContactPoints; i++)
        {
            cout << "           penetration at start up " << contact.contactPoints[i].penetration << endl;
        }
#endif

        glm::vec3 linearChange[2];
        glm::vec3 angularChange[2];

        
        int iterations = contact.numContactPoints * 4;
        for (int i = 0; i < iterations; i++)
        {
#if DEBUGGING
            cout << "       >>>>>>> iteration " << i << endl;
#endif            
            ContactPoint* worstContact = FindContactWithMostPenetration(contact.contactPoints, contact.numContactPoints);
            if (worstContact == NULL)
            {
                break;
            }
#if DEBUGGING
            utl::debug("                worstContact position", worstContact->position);
            utl::debug("                worstContact penetration", worstContact->penetration);
#endif

            CheckAwakeState(a, b);

            ResolveInterpenetrationForContactPoint(*worstContact, a, b, linearChange, angularChange);

            UpdatePenetrations(worstContact, contact.numContactPoints, a, b, linearChange, angularChange);
        }
        
#if DEBUGGING
        cout << "########## resolving velocity for each contact points" << endl;

#endif

        glm::vec3 linearVelocityChange[2];
        glm::vec3 angularVelocityChange[2];

        iterations = 4;
        for (int i = 0; i < iterations; i++)
        {
            ContactPoint* worstContact = FindContactWithMostDesiredVelocityChange(contact.contactPoints, contact.numContactPoints);          

            if (worstContact == NULL)
            {
                break;
            }


#if DEBUGGING
            cout << "       >>>>>>> Resolving velocity iteration " << i << endl;
            utl::debug("                    resolving velocity for contact", worstContact->position);
            utl::debug("                    desiredDeltaVelocity", worstContact->desiredDeltaVelocity);
#endif    

            CheckAwakeState(a, b);

            ResolveVelocityForContactPoint(*worstContact, a, b, linearVelocityChange, angularVelocityChange, dt_s);


            UpdateContactVelocities(contact.contactPoints, contact.numContactPoints, worstContact, 
                a, b, linearVelocityChange, angularVelocityChange, dt_s);
            
#if DEBUGGING
            cout << "\n\n" << endl;
#endif       

        }



        /*
        iterations = 1;
        for (int i = 0; i < iterations; i++)
        {
            cout << "       >>>>>>> Resolving velocity iteration " << i << endl;
            for (int j = 0; j < contact.numContactPoints; j++)
            {
                utl::debug("                resolving velocity for contact", contact.contactPoints[j].position);
                ResolveVelocityForContactPoint(contact.contactPoints[j], a, b, dt_s);
                cout << "\n\n" << endl;

            }
        }
        */

        //adjustPositions();

        //adjustVelocities();


        /*
        for (int i = 0; i < contact.numContactPoints; i++)
        {
            ContactPoint cp = {};
            PrepareContactPoint(cp, contact.contactPoints[i], contact.normal, contact.penetration, a, b);

            ResolveVelocityForConactPoint(cp, worldToContact, a, b);
            ResolveInterpenetrationForContactPoint(cp, worldToContact, a, b);
        }
        */
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