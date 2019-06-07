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
    void ResolveVelocity(CollisionData& contact, Entity* a, Entity* b)
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

    void ResolveInterpenetration(CollisionData& contact, Entity* a, Entity* b)
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


    void Resolve(CollisionData& contact, Entity* a, Entity* b)
    {
        ResolveVelocity(contact, a, b);
        ResolveInterpenetration(contact, a, b);
    }


    /*
    bool TestContactInfo(Entity a, Entity b, CollisionData& contact)
    {
        if (a.entityType == EntityType::Floor && b.entityType == EntityType::Box)
        {
            glm::vec3 bCenter = b.position + b.physBody.obb.center;
            PhysBodyTransform bTransform = { bCenter , b.orientation };
            PhysBodyTransform aTransform = { a.position, a.orientation };

            return TestOBBPlane(b.physBody.obb, bTransform, a.physBody.plane, aTransform);
        }
        else if (a.entityType == EntityType::Box && b.entityType == EntityType::Floor)
        {
            glm::vec3 aCenter = a.position + a.physBody.obb.center;
            PhysBodyTransform aTransform = { aCenter , a.orientation };
            PhysBodyTransform bTransform = { b.position , b.orientation };

            return TestOBBPlane(a.physBody.obb, aTransform, b.physBody.plane, bTransform);
        }
        return false;
    };
    */

    void GenerateContactInfo(Entity a, Entity b, CollisionData& contact)
    {        
        if (a.entityType == EntityType::Floor && b.entityType == EntityType::Box)
        {
            glm::vec3 bCenter = b.position + b.physBody.obb.center;
            PhysBodyTransform bTransform = { bCenter , b.orientation};
            PhysBodyTransform aTransform = { a.position, a.orientation };

            GetOBBPlaneContacts(b.physBody.obb, bTransform, a.physBody.plane, aTransform, contact);
        }
        else if (a.entityType == EntityType::Box && b.entityType == EntityType::Floor)
        {
            glm::vec3 aCenter = a.position + a.physBody.obb.center;
            PhysBodyTransform aTransform = { aCenter , a.orientation };
            PhysBodyTransform bTransform = { b.position , b.orientation };

            GetOBBPlaneContacts(a.physBody.obb, aTransform, b.physBody.plane, bTransform, contact);
        }
    };

};

#endif