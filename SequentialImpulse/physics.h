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

    int TestOBBPlane(OBB b, glm::mat4 rot, glm::vec3 bCenter, Plane p, ContactInfo& contact)
    {
        glm::vec3 realAxes[3] = { glm::vec3(rot * glm::vec4(b.axes[0], 0)),
            glm::vec3(rot * glm::vec4(b.axes[1], 0)),
            glm::vec3(rot * glm::vec4(b.axes[2], 0)) };
        
        float r = b.halfEdges[0] * abs(glm::dot(p.normal, realAxes[0])) +
                b.halfEdges[1] * abs(glm::dot(p.normal, realAxes[1])) +
                b.halfEdges[2] * abs(glm::dot(p.normal, realAxes[2]));

        /*
        float r = b.halfEdges[0] * abs(glm::dot(p.normal, b.axes[0])) +
            b.halfEdges[1] * abs(glm::dot(p.normal, b.axes[1])) +
            b.halfEdges[2] * abs(glm::dot(p.normal, b.axes[2]));
        */
        
        /*
        cout << "r " << r << endl;
        utl::debug("bCenter", bCenter);
        utl::debug("b.halfEdges", b.halfEdges);
        */


        // distance from box center to the plane
//        float s = glm::dot(p.normal, b.center) - p.d;
        float s = glm::dot(p.normal, bCenter) - p.d;

        // ########### DONT THINK THE normal is RIGHT ################ //
        contact.normal = p.normal;

        return abs(s) <= r;
    };


    int TestSphereOBB()
    {
        return 0;
    }


    void ResolveInterpenetration(ContactInfo& contact, Entity* a, Entity* b)
    {

    }

    void ResolveVelocity(ContactInfo& contact, Entity* a, Entity* b)
    {

    }


    int Resolve(ContactInfo& contact, Entity* a, Entity* b)
    {
        float force = 1;
        if (!(a->flags & EntityFlag_Static))
        {
            a->position += 1.0f * contact.normal;
            a->velocity += force * contact.normal;
        }

        if (!(b->flags & EntityFlag_Static))
        {
            b->position -= 1.0f * contact.normal;
            b->velocity -= force * contact.normal;
        }
        return 0;
    }

    bool GenerateContactInfo(Entity a, Entity b, ContactInfo& contact)
    {        
        if (a.entityType == EntityType::Floor && b.entityType == EntityType::Box)
        {
            glm::vec3 bCenter = b.physBody.obb.center + b.position;            
            return TestOBBPlane(b.physBody.obb, b.orientation, bCenter, a.physBody.plane, contact);            
        }
        else if (a.entityType == EntityType::Box && b.entityType == EntityType::Floor)
        {
            glm::vec3 aCenter = a.physBody.obb.center + a.position; 
            return TestOBBPlane(a.physBody.obb, a.orientation, aCenter, b.physBody.plane, contact);
        }
        
        return false;
    };

};

#endif