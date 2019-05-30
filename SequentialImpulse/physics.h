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
    int TestOBBPlane(OBB b, glm::vec3 bCenter, Plane p, ContactInfo& contact)
    {
        float r = b.halfEdges[0] * abs(glm::dot(p.normal, b.axes[0])) +
            b.halfEdges[1] * abs(glm::dot(p.normal, b.axes[1])) +
            b.halfEdges[2] * abs(glm::dot(p.normal, b.axes[2]));
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


    int Resolve(ContactInfo& contact, Entity* a, Entity* b)
    {
        float force = 5;
        if (!(a->flags & EntityFlag_Static))
        {
            a->velocity += force * contact.normal;
        }

        if (!(b->flags & EntityFlag_Static))
        {
            b->velocity -= force * contact.normal;
        }
        return 0;
    }

    bool GenerateContactInfo(Entity a, Entity b, ContactInfo& contact)
    {        
        if (a.entityType == EntityType::Floor && b.entityType == EntityType::Box)
        {
            glm::vec3 bCenter = b.physBody.obb.center + b.position;            
            return TestOBBPlane(b.physBody.obb, bCenter, a.physBody.plane, contact);            
        }
        else if (a.entityType == EntityType::Box && b.entityType == EntityType::Floor)
        {
            glm::vec3 aCenter = a.physBody.obb.center + a.position; 
            return TestOBBPlane(a.physBody.obb, aCenter, b.physBody.plane, contact);
        }
        
        return false;
    };

};

#endif