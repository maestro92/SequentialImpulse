#pragma once


#include "entity.h"
#include "physics_core.h"

namespace Physics
{
    const int MAX_CONTACT_POINTS = 16;



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


    // http://box2d.org/files/GDC2015/DirkGregorius_Contacts.pdf








    // assuming all contact points have the same normal. 
    // for this application, this may be true
    // for for advance complicated convex shape, this wont be
    struct ContactManifold
    {
        enum Type
        {

            FACE_A,
            FACE_B,
        };


        //    glm::vec3 normal;

        int numContactPoints;
        ContactPoint contactPoints[MAX_CONTACT_POINTS];
        Type type;
        PhysBody* a;
        PhysBody* b;
        //    glm::vec3 contactPoints[16];

     //   float penetration;

        ContactManifold()
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



    void TestSphereSphere(ContactManifold& contact)
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


    void GetSpherePlaneContacts(Sphere s, PhysBody* bBody, Plane p, PhysBody* pBody, ContactManifold& contact)
    {

    }




    float ProjectToAxis(OBB b, glm::vec3 axes[3], glm::vec3 testAxis)
    {
        return b.halfEdges[0] * abs(glm::dot(testAxis, axes[0])) +
            b.halfEdges[1] * abs(glm::dot(testAxis, axes[1])) +
            b.halfEdges[2] * abs(glm::dot(testAxis, axes[2]));
    }

    float TestOverlapOnAxis(OBB a, glm::vec3 aAxes[3],
                            OBB b, glm::vec3 bAxes[3], glm::vec3 aTob, glm::vec3 testAxis)
    {
        // Project the half-size of one onto axis
        float projectionA = ProjectToAxis(a, aAxes, testAxis);
        float projectionB = ProjectToAxis(b, bAxes, testAxis);

        // Project this onto the axis
        float distance = abs(glm::dot(aTob, testAxis));

        // Check for overlap
        return (distance < projectionA + projectionB);
    }


    /*
    in 2D, the test cam be performed by checking if the vertices of box A are all on the outside of the planes defined by the faces of box B
    and vice verca.

    in 3D, this wont work, we have to a bunch of separating axis test.
    we need to make 15 separating axes test to determine if OBB overlap.

    3 axes of A
    3 axes of B
    nine axes perpendicular to an axis from each. 

    */

    bool TestOBBOBB(OBB a, glm::vec3 aPos, glm::vec3 aAxes[3],
                    OBB b, glm::vec3 bPos, glm::vec3 bAxes[3])
    {
        glm::vec3 aTob = bPos - aPos;

        /*
        bool b0 = TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, aAxes[0]);
        bool b1 = TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, aAxes[1]);
        bool b2 = TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, aAxes[2]);

        bool b3 = TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, bAxes[0]);
        bool b4 = TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, bAxes[1]);
        bool b5 = TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, bAxes[2]);

        bool b6 = TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[0], bAxes[0]));
        bool b7 = TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[0], bAxes[1]));
        bool b8 = TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[0], bAxes[2]));

        bool b9 = TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[1], bAxes[0]));
        bool ba = TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[1], bAxes[1]));
        bool bb = TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[1], bAxes[2]));

        bool bc = TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[2], bAxes[0]));
        bool bd = TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[2], bAxes[1]));
        bool be = TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[2], bAxes[2]));

        // its possible that two axis are the same: aAxes[0] == bAxes[0]

        return TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, aAxes[0]) &&
            TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, aAxes[1]) &&
            TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, aAxes[2]) &&

            TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, bAxes[0]) &&
            TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, bAxes[1]) &&
            TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, bAxes[2]) &&

            TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[0], bAxes[0])) &&
            TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[0], bAxes[1])) &&
            TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[0], bAxes[2])) &&
            
            TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[1], bAxes[0])) &&
            TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[1], bAxes[1])) &&
            TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[1], bAxes[2])) &&

            TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[2], bAxes[0])) &&
            TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[2], bAxes[1])) &&
            TestOverlapOnAxis(a, aAxes, b, bAxes, aTob, glm::cross(aAxes[2], bAxes[2]));
            */

        return 0;
    }



    /*
    the few possible types 

    1. point-face contact
    2. edge-edge contact
    3. face-face    4 points
    4. edge-face    2 contact points
    5. point-point  ignored
    6. point-edge   ignored
    */

    /*
    int GetOBBPointFaceContact(OBB box, glm::vec3 boxPos, glm::vec3 boxAxes[3], glm::vec3 point, CollisionData& contact)
    {
        // transform the point into box coordinate
        glm::mat3 om = utl::axes2GLMMat3(boxAxes[0], boxAxes[1], boxAxes[2]);
        glm::vec3 relPt = om * point;

        glm::vec3 normal; 
        
        // check each axis, looking for the axis on which the penetration is least deep
        float minPenetration = box.halfEdges.x - abs(relPt.x);
        if (minPenetration < 0)
            return 0;
        normal = boxAxes[0] * (float)((relPt.x < 0) ? - 1 : 1);

        float penetration = 0;
        for (int i = 1; i < 3; i++)
        {
            penetration = box.halfEdges[i] - abs(relPt[i]);
            if (penetration < 0)
            {
                return 0;
            }
            else if (penetration < minPenetration)
            {
                minPenetration = penetration;
                normal = boxAxes[i] * (float)((relPt[i] < 0) ? -1 : 1);
            }
        }

        assert(contact.numContactPoints < 16);
        ContactPoint* cp = &contact.contactPoints[contact.numContactPoints];        
        cp->normal = normal;
        cp->position = point;
        cp->penetration = minPenetration;

        return 1;
    }
    */


    /*

    // https://www.randygaul.net/2014/05/22/deriving-obb-to-obb-intersection-sat/

    int GetPointFaceOBBContact(OBB a, glm::vec3 aPos, glm::vec3 aAxes[3],
                               OBB b, glm::vec3 bPos, glm::vec3 bAxes[3], int collisionAxisIndex, float penetration, ContactManifold& contact)
    {
        // transform the point into box coordinate
        glm::mat3 om = utl::axes2GLMMat3(aAxes[0], aAxes[1], aAxes[2]);
        glm::vec3 relPt = om * point;

        glm::vec3  normal = boxAxes[collisionAxisIndex] * (float)((relPt[collisionAxisIndex] < 0) ? -1 : 1);
        
        assert(contact.numContactPoints < 16);
        ContactPoint* cp = &contact.contactPoints[contact.numContactPoints];
        cp->normal = normal;
        cp->position = point;
        cp->penetration = penetration;

        return 1;
    }

    */



    


    float GetOverlapPenetrationOnAxis(OBB a, glm::vec3 aAxes[3],
        OBB b, glm::vec3 bAxes[3], glm::vec3 aTob, glm::vec3 testAxis)
    {
        // Project the half-size of one onto axis
        float projectionA = ProjectToAxis(a, aAxes, testAxis);
        float projectionB = ProjectToAxis(b, bAxes, testAxis);

        // Project this onto the axis
        float distance = abs(glm::dot(aTob, testAxis));

        // Check for overlap
        return projectionA + projectionB - distance;
    }

    // returns true if there is a separating axis so the two are not intersecting
    bool SATOnFaceAxis(OBB a, glm::vec3 aAxes[3],
                             OBB b, glm::vec3 bAxes[3], glm::vec3 aTob, glm::vec3 testAxis, 
                        float& minPenetration, glm::vec3& normal, int& minPenetrationIndex, int axisIndex)
    {
        if (glm::length2(testAxis) < 0.0001)
            return true;

        float penetration = GetOverlapPenetrationOnAxis(a, aAxes, b, bAxes, aTob, testAxis);
   //     cout << "penetration " << penetration << endl;
        if (penetration < 0)
            return true;

        if (penetration < minPenetration)
        {
            minPenetration = penetration;
            minPenetrationIndex = axisIndex;
            normal = testAxis;
        }

        return false;
    }



    LineSegment GetFace(OBB a, glm::vec3 aAxes[3], PhysBody* aBody, int axisIndex, int axisDirection)
    {
        int axisIndex2 = 1 - axisIndex;

        glm::vec3 v0 = aAxes[axisIndex] * (float)axisDirection * a.halfEdges[axisIndex] + axisIndex2 * a.halfEdges[axisIndex2];
        glm::vec3 v1 = aAxes[axisIndex] * (float)axisDirection * a.halfEdges[axisIndex] - axisIndex2 * a.halfEdges[axisIndex2];

        v0 += aBody->position;
        v1 += aBody->position;

        return { v0, v1 };
    }





    LineSegment GetIncidentFace(OBB b, glm::vec3 bAxes[3], PhysBody* bBody, glm::vec3 referenceFaceNormal)
    {
        vector <glm::vec3> normals = { bAxes[0],  -bAxes[0], bAxes[1] -bAxes[1]};
        int sign = 1;
        int axisIndex = -1;
        float minDotProduct = FLT_MAX;
        
        for (int i = 0; i < 2; i++)
        {            
            float dotProduct = glm::dot(referenceFaceNormal, bAxes[i]);
            if (dotProduct < minDotProduct)
            {
                minDotProduct = dotProduct;
                axisIndex = i;
                sign = 1;
            }


            dotProduct = glm::dot(referenceFaceNormal, -bAxes[i]);
            if (dotProduct < minDotProduct)
            {
                minDotProduct = dotProduct;
                axisIndex = i;
                sign = -1;
            }
        }

        return GetFace(b, bAxes, bBody, axisIndex, sign);
    }




    // Sutherland-Hodgman clipping
    int ClipLineSegmentToLine(glm::vec3 lineSegment[2], Plane p, glm::vec3 clippedVertices[2])
    {
        int numVerticesOut = 0;

        // compute the distance of line segment ends points to the plane
        float dist0 = glm::dot(p.normal, lineSegment[0]) - p.offset;
        float dist1 = glm::dot(p.normal, lineSegment[1]) - p.offset;

        if (dist0 <= 0.0f)
            clippedVertices[numVerticesOut++] = lineSegment[0];

        if (dist1 <= 0.0f)
            clippedVertices[numVerticesOut++] = lineSegment[1];

        // if one point is in, the other point is outside of the plane
        if (dist0 * dist1 < 0.0f)
        {
            // find the intersection point of line segment and the plane

            // the order of v0 and v1 doesn't really matter, becuz the one point that is inside the plane
            // is already added to the clippedVertices array.
            
            float interpolation = dist0 / (dist0 - dist1);

            // just doing interpolation here
            clippedVertices[numVerticesOut] = lineSegment[0] + interpolation * (lineSegment[1] - lineSegment[0]);
            numVerticesOut++;
        }

        return numVerticesOut;
    }


    void GetOBBOBBContacts(OBB a, PhysBody* aBody,
        OBB b, PhysBody* bBody, ContactManifold& contactManifold)
    {
        // early out test using principle of separating axes
        glm::vec3 aAxes[3] = { glm::vec3(aBody->orientation * glm::vec4(a.axes[0], 0)),
                               glm::vec3(aBody->orientation * glm::vec4(a.axes[1], 0)),
                               glm::vec3(aBody->orientation * glm::vec4(a.axes[2], 0)) };

        glm::vec3 bAxes[3] = { glm::vec3(bBody->orientation * glm::vec4(b.axes[0], 0)),
                               glm::vec3(bBody->orientation * glm::vec4(b.axes[1], 0)),
                               glm::vec3(bBody->orientation * glm::vec4(b.axes[2], 0)) };
        /*
        PhysBodyTransform aTransform = {};
        aTransform.position = aBody->position;
        aTransform.axes[0] = aAxes[0];
        aTransform.axes[1] = aAxes[1];
        aTransform.axes[2] = aAxes[2];


        PhysBodyTransform bTransform = {};
        bTransform.position = bBody->position;
        bTransform.axes[0] = bAxes[0];
        bTransform.axes[1] = bAxes[1];
        bTransform.axes[2] = bAxes[2];
        */



        // 2015 Dirk physics talk page 96:
        // we prefer face contacts over edge contacts and one face axis over another
        float minPenetration = FLT_MAX;
        int minPenetrationAxisIndex = -1;
        glm::vec3 normal;

        glm::vec3 aTob = bBody->position - aBody->position;


        // we find the axis of least penetration

        // face axis checks. face normals and face axis are the same
        // testing a's x axis
        if (SATOnFaceAxis(a, aAxes, b, bAxes, aTob, aAxes[0], minPenetration, normal, minPenetrationAxisIndex, 0))
            return;

        // testing a's y axis
        if (SATOnFaceAxis(a, aAxes, b, bAxes, aTob, aAxes[1], minPenetration, normal, minPenetrationAxisIndex, 1))
            return;

        // the face normals of polyhedron B
        // testing b's x axis
        if (SATOnFaceAxis(a, aAxes, b, bAxes, aTob, bAxes[0], minPenetration, normal, minPenetrationAxisIndex, 2))
            return;

        // testing b's y axis
        if (SATOnFaceAxis(a, aAxes, b, bAxes, aTob, bAxes[1], minPenetration, normal, minPenetrationAxisIndex, 3))
            return;

        //    assert(b0 && b1 && b2 && b3 &&b4 && b5 && b6 && b7 && b8 && b9 && ba && bb && bc && bd && bd);

        assert(0 <= minPenetrationAxisIndex && minPenetrationAxisIndex < 4);

        /*
        need to correct the direction of the normal if necessary
        see example below:

        lets say we tested the x-axis, and we set n to be xAxis
        if b is actually opposite of n, then we need to flip the sign

             ___________         ___________
            |           | n     |           |
            |     A     |---->  |     B     |
            |___________|       |___________|

             ___________         ___________
            |           |       |           |
            |     B     |       |     A     |----->
            |___________|       |___________|
        
        we do that check by doing a dot product of vectorA2B and normal
        */
        int axisDirection = 1;
        if (glm::dot(normal, aTob) < 0.0f)
        {
            normal = -normal;
            axisDirection = -1;
        }

        LineSegment referenceFace;
        LineSegment incidentFace;

        OBB referenceOBB;
        glm::vec3* referencesAxes;
        PhysBody* referencePhysBody;

        // for 2D, we are not doing edge contacts. so just faceA and faceB
        if (minPenetrationAxisIndex < 2)
        {
            // a has reference face
            // b has incident face
            contactManifold.type = ContactManifold::Type::FACE_A;
            
            referenceOBB = a;
            referencesAxes = aAxes;
            referencePhysBody = aBody;

            referenceFace = GetFace(a, aAxes, aBody, minPenetrationAxisIndex, axisDirection);            
            incidentFace = GetIncidentFace(b, bAxes, bBody, normal);            
        }
        else  
        {
            // b has reference face
            // a has incident face
            contactManifold.type = ContactManifold::Type::FACE_B;

            referenceOBB = b;
            referencesAxes = bAxes;
            referencePhysBody = bBody;

            referenceFace = GetFace(b, bAxes, bBody, minPenetrationAxisIndex, axisDirection);
            incidentFace = GetIncidentFace(a, aAxes, aBody, normal);
        }


        // we do Sutherland-Hodgman clipping

        // side planes for reference plane
        int planeNormalIndex = 1 - minPenetrationAxisIndex;


        float cos = 0;
        float sin = 1;

        glm::vec3 planeNormal = glm::vec3(cos * normal.x - sin * normal.y,
                                          sin * normal.x + cos * normal.y,
                                          0);
        
        glm::vec3 p0 = referencePhysBody->position + planeNormal * referenceOBB.halfEdges[planeNormalIndex];
        glm::vec3 p1 = referencePhysBody->position - planeNormal * referenceOBB.halfEdges[planeNormalIndex];

        Plane plane0 = { planeNormal, glm::dot(planeNormal, p0) };
        Plane plane1 = { -planeNormal, glm::dot(-planeNormal, p1) };

        glm::vec3 incidentVertices[2];
        incidentVertices[0] = incidentFace.v0;
        incidentVertices[1] = incidentFace.v1;

        glm::vec3 clippedVertices[2];

        int numVerticesOut = ClipLineSegmentToLine(incidentVertices, plane0, clippedVertices);
        if (numVerticesOut < 2);
            return;

        glm::vec3 clippedVertices2[2];


        numVerticesOut = ClipLineSegmentToLine(clippedVertices, plane1, clippedVertices2);
        if (numVerticesOut < 2);
            return;

            
        assert(numVerticesOut == 2);

        float referencePlaneOffset = glm::dot(referenceFace.v0, normal);

        for (int i = 0; i < numVerticesOut; i++)
        {

            int numContactPoints;
            ContactPoint contactPoints[MAX_CONTACT_POINTS];


            float distToReferencePlane = glm::dot(normal, clippedVertices2[i]) - referencePlaneOffset;

            if (distToReferencePlane <= 0)
            {
                int contactPointIndex = contactManifold.numContactPoints;
                ContactPoint* cp = &contactManifold.contactPoints[contactPointIndex];
                cp->position = clippedVertices2[i];
                cp->normal = normal;
                cp->penetration = distToReferencePlane;
            }

        }
            
            
            
            
        /*

        glm::vec3 planeNormal = referencesAxes[planeNormalIndex];

        glm::vec3 p0 = referencePhysBody->position + planeNormal * referenceOBB.halfEdges[planeNormalIndex];
        glm::vec3 p1 = referencePhysBody->position - planeNormal * referenceOBB.halfEdges[planeNormalIndex];

        Plane plane0 = { planeNormal, glm::dot(planeNormal, p0)};
        Plane plane1 = { planeNormal, glm::dot(planeNormal, p1) };
        */

    }


    void GetOBBPlaneContacts(OBB b, PhysBody* bBody,
                            Plane p, PhysBody* pBody, ContactManifold& contact)
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


        for (int i = 0; i < 4; i++)
        {

            glm::vec3 vertexPos = bBody->position + b.center + GetBoxVertexOffset(realAxes, b.halfEdges, dirs[i]);

            // compute vertex distance from the plane

            // need to change penetration to be negative, cuz in academic papers, they use that convention
            float dist = p.offset - glm::dot(vertexPos, p.normal);


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


    void PrepareContactPoints(ContactManifold& contact, PhysBody* a, PhysBody* b)
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

    void ResolveVelocity(ContactManifold& contact, PhysBody* a, PhysBody* b, float dt_s)
    {

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


    void ResolvePosition(ContactManifold& contact, PhysBody* a, PhysBody* b, float dt_s)
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


    void GenerateContactInfo(PhysBody* a, PhysBody* b, ContactManifold& contact)
    {
        if (a->shapeData.shape == PhysBodyShape::PB_OBB && b->shapeData.shape == PhysBodyShape::PB_PLANE)
        {
            glm::vec3 aCenter = a->position + a->shapeData.obb.center;

            contact.a = a;
            contact.b = NULL;

            GetOBBPlaneContacts(a->shapeData.obb, a, b->shapeData.plane, b, contact);
        }
        else if (a->shapeData.shape == PhysBodyShape::PB_CIRCLE && b->shapeData.shape == PhysBodyShape::PB_PLANE)
        {
            glm::vec3 aCenter = a->position + a->shapeData.sphere.center;

            contact.a = a;
            contact.b = NULL;

            GetSpherePlaneContacts(a->shapeData.sphere, a, b->shapeData.plane, b, contact);
        }
        else if (a->shapeData.shape == PhysBodyShape::PB_OBB && b->shapeData.shape == PhysBodyShape::PB_OBB)
        {
            glm::vec3 aCenter = a->position + a->shapeData.obb.center;

            contact.a = a;
            contact.b = b;

            GetOBBOBBContacts(a->shapeData.obb, a, b->shapeData.obb, b, contact);
        }
    };
};

