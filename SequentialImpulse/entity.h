#ifndef ENTITY_H_
#define	ENTITY_H_

#include "utility.h"
#include "renderer.h"
#include "model_enum.h"
#include "model.h"
#include <string>
#include "global.h"
#include "physics_core.h"

#include <vector>


using namespace std;

enum EntityType
{
    None,
    Ball,
    Box,
    Floor,
    Wall,
    XYZAxis,
};

enum EntityFlags
{
    // TODO(casey): Does it make more sense to have the flag be for _non_ colliding entities?
    // TODO(casey): Collides and ZSupported probably can be removed now/soon
    EntityFlag_Collides = (1 << 0),
    EntityFlag_Static = (1 << 1),
};







/*
Dragf

*/


/*
#pragma once
#include "transform_component.h"
#include "render_component.h"
class Player
{
	public:
		const float STEP_SIZE = 0.3f;

		glm::vec2 simPos;
		glm::vec2 curDir;
		int vision;

		TransformComponent transform;
		RenderComponent render;

		void update();
		void setCurDir(glm::vec2 dir);
		void move();

		void renderCore(Pipeline& p, Renderer* r);
};
*/


struct Entity
{
    public:
        Entity();

        static Entity* getOne();

		
        int id;
        unsigned int flags;

     //   char name[16];
        EntityType entityType;

        float mass;
        float invMass;

		glm::vec3 position;
		glm::vec3 velocity;
        glm::vec3 acceleration;
		glm::vec3 scale;
        glm::quat orientation;
        glm::mat4 orientationMat;
		glm::mat4 modelMatrix;
        glm::vec3 lastAcceleration;

        glm::vec3 angularVelocity;
        glm::vec3 angularAcceleration;
        // position, velocity,
        // Orientation, angular Velocity (aka rotation)
        // Game Physics Engine Development
        // Devs usually set a fixed range of orientation values, say  {-pi pi]
        // square means included, curly means not included
        
        // see Game Physics Engine Development 9.3
        // the angular velocity is r * axis
        // r is the rate of angular change, axis is the axis it is rotating



        Physics::PhysBody physBody;

		bool active;

        Model* m_model;
      


        inline void setModel(Model* model);
        inline void resetModel();
        inline void updateModelMatrix();

        glm::vec3 m_xAxis;
        glm::vec3 m_yAxis;
        glm::vec3 m_zAxis;
        inline void setOrientation(glm::mat4 rot);

        bool canRender();
        bool shouldRender();

        void renderCore(Pipeline& p, Renderer* r);

        

        
        // p211, holds the amount of damping applied to angular motion. Damping
        // is required to remove energy added through numerical instability in the integrator
        float velocityDamping;
        float angularDamping;

        // this is in local coordinate
        glm::mat3 inertiaTensor;

        // this is in world cooridnates
        glm::mat3 inverseInertiaTensor;

        glm::vec3 forceAccum;
        glm::vec3 torqueAccum;

        // Game Physics Engine Development
        // page 194
        // I have added a matrix to the class to hold the current transformation matrix for the object
        // the matrix is useful for rendering the object and will be useful at vaious points in the physics too,
        // so much so that it is worth the storage space to keep a copy with the rigid body.

        // page 206
        void addForce(glm::vec3 f)
        {
            forceAccum += f;
        }

        void addTorqueFromForce(glm::vec3 f, glm::vec3 vecToForce)
        {
            torqueAccum += glm::cross(vecToForce, f);

            utl::debug("    torqueAccum is ", torqueAccum);
        }
        
        void updateOrientation(glm::vec3 angularVelocity, float dtSec)
        {           
            glm::quat q(0, angularVelocity.x * dtSec, angularVelocity.y * dtSec, angularVelocity.z * dtSec);

            q = q * orientation;

            orientation.w += q.w * 0.5;
            orientation.x += q.x * 0.5;
            orientation.y += q.y * 0.5;
            orientation.z += q.z * 0.5;

            orientation = glm::normalize(orientation);

            SyncOrientationMat();
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

        
};






// velocity of a point is


inline void Entity::setOrientation(glm::mat4 rot)
{
	m_xAxis = glm::vec3(rot[0][0], rot[0][1], rot[0][2]);
	m_yAxis = glm::vec3(rot[1][0], rot[1][1], rot[1][2]);
	m_zAxis = glm::vec3(rot[2][0], rot[2][1], rot[2][2]);

	float temp[16] = {rot[0][0], rot[0][1], rot[0][2], 0.0,
                      rot[1][0], rot[1][1], rot[1][2], 0.0,
                      rot[2][0], rot[2][1], rot[2][2], 0.0,
                      0.0,       0.0,       0.0,       1.0};
    orientationMat = glm::make_mat4(temp);
}


inline void Entity::setModel(Model* model)
{
	m_model = model;
}

inline void Entity::resetModel()
{
	// we don't delete it here, cuz model manager owns the actual models
	m_model = NULL;
}


inline void Entity::updateModelMatrix()
{
	modelMatrix = glm::translate(position) * orientationMat * glm::scale(scale);
}

#endif


