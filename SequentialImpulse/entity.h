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
    Sphere,
    Box,
    Capsule,
    Floor,
    Wall,
    Character,
    XYZAxis,
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

static float sleepEpislon = 0.3;


struct Entity
{
    public:


        int id;


     //   char name[16];
        EntityType entityType;

    //    glm::mat4 modelMatrix;

        // position, velocity,
        // Orientation, angular Velocity (aka rotation)
        // Game Physics Engine Development
        // Devs usually set a fixed range of orientation values, say  {-pi pi]
        // square means included, curly means not included
        
        // see Game Physics Engine Development 9.3
        // the angular velocity is r * axis
        // r is the rate of angular change, axis is the axis it is rotating

    //    bool isDead;
        float motion;

        bool canSleep;


        Physics::PhysBody physBody;

        bool active;

     //   Model* m_model;
      

;
        inline void updateModelMatrix();

        glm::vec3 m_xAxis;
        glm::vec3 m_yAxis;
        glm::vec3 m_zAxis;
        inline void setOrientation(glm::mat4 rot);

        bool canRender();
        bool shouldRender();

    //    void renderCore(Pipeline& p, Renderer* r);

        // p211, holds the amount of damping applied to angular motion. Damping
        // is required to remove energy added through numerical instability in the integrator


        // Game Physics Engine Development
        // page 194
        // I have added a matrix to the class to hold the current transformation matrix for the object
        // the matrix is useful for rendering the object and will be useful at vaious points in the physics too,
        // so much so that it is worth the storage space to keep a copy with the rigid body.

        // page 206
       
};




// velocity of a point is



/*
inline void Entity::setModel(Model* model)
{
	m_model = model;
}

inline void Entity::resetModel()
{
	// we don't delete it here, cuz model manager owns the actual models
	m_model = NULL;
}
*/
/*
inline void Entity::updateModelMatrix()
{
	modelMatrix = glm::translate(physBody.position) * physBody.orientationMat * glm::scale(physBody.scale);
}
*/
#endif


