
#ifndef GAME_H_
#define GAME_H_

#include "model_manager.h"
#include "global.h"
#include "platform.h"
#include "utility_math.h"
#include "pipeline.h"
#include "camera.h"
#include "physics.h"
#include "entity.h"

struct GameState
{
    int worldWidth;
    int worldHeight;

    Entity* entities;
    int numEntities;

    Camera mainCamera;

    Entity* draggedEntity;

    float angle;
    int boxIndex;
};


namespace GameCode
{
    glm::vec3 GRAVITY = glm::vec3(0, -0.03, 0);

    void addRandomBall(GameState* gameState)
    {
        Entity entity;

        int x = utl::randFloat(0, 30);
        int y = utl::randFloat(0, 30);

        int size = utl::randFloat(1, 2);

        entity.entityType = EntityType::Ball;
        entity.flags |= EntityFlag_Collides;
        entity.position = glm::vec3(x, y, 0);
        entity.scale = glm::vec3(size, size, size);
        entity.setModel(global.modelMgr->get(ModelEnum::circle));
        gameState->entities[gameState->numEntities++] = entity;
    }

    void addRandomBox(GameState* gameState)
    {
        Entity entity;

        int x = utl::randFloat(0, 30);
        int y = utl::randFloat(0, 30);

        int size = utl::randFloat(1, 2);

        entity.entityType = EntityType::Ball;
        entity.flags |= EntityFlag_Collides;
        entity.position = glm::vec3(x, y, 0);
        entity.scale = glm::vec3(size, size, size);
        entity.setModel(global.modelMgr->get(ModelEnum::centeredQuad));
        gameState->entities[gameState->numEntities++] = entity;
    }


    void ProcessInputRaycast(GameState* gameState, glm::vec3 raycastDirection)
    {
        for (int i = 0; i < gameState->numEntities; i++)
        {
            Entity* entity = &gameState->entities[i];
            if (!entity->flags & EntityFlag_Static)
            {
                continue;
            }
            
            if (entity->physBody.type == Physics::PhysBodyType::PB_OBB)
            {
                
                if (Physics::testPointInsideOBB2D(raycastDirection, entity->physBody.obb, entity->orientation, entity->position))
                {
                    cout << "selecting entity " << entity->id << endl;
                    gameState->draggedEntity = entity;
                }
                
            }
        }
    }



    void demo1Init(GameState* gameState)
    {
        gameState->numEntities = 0;
        gameState->worldWidth = 50;
        gameState->worldHeight = 50;
        gameState->entities = new Entity[4096];
        gameState->angle = 0;
        gameState->draggedEntity = NULL;

        float scale = 100.0;
        int index = gameState->numEntities++;
        Entity* entity = &gameState->entities[index];
        entity->id = gameState->numEntities;
        entity->entityType = EntityType::XYZAxis;
        entity->scale = glm::vec3(scale, scale, scale);
        entity->flags = EntityFlag_Static;
        entity->setModel(global.modelMgr->get(ModelEnum::xyzAxis));

        if (global.modelMgr->get(ModelEnum::xyzAxis) == NULL)
        {
            cout << "XYZ is null" << endl;
        }
        else
        {
            cout << "XYZ is not null" << endl;
        }

        cout << "name is " << gameState->entities[0].entityType << endl;
        if (gameState->entities[0].m_model != NULL)
        {
            cout << "   Can render0" << endl;
        }
        else
        {
            cout << "   Cant render0" << endl;
        }

        if (gameState->entities[0].m_model != NULL)
        {
            cout << "   Can render01" << endl;
        }
        else
        {
            cout << "   Cant render01" << endl;
        }

        //        gameState->entities[gameState->numEntities++] = entity;


        
        // the box
        int x = 0;
        int y = 10;
        int size = 5;
        index = gameState->numEntities++;
        entity = &gameState->entities[index];
        entity->id = index;
        entity->entityType = EntityType::Box;
        entity->flags = EntityFlag_Collides;

        entity->mass = 1;
        entity->position = glm::vec3(x, y, 0);
        entity->orientation = glm::rotate(30.0f, glm::vec3(0, 0, 1));
        entity->scale = glm::vec3(size, size, size);
        entity->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));
        
        entity->physBody.type = Physics::PhysBodyType::PB_OBB;
        entity->physBody.obb.center = glm::vec3(0, 0, 0);
        entity->physBody.obb.axes[0] = glm::vec3(1, 0, 0);
        entity->physBody.obb.axes[1] = glm::vec3(0, 1, 0);
        entity->physBody.obb.axes[2] = glm::vec3(0, 0, 1);
        entity->physBody.obb.halfEdges = glm::vec3(size, size, size);
        
        gameState->boxIndex = index;

        // the floor 
        index = gameState->numEntities++;
        entity = &gameState->entities[index];
        entity->id = index;
        entity->entityType = EntityType::Floor;
        entity->position = glm::vec3(0, 0, 0);
        entity->flags = EntityFlag_Collides | EntityFlag_Static;
        entity->scale = glm::vec3(gameState->worldWidth, 0.2, 1);
        entity->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));

        entity->physBody.type = Physics::PhysBodyType::PB_PLANE;
        entity->physBody.plane.normal = glm::vec3(0, 1, 0);
        entity->physBody.plane.d = 0;       // dot(glm::vec3(0, 1, 0),  glm::vec3(0, 0, 0));


        // the wall
        index = gameState->numEntities++;
        entity = &gameState->entities[index];
        entity->id = index;
        entity->entityType = EntityType::Wall;
        entity->position = glm::vec3(gameState->worldWidth / 2, gameState->worldHeight / 2, 0);
        entity->flags = EntityFlag_Collides | EntityFlag_Static;
        entity->scale = glm::vec3(2, gameState->worldHeight, 1);
        entity->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));


        index = gameState->numEntities++;
        entity = &gameState->entities[index];
        entity->id = index;
        entity->entityType = EntityType::Wall;
        entity->position = glm::vec3(-gameState->worldWidth / 2, gameState->worldHeight / 2, 0);
        entity->flags = EntityFlag_Collides | EntityFlag_Static;
        entity->scale = glm::vec3(2, gameState->worldHeight, 1);
        entity->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));
    }


    void demo2Init(GameState* gameState)
    {
        gameState->numEntities = 0;
        gameState->worldWidth = 50;
        gameState->worldHeight = 50;
        gameState->entities = new Entity[4096];

        float scale = 100.0;
        Entity entity;
        entity.entityType = EntityType::XYZAxis;
        entity.scale = glm::vec3(scale, scale, scale);
        entity.flags = EntityFlag_Static;
        entity.setModel(global.modelMgr->get(ModelEnum::xyzAxis));
        gameState->entities[gameState->numEntities++] = entity;

        for (int i = 0; i < 50; i++)
        {
            addRandomBall(gameState);
        }

        for (int i = 0; i < 50; i++)
        {
            addRandomBox(gameState);
        }

        entity.entityType = EntityType::Floor;
        entity.position = glm::vec3(0, 0, 0);
        entity.flags |= EntityFlag_Collides | EntityFlag_Static;
        entity.scale = glm::vec3(gameState->worldWidth, 2, 1);
        entity.setModel(global.modelMgr->get(ModelEnum::centeredQuad));
        gameState->entities[gameState->numEntities++] = entity;

        entity.entityType = EntityType::Wall;
        entity.position = glm::vec3(gameState->worldWidth / 2, gameState->worldHeight / 2, 0);
        entity.flags = EntityFlag_Collides | EntityFlag_Static;
        entity.scale = glm::vec3(2, gameState->worldHeight, 1);
        entity.setModel(global.modelMgr->get(ModelEnum::centeredQuad));
        gameState->entities[gameState->numEntities++] = entity;

        entity.entityType = EntityType::Wall;
        entity.position = glm::vec3(-gameState->worldWidth / 2, gameState->worldHeight / 2, 0);
        entity.flags = EntityFlag_Collides | EntityFlag_Static;
        entity.scale = glm::vec3(2, gameState->worldHeight, 1);
        entity.setModel(global.modelMgr->get(ModelEnum::centeredQuad));
        gameState->entities[gameState->numEntities++] = entity;
    }



    glm::vec3 screenToWorldPoint(GameState* gameState, glm::vec2 screenPoint)
    {
        glm::vec4 viewPort = glm::vec4(0, 0, utl::SCREEN_WIDTH, utl::SCREEN_HEIGHT);
        glm::vec3 temp = glm::vec3(screenPoint.x, screenPoint.y, 0);

        glm::vec3 worldPoint = glm::unProject(temp, (gameState->mainCamera.getPipeline().getModelViewMatrix()), gameState->mainCamera.getPipeline().getProjectionMatrix(), viewPort);
        return worldPoint;
    }


    glm::vec3 worldToScreen(GameState* gameState, glm::vec3 pos)
    {
        glm::vec4 viewPort = glm::vec4(0, 0, utl::SCREEN_WIDTH, utl::SCREEN_HEIGHT);
        //	glm::vec3 screenPos = glm::project(pos, glm::inverse(m_pipeline.getModelViewMatrix()), m_pipeline.getProjectionMatrix(), viewPort);
        glm::vec3 screenPos = glm::project(pos, gameState->mainCamera.getPipeline().getModelViewMatrix(), gameState->mainCamera.getPipeline().getProjectionMatrix(), viewPort);
        return screenPos;
    }



    void init(GameState* gameState)
    {
        demo1Init(gameState);        
    }

    float angle = 0;



    // Game Physics Enginer Development 
    // although the proper formula is x' = x + vt + 0.5 * a * t^2
    // 3.31 the acceleration is often neglecgible. so we just ignore the acceleration term
    // so we just ignore the acceleration term
    void integrate(Entity* entity, float dt_s)
    {        
        
        entity->position += dt_s * entity->velocity;
    //    entity->orientation += dt_s * entity->angularVelocity;


        // velocity has damping
        // you can consider removing the damping term altogether if you have lots of entities
        entity->acceleration = entity->forceAccum / entity->mass;
        entity->velocity = entity->velocityDamping * entity->velocity + entity->acceleration * dt_s;


        entity->angularVelocity = entity->angularDamping * entity->angularVelocity;


        // update matrices with the new position and orientation

        entity->forceAccum = glm::vec3(0, 0, 0);
        entity->torqueAccum = glm::vec3(0, 0, 0);        
    }


    void tick(GameInput gameInput, GameState* gameState)
    {

        gameState->angle += 1;
        if (gameState->angle >= 360)
        {
            gameState->angle -= 360;
        }

        glm::mat4 rot = glm::rotate(gameState->angle, glm::vec3(0, 0, 1));
//        gameState->entities[gameState->boxIndex].setRotation(rot);

        gameState->entities[gameState->boxIndex].orientation = rot;

        // cout << "gameState->numEntities " << gameState->numEntities << endl;
        for (int i = 0; i < gameState->numEntities; i++)
        {
            if (!(gameState->entities[i].flags & EntityFlag_Static))
            {
                /*
                if ( glm::distance(gameState->entities[i].velocity, glm::vec3(0,0,0)) < 1)
                {
                    gameState->entities[i].velocity += GRAVITY;
                }
                */

            //    gameState->entities[i].addForce(GRAVITY);
            
                // gravity doesnt exert torque
                // gameState->entities[i].addTorque(GRAVITY);

            }
        }
        

        for (int i = 0; i < gameState->numEntities; i++)
        {
            if (!gameState->entities[i].flags & EntityFlag_Static)
            {
                continue;
            }

            for (int j = i + 1; j < gameState->numEntities; j++)
            {
                Physics::ContactInfo contact;
                if (Physics::GenerateContactInfo(gameState->entities[i], gameState->entities[j], contact))
                {
                    cout << "Resolving contact" << endl;
                    Physics::Resolve(contact, &gameState->entities[i], &gameState->entities[j]);
                }

            }
        }


        for (int i = 0; i < gameState->numEntities; i++)
        {
            if (!(gameState->entities[i].flags & EntityFlag_Static))
            {
                integrate(&gameState->entities[i], gameInput.dt_s);
            }
        }
    }
};


#endif