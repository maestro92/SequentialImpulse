
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

    int numContactPoints;
    glm::vec3* contactPoints;


    Camera mainCamera;

    Entity* draggedEntity;

    float angle;
    int boxIndex;
};


namespace GameCode
{
    // box2d has it at -10
    glm::vec3 GRAVITY = glm::vec3(0, -10, 0);
    bool appliedForce = false;

    /*
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
    */

    void ProcessInputRaycast(GameState* gameState, glm::vec3 raycastDirection)
    {
        for (int i = 0; i < gameState->numEntities; i++)
        {
            Entity* entity = &gameState->entities[i];
            if (!entity->flags & EntityFlag_Static)
            {
                continue;
            }
            
            if (entity->physBody.shapeData.shape == Physics::PhysBodyShape::PB_OBB)
            {
                
                if (Physics::testPointInsideOBB2D(raycastDirection, entity->physBody.shapeData.obb, entity->physBody.orientationMat, entity->physBody.position))
                {
                    
                    gameState->draggedEntity = entity;
                    
                    /*
                    cout << "selecting entity " << entity->id << endl;
                    gameState->draggedEntity = entity;


//                    glm::vec3 vecToForce = glm::vec3(raycastDirection.x, raycastDirection.y, 0) - entity->position;
                    glm::vec3 vecToForce = glm::vec3(0, -5, 0);

                    glm::vec3 zAxis = glm::vec3(0, 0, 1);
                    glm::vec3 force = 1000.0f * glm::cross(vecToForce, zAxis);

                //    utl::debug("        vecToForce ", vecToForce);
                //    utl::debug("        force is ", force);

                    
                    appliedForce = true;
            //        entity->addForce(force);
                    entity->addTorqueFromForce(force, vecToForce);
                    */


                }                
            }
        }
    }



    void demo1Init(GameState* gameState)
    {
        gameState->worldWidth = 50;
        gameState->worldHeight = 50;
        gameState->numEntities = 0;


        gameState->entities = (Entity*)malloc(4096); // new Entity[4096];

        gameState->numContactPoints = 0;
        gameState->contactPoints = new glm::vec3[1024];

        gameState->angle = 0;
        gameState->draggedEntity = NULL;

        float scale = 100.0;
        int index = gameState->numEntities++;
        Entity* entity = &gameState->entities[index];
        entity->init();
        entity->id = gameState->numEntities;
        entity->entityType = EntityType::XYZAxis;
        entity->flags = EntityFlag_Static;
        entity->setModel(global.modelMgr->get(ModelEnum::xyzAxis));

        Physics::PhysBody* pb = &entity->physBody;
        pb->Init();
        
        pb->scale = glm::vec3(scale, scale, scale);



        // the box
        int x = 0;
        int y = 10;
        int size = 1;
        int w = size * 2;
        int h = size * 2;
        int d = 1;
        index = gameState->numEntities++;
        entity = &gameState->entities[index];
        entity->init();
        entity->id = index;
        entity->entityType = EntityType::Box;
        entity->flags = EntityFlag_Collides;
        entity->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));


        pb = &entity->physBody;
        pb->Init();
        pb->mass = 1;
        pb->invMass = 1 / (float)pb->mass;
        pb->position = glm::vec3(x, y, 0);
        glm::mat4 om = glm::rotate(0.0f, glm::vec3(0, 0, 1));
     //   glm::mat4 om = glm::rotate(0.0f, glm::vec3(0, 0, 1));

        /*
        glm::vec3 xAxis = glm::vec3(0, 1, 0);
        xAxis = glm::normalize(xAxis);
        glm::vec3 yAxis = glm::vec3(-1, 0, 0);
        yAxis = glm::normalize(yAxis);
        glm::vec3 zAxis = glm::vec3(0, 0, 1);

        // world to contact local matrix
        glm::mat4 om = utl::axes2GLMMat(xAxis, yAxis, zAxis);
        */
        pb->orientation = glm::toQuat(om);
        pb->SyncOrientationMat();
        pb->scale = glm::vec3(w, h, d);

        pb->velocityDamping = 0.95f;
        pb->angularDamping = 0.80f;

        pb->inertiaTensor = Physics::GetBoxInertiaTensor(pb->mass, w * 2, h * 2, d * 2);
        pb->transformInertiaTensor();
        

        pb->shapeData.shape = Physics::PhysBodyShape::PB_OBB;
        pb->shapeData.obb.center = glm::vec3(0, 0, 0);
        pb->shapeData.obb.axes[0] = glm::vec3(1, 0, 0);
        pb->shapeData.obb.axes[1] = glm::vec3(0, 1, 0);
        pb->shapeData.obb.axes[2] = glm::vec3(0, 0, 1);
        pb->shapeData.obb.halfEdges = glm::vec3(w, h, d);





        // the box
        x = 0;
        y = 14.5;
        size = 1;
        w = size * 2;
        h = size * 2;
        d = 1;
        index = gameState->numEntities++;
        entity = &gameState->entities[index];
        entity->init();
        entity->id = index;
        entity->entityType = EntityType::Box;
        entity->flags = EntityFlag_Collides;
        entity->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));

        pb = &entity->physBody;
        pb->Init();
        pb->mass = 1;
        pb->invMass = 1 / (float)pb->mass;
        pb->position = glm::vec3(x, y, 0);
        om = glm::rotate(45.0f, glm::vec3(0, 0, 1));
//         om = glm::rotate(0.0f, glm::vec3(0, 0, 1));

        pb->orientation = glm::toQuat(om);
        pb->SyncOrientationMat();
        pb->scale = glm::vec3(w, h, d);

        pb->velocityDamping = 0.95f;
        pb->angularDamping = 0.80f;

        pb->inertiaTensor = Physics::GetBoxInertiaTensor(pb->mass, w * 2, h * 2, d * 2);
        pb->transformInertiaTensor();


        pb->shapeData.shape = Physics::PhysBodyShape::PB_OBB;
        pb->shapeData.obb.center = glm::vec3(0, 0, 0);
        pb->shapeData.obb.axes[0] = glm::vec3(1, 0, 0);
        pb->shapeData.obb.axes[1] = glm::vec3(0, 1, 0);
        pb->shapeData.obb.axes[2] = glm::vec3(0, 0, 1);
        pb->shapeData.obb.halfEdges = glm::vec3(w, h, d);







        // the floor 
        index = gameState->numEntities++;
        entity = &gameState->entities[index];
        entity->init();
        entity->id = index;
        entity->entityType = EntityType::Floor;


        pb = &entity->physBody;
        pb->Init();
        pb->shapeData.shape = Physics::PhysBodyShape::PB_PLANE;
        pb->shapeData.plane.normal = glm::vec3(0, 1, 0);
        pb->shapeData.plane.offset = 0;       // dot(glm::vec3(0, 1, 0),  glm::vec3(0, 0, 0));

        pb->position = glm::vec3(0, 0, 0);
        pb->scale = glm::vec3(gameState->worldWidth, 0.2, 1);

        entity->flags = EntityFlag_Collides | EntityFlag_Static;
        entity->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));



        // the wall
        index = gameState->numEntities++;
        entity = &gameState->entities[index];
        entity->init();
        entity->id = index;
        entity->entityType = EntityType::Wall;
        entity->flags = EntityFlag_Collides | EntityFlag_Static;
        entity->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));

        pb = &entity->physBody;
        pb->Init();
        pb->position = glm::vec3(gameState->worldWidth / 2, gameState->worldHeight / 2, 0);
        pb->scale = glm::vec3(2, gameState->worldHeight, 1);



        index = gameState->numEntities++;
        entity = &gameState->entities[index];
        entity->init();
        entity->id = index;
        entity->entityType = EntityType::Wall;
        entity->flags = EntityFlag_Collides | EntityFlag_Static;
        entity->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));
    
        pb = &entity->physBody;
        pb->Init();
        pb->position = glm::vec3(-gameState->worldWidth / 2, gameState->worldHeight / 2, 0);
        pb->scale = glm::vec3(2, gameState->worldHeight, 1);

}

    /*
    void demo2Init(GameState* gameState)
    {
        gameState->worldWidth = 50;
        gameState->worldHeight = 50;
        gameState->numEntities = 0;
        gameState->entities = new Entity[4096];

        gameState->numContactPoints = 0;
        gameState->contactPoints = new glm::vec3[1024];


        float scale = 100.0;
        Entity* entity = gameState[
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
    */

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




    void integrateVelocity(Physics::PhysBody* pb, float dt_s)
    {
   //     std::cout << "awake " << entity->isAwake << std::endl;

   //     if (!entity->isAwake)
   //         return;

        float linearDamping = 0.0f;
        float angularDamping = 0.0f;

        pb->velocity += pb->forceAccum / pb->mass * dt_s;
        // apply linear damping        
     //   entity->velocity = entity->velocity * 1.0f / (1.0f + linearDamping * dt_s);

        pb->angularVelocity += pb->inverseInertiaTensor * pb->torqueAccum * dt_s;
        // apply angular damping
    //    entity->angularVelocity = entity->angularVelocity * 1.0f / (1.0f + linearDamping * dt_s);


    }



    void integratePosition(Physics::PhysBody* pb, float dt_s)
    {
        float maxTranslation = 2.0f;

        
//        if (glm::dot(entity->velocity, entity->velocity) > 0.001)
        if (true)
        {

            glm::vec3 translation = dt_s * pb->velocity;
            //    std::cout << "glm::length(translation) " << glm::length(translation) << std::endl;



            if (glm::dot(translation, translation) > (maxTranslation * maxTranslation))
            {

                //    utl::debug("        clamping velocity");
                float ratio = maxTranslation / glm::length(translation);
                pb->velocity *= ratio;
            }
            /*
            utl::debug("        entityVelocity", entity->velocity);
            float vellength = glm::dot(entity->velocity, entity->velocity);
            utl::debug("        vellength", vellength);

            utl::debug("        entity->velocity * dt_s", entity->velocity * dt_s);
            */

            pb->position += pb->velocity * dt_s;
        }
        else
        {
            pb->velocity = glm::vec3(0,0,0);
        }
    //    utl::debug("        entity->position", entity->position);

        // do the same for orientation

    
        float angularMag = glm::length(pb->angularVelocity);
        if (angularMag != 0)
        {
            // https://math.stackexchange.com/questions/22437/combining-two-3d-rotations
            pb->addRotation(pb->angularVelocity, dt_s);
            pb->transformInertiaTensor();
        }
    }



    void CopyContactPoints(GameState* gameState, Physics::ContactManifold* contact)
    {
        for (int i = 0; i < contact->numContactPoints; i++)
        {
            gameState->contactPoints[gameState->numContactPoints++] = contact->contactPoints[i].position;
        }
    }
    

    void tick(GameInput gameInput, GameState* gameState)
    {
        gameState->numContactPoints = 0;

        // cout << "gameState->numEntities " << gameState->numEntities << endl;
        for (int i = 0; i < gameState->numEntities; i++)
        {
            if (!(gameState->entities[i].flags & EntityFlag_Static) && &gameState->entities[i] != gameState->draggedEntity)
            {
                Entity* entity = &gameState->entities[i];
                {
            //        gameState->entities[i].physBody.addForce(gameState->entities[i].physBody.mass * GRAVITY, false);

/*
                    // adding drag
                    glm::vec3 drag = gameState->entities[i].velocity;


                    if (glm::length(drag) != 0)
                    {
                        float dragCoeff = glm::length(drag);
                        dragCoeff = 0.1 * dragCoeff;

                        drag = glm::normalize(drag);
                        drag *= -dragCoeff;

                        // gameState->entities[i].forceAccum += drag;
                        gameState->entities[i].addForce(drag, false);



                 //       utl::debug("forceAccum ", gameState->entities[i].forceAccum);
                    }
                    */
                }
            }
        }
        


        for (int i = 0; i < gameState->numEntities; i++)
        {
            if (!(gameState->entities[i].flags & EntityFlag_Static))
            {
                integrateVelocity(&gameState->entities[i].physBody, gameInput.dt_s);
        //        integratePosition(&gameState->entities[i], gameInput.dt_s);
            }
        }

        vector<Physics::ContactManifold> contactsThisTick;
    //    cout << "gameState->numEntities " << gameState->numEntities << endl;
        for (int i = 0; i < gameState->numEntities; i++)
        {
            if (!gameState->entities[i].flags & EntityFlag_Static)
            {
                continue;
            }

            for (int j = i + 1; j < gameState->numEntities; j++)
            {                
                Physics::ContactManifold contact;

                /*
                if (Physics::TestContactInfo(gameState->entities[i], gameState->entities[j], contact))
                {
                    CopyContactPoints(gameState, &contact);

                    cout << "Resolving contact" << endl;
                    Physics::Resolve(contact, &gameState->entities[i], &gameState->entities[j]);
                }
                */
                
                Physics::GenerateContactInfo(&gameState->entities[i].physBody, &gameState->entities[j].physBody, contact);
                if (contact.numContactPoints > 0)
                {                    

                    cout << "Colliding" << endl;

                    CopyContactPoints(gameState, &contact);
                    contactsThisTick.push_back(contact);

              //      cout << "\n\nResolving contact" << endl;

                    if (gameState->entities[j].flags & EntityFlag_Static)
                    {
                        Physics::PrepareContactPoints(contact, &gameState->entities[i].physBody, NULL);
                        Physics::ResolveVelocity(contact, &gameState->entities[i].physBody, NULL, gameInput.dt_s);
                    }
                    else
                    {
                        Physics::PrepareContactPoints(contact, &gameState->entities[i].physBody, &gameState->entities[j].physBody);
                        Physics::ResolveVelocity(contact, &gameState->entities[i].physBody, &gameState->entities[j].physBody, gameInput.dt_s);
                    }
                }
            }
        }


        
        // by applying velocity first, we may get to skip doing position resolution. so this saves us some computation
        for (int i = 0; i < gameState->numEntities; i++)
        {
            if (!(gameState->entities[i].flags & EntityFlag_Static))
            {
                integratePosition(&gameState->entities[i].physBody, gameInput.dt_s);
            }
        }

        for (int i = 0; i < contactsThisTick.size(); i++)
        {
            Physics::ResolvePosition(contactsThisTick[i], contactsThisTick[i].a, contactsThisTick[i].b, gameInput.dt_s);
        }


        for (int i = 0; i < gameState->numEntities; i++)
        {
            if (!(gameState->entities[i].flags & EntityFlag_Static))
            {
                gameState->entities[i].physBody.forceAccum = glm::vec3(0.0);
                gameState->entities[i].physBody.torqueAccum = glm::vec3(0.0);

            }
        }


        /*
        for (int i = 0; i < contactsThisTick.size(); i++)
        {

        }
        */

        /*

        for (int i = 0; i < gameState->numEntities; i++)
        {
            if (!(gameState->entities[i].flags & EntityFlag_Static))
            {
                integratePosition(&gameState->entities[i], gameInput.dt_s);
            }
        }
        
  //      Physics::Resolve(contact, &gameState->entities[i], NULL, gameInput.dt_s);
  */
    }
};


#endif