
#ifndef GAME_H_
#define GAME_H_

#include "model_manager.h"
#include "global.h"
#include "platform.h"
#include "utility_math.h"
#include "pipeline.h"
#include "camera.h"
#include "physics_core.h"
#include "physics.h"
#include "entity.h"
#include "memory_util.h"

/*

-   fix floor not having to be the last entity                      done
    put xyz axis back                                               done
-   display number of entities / contacts / joints as debug text    done
-   solve complete box overlap                                      sort of done
    make it more stable                                             done
-   clean up quad class                                             done                
-   fix current mouse joint                                         done
-   make rendering subpixel accurate                                
-   make the input polling code like handmade hero's                
-   set it up as a pyramid demo
*/




struct GameState
{
    GameMemory gameMemory;
    MemorySection simMemorySection;
    MemorySection assetMemorySection;

    int worldWidth;
    int worldHeight;

    Entity* entities;
    int numEntities;

    int numDebugContactManifolds;
    Physics::ContactManifold* debugContactManifolds;

    int numContacts;
    Physics::ContactManifold* contacts;


    int numJoints;
    Physics::Joint* joints;


    Camera mainCamera;

    float angle;
    int boxIndex;

  //  int mouseJointIndex;
    int mouseJointEntityIndex;

    int frameCount;

    bool isInteracting;

};



namespace GameCode
{
    // box2d has it at -10
    glm::vec3 GRAVITY = glm::vec3(0, -10, 0);
    bool appliedForce = false;
    bool GRAVITY_ACTIVE = true;
    bool COLLISION_ACTIVE = true;

    int jointcounter = 0;
    bool debug = false;

    // sleeping is not correct currently
    bool ALLOW_SLEEPING = false;


    float LINEAR_SLEEP_TOLERANCE = 0.01f;
    float ANGULAR_SLEEP_TOLERANCE = 2.0f;
    float DORMANT_TIME_TO_SLEEP = 0.5f; // if you are dormatn for more than half a second, you sleep

    float MAX_TRANSLATION = 2.0f;

    float MAX_ROTATION = (0.5f * 3.14f);
    float MAX_ROTATION_SQUARED = MAX_ROTATION * MAX_ROTATION;


    Entity* addEntity(GameState* gameState, EntityType entType, Physics::PhysBodyDef physDef)
    {
        int index = gameState->numEntities++;
        Entity* ent = &gameState->entities[index];

        ent->init();
        ent->id = index;
        ent->entityType = entType;
        ent->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));
    //    ent->isDead = false;
        Physics::PhysBody* pb = &ent->physBody;
        pb->id = index;
        pb->initAsBox(physDef);
        return ent;
    }


    void addRandomBox(GameState* gameState, int height)
    {
        Physics::PhysBodyDef def = {};

        /*
        float size = 2;
        def.halfDim = glm::vec3(size * utl::randInt(1, 3),
                                size * utl::randInt(1, 3), 
                                0.5);
        def.flags = Physics::PhysBodyFlag_Collides;
        def.mass = 5;
        float xOffset = 0;
        def.pos = glm::vec3(utl::randFloat(-20, 20) + xOffset,
                            utl::randFloat(5, 20),
                            0);
        float rot = utl::randFloat(0, 360);
        def.rot = glm::rotate(rot, glm::vec3(0, 0, 1));
        */


        float size = 3;
        def.halfDim = glm::vec3(size,
                                size,
                                0.5);
        def.flags = Physics::PhysBodyFlag_Collides;
        def.mass = 5;
        float xOffset = 0;
        def.pos = glm::vec3(0,
                            50,
                            0);
        float rot = 0; //utl::randFloat(0, 360);
        def.rot = glm::rotate(rot, glm::vec3(0, 0, 1));


        def.hasJoint = false;
        addEntity(gameState, Box, def);
    }

    Entity* addRagdollBody(GameState* gameState)
    {
        Physics::PhysBodyDef def = {};
        float size = 5;
        def.halfDim = glm::vec3(size, 1.5 * size, 0.5);
        def.flags = Physics::PhysBodyFlag_Collides;
        def.mass = 5;
        def.hasJoint = true;

        float xOffset = 0;
        def.pos = glm::vec3(xOffset, 50, 0);

        float rot = 0;
        def.rot = glm::rotate(rot, glm::vec3(0, 0, 1));

        return addEntity(gameState, EntityType::Box, def);
    }

    Entity* addRagdollHead(GameState* gameState, Physics::PhysBody* body, glm::vec3& anchor)
    {
        Physics::PhysBodyDef def = {};
        float size = 2;
        def.halfDim = glm::vec3(size, size, 0.5);
        def.flags = Physics::PhysBodyFlag_Collides;
        def.mass = 5;
        def.hasJoint = true;

        anchor = body->position + glm::vec3(0, body->scale.y, 0);
        def.pos = body->position + glm::vec3(0, body->scale.y + def.halfDim.y, 0);

        return addEntity(gameState, EntityType::Box, def);
    }

    Entity* addRagdollShoulder(GameState* gameState, bool left, Physics::PhysBody* body, glm::vec3& anchor)
    {
        Physics::PhysBodyDef def = {};
        float size = 2;
        def.halfDim = glm::vec3(1.5 * size, size, 0.5);
        def.flags = Physics::PhysBodyFlag_Collides;
        def.mass = 5;
        def.hasJoint = true;

        // assume in identity orientation
        if (left)
        {
            glm::vec3 topCorner = body->position + glm::vec3(-body->scale.x, body->scale.y, 0);

            anchor = topCorner + glm::vec3(0, -def.halfDim.y, 0);
            glm::vec3 pos = topCorner + glm::vec3(-def.halfDim.x, -def.halfDim.y, 0);
            def.pos = pos;
        }
        else
        {
            glm::vec3 topCorner = body->position + glm::vec3(body->scale.x, body->scale.y, 0);

            anchor = topCorner + glm::vec3(0, -def.halfDim.y, 0);
            glm::vec3 pos = topCorner + glm::vec3(def.halfDim.x, -def.halfDim.y, 0);
            def.pos = pos;
        }
        return addEntity(gameState, EntityType::Box, def);
    }

    Entity* addRagdollArm(GameState* gameState, bool left, Physics::PhysBody* parent, glm::vec3& anchor)
    {
        Physics::PhysBodyDef def = {};
        float size = 2;
        def.halfDim = glm::vec3(2 * size, size, 0.5);
        def.flags = Physics::PhysBodyFlag_Collides;
        def.mass = 5;
        def.hasJoint = true;
        if (left)
        {
            glm::vec3 corner = parent->position + glm::vec3(-parent->scale.x, parent->scale.y, 0);

            anchor = corner + glm::vec3(0, -def.halfDim.y, 0);
            glm::vec3 pos = corner + glm::vec3(-def.halfDim.x, -def.halfDim.y, 0);
            def.pos = pos;
        }
        else
        {
            glm::vec3 corner = parent->position + glm::vec3(parent->scale.x, parent->scale.y, 0);

            anchor = corner + glm::vec3(0, -def.halfDim.y, 0);
            glm::vec3 pos = corner + glm::vec3(def.halfDim.x, -def.halfDim.y, 0);
            def.pos = pos;
        }
        return addEntity(gameState, EntityType::Box, def);
    }

    Entity* addRagdollThigh(GameState* gameState, bool left, Physics::PhysBody* parent, glm::vec3& anchor)
    {
        Physics::PhysBodyDef def = {};
        float size = 2;
        def.halfDim = glm::vec3(size, 2 * size, 0.5);
        def.flags = Physics::PhysBodyFlag_Collides;
        def.mass = 5;
        def.hasJoint = true;

        if (left)
        {
            glm::vec3 corner = parent->position + glm::vec3(-parent->scale.x, -parent->scale.y, 0);

            anchor = corner + glm::vec3(def.halfDim.x, 0, 0);
            glm::vec3 pos = corner + glm::vec3(def.halfDim.x, -def.halfDim.y, 0);
            def.pos = pos;
        }
        else
        {
            glm::vec3 corner = parent->position + glm::vec3(parent->scale.x, -parent->scale.y, 0);

            anchor = corner + glm::vec3(-def.halfDim.x, 0, 0);
            glm::vec3 pos = corner + glm::vec3(-def.halfDim.x, -def.halfDim.y, 0);
            def.pos = pos;
        }
        return addEntity(gameState, EntityType::Box, def);
    }

    Entity* addRagdollLeg(GameState* gameState, bool left, Physics::PhysBody* parent, glm::vec3& anchor)
    {
        Physics::PhysBodyDef def = {};
        float size = 2;
        def.halfDim = glm::vec3(size, 2.5 * size, 0.5);
        def.flags = Physics::PhysBodyFlag_Collides;
        def.mass = 5;
        def.hasJoint = true;

        if (left)
        {
            glm::vec3 corner = parent->position + glm::vec3(-parent->scale.x, -parent->scale.y, 0);

            anchor = corner + glm::vec3(def.halfDim.x, 0, 0);

            def.pos = corner + glm::vec3(def.halfDim.x, -def.halfDim.y, 0);
        }
        else
        {
            glm::vec3 corner = parent->position + glm::vec3(parent->scale.x, -parent->scale.y, 0);

            anchor = corner + glm::vec3(-def.halfDim.x, 0, 0);

            def.pos = corner + glm::vec3(-def.halfDim.x, -def.halfDim.y, 0);
        }
        return addEntity(gameState, EntityType::Box, def);
    }


    glm::vec3 computeLocalAnchorPoint(glm::vec3 anchor, Physics::PhysBody* body)
    {
        return glm::mat3(body->orientationMat) * (anchor - body->position);
    }

    void addJoint(GameState* gameState, glm::vec3 worldAnchorPoint, Physics::PhysBody* a, Physics::PhysBody* b, bool ignoreCollision)
    {
        Physics::Joint* joint = &gameState->joints[gameState->numJoints];
        gameState->numJoints++;
    //    joint->isDead = false;
        joint->a = a;
        joint->b = b;

        joint->aLocalAnchor = computeLocalAnchorPoint(worldAnchorPoint, a);
        joint->bLocalAnchor = computeLocalAnchorPoint(worldAnchorPoint, b);

        joint->ignoreCollision = ignoreCollision;
    }


    void addMouseJoint(GameState* gameState, glm::vec3 worldAnchorPoint, Physics::PhysBody* mouse, Physics::PhysBody* draggedEnt, bool ignoreCollision)
    {
        Physics::Joint* joint = &gameState->joints[gameState->numJoints];
        gameState->numJoints++;
     //   joint->isDead = false;
        joint->a = draggedEnt;
        joint->b = mouse;

        joint->aLocalAnchor = computeLocalAnchorPoint(worldAnchorPoint, draggedEnt);
        joint->bLocalAnchor = glm::vec3(0,0,0);

        joint->ignoreCollision = ignoreCollision;


    }

    void addRagdoll(GameState* gameState)
    {
        Entity* body = addRagdollBody(gameState);

        glm::vec3 headAnchor = glm::vec3(0.0);
        Entity* head = addRagdollHead(gameState, &body->physBody, headAnchor);

        
        glm::vec3 leftShouldAnchor = glm::vec3(0.0);
        Entity* leftShoulder = addRagdollShoulder(gameState, true, &body->physBody, leftShouldAnchor);
       
        glm::vec3 rightShouldAnchor = glm::vec3(0.0);
        Entity* rightShoulder = addRagdollShoulder(gameState, false, &body->physBody, rightShouldAnchor);


        glm::vec3 leftArmAnchor = glm::vec3(0.0);
        Entity* leftArm = addRagdollArm(gameState, true, &leftShoulder->physBody, leftArmAnchor);

        glm::vec3 rightArmAnchor = glm::vec3(0.0);
        Entity* rightArm = addRagdollArm(gameState, false, &rightShoulder->physBody, rightArmAnchor);


        glm::vec3 leftThighAnchor = glm::vec3(0.0);
        Entity* leftThigh = addRagdollThigh(gameState, true, &body->physBody, leftThighAnchor);

        glm::vec3 rightThighAnchor = glm::vec3(0.0);
        Entity* rightThigh = addRagdollThigh(gameState, false, &body->physBody, rightThighAnchor);


        glm::vec3 leftLegAnchor = glm::vec3(0.0);
        Entity* leftLeg = addRagdollLeg(gameState, true, &leftThigh->physBody, leftLegAnchor);

        glm::vec3 rightLegAnchor = glm::vec3(0.0);
        Entity* rightLeg = addRagdollLeg(gameState, false, &rightThigh->physBody, rightLegAnchor);
        

        addJoint(gameState, headAnchor, &body->physBody, &head->physBody, false);
        
        addJoint(gameState, leftShouldAnchor, &body->physBody, &leftShoulder->physBody, true);
        addJoint(gameState, rightShouldAnchor, &body->physBody, &rightShoulder->physBody, true);

        addJoint(gameState, leftArmAnchor, &leftShoulder->physBody, &leftArm->physBody, true);
        addJoint(gameState, rightArmAnchor, &rightShoulder->physBody, &rightArm->physBody, true);

        addJoint(gameState, leftThighAnchor, &body->physBody, &leftThigh->physBody, true);
        addJoint(gameState, rightThighAnchor, &body->physBody, &rightThigh->physBody, true);

        addJoint(gameState, leftLegAnchor, &leftThigh->physBody, &leftLeg->physBody, true);
        addJoint(gameState, rightLegAnchor, &rightThigh->physBody, &rightLeg->physBody, true);
        
    }









    Entity* addBoxForJointDemo(GameState* gameState, float startY, int i)
    {
        Physics::PhysBodyDef def = {};
        float size = 2;
        def.halfDim = glm::vec3(size, 
                                1, 
                                0.5);
        def.mass = 5;
        def.pos = glm::vec3(i * 4,
                            startY,
                            0);

        float rot = utl::randFloat(0, 360);
        def.rot = glm::rotate(rot, glm::vec3(0, 0, 1));


        int index = gameState->numEntities++;
        Entity* entity = &gameState->entities[index];

        entity->init();
        entity->id = index;
        entity->entityType = EntityType::Box;
        entity->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));

        Physics::PhysBody* pb = &entity->physBody;
        pb->id = index;
        pb->initAsBox(def);

        return entity;
    }



    void RemoveJoint(GameState* gameState, int toRemoveIndex)
    {
        int lastJointIndex = gameState->numJoints - 1;
        if (gameState->numJoints > 0 && toRemoveIndex != lastJointIndex)
        {
            gameState->joints[toRemoveIndex] = gameState->joints[lastJointIndex];
        }
        gameState->numJoints--;
    }




    // void RemoveMouseJoint(GameState* gameState)
    void RemoveEntity(GameState* gameState, int targetIndex)
    {        
        assert(gameState->numEntities >= 1);
        Entity* mouseJointEnt = &gameState->entities[targetIndex];
        int lastIndex = gameState->numEntities - 1;


        vector<int> jointIndicesToRemove;
        // first we remove any joint on that entity
        for (int i = 0; i < gameState->numJoints; i++)
        {
            Physics::Joint* joint = &gameState->joints[i];

            if (joint->a == &gameState->entities[targetIndex].physBody)
            {
                jointIndicesToRemove.push_back(i);
            }

            if (joint->b == &gameState->entities[targetIndex].physBody)
            {
                jointIndicesToRemove.push_back(i);
            }


            if (joint->a == &gameState->entities[lastIndex].physBody)
            {
                joint->a = &gameState->entities[targetIndex].physBody;
            }

            if (joint->b == &gameState->entities[lastIndex].physBody)
            {
                joint->b = &gameState->entities[targetIndex].physBody;
            }
        }

        for (int i = 0; i < jointIndicesToRemove.size(); i++)
        {
            RemoveJoint(gameState, jointIndicesToRemove[i]);
        }

        // finally we remove the entity  
        if (gameState->numEntities > 0 && lastIndex != targetIndex)
        {
            memcpy(&gameState->entities[targetIndex], &gameState->entities[lastIndex], sizeof(Entity));
        }
        gameState->numEntities--;
    }

    void RemoveMouseJoint(GameState* gameState)
    {
        if (gameState->mouseJointEntityIndex != -1)
        {
            RemoveEntity(gameState, gameState->mouseJointEntityIndex);
            gameState->mouseJointEntityIndex = -1;

            if (jointcounter == 2)
            {
        //        debug = true;
            }
        }
    }


    void MoveMouseJoint(GameState* gameState, glm::vec2 raycastDirection)
    {
        if (gameState->mouseJointEntityIndex != -1)
        {
        //    glm::vec3 vel = (glm::vec3(raycastDirection.x, raycastDirection.y, 0) - gameState->entities[gameState->mouseJointEntityIndex].physBody.position);
        //    gameState->entities[gameState->mouseJointEntityIndex].physBody.velocity = vel;
            gameState->entities[gameState->mouseJointEntityIndex].physBody.position = glm::vec3(raycastDirection.x, raycastDirection.y, 0);
        }
    }

    void ProcessInputRaycast(GameState* gameState, glm::vec3 raycastDirection)
    {
        if (gameState->mouseJointEntityIndex == -1)
        {
            for (int i = 0; i < gameState->numEntities; i++)
            {
                Entity* entity = &gameState->entities[i];
                if (  (entity->physBody.flags & Physics::PhysBodyFlag_Static)) // || entity->isDead)
                {
                    continue;
                }
            
                if (entity->physBody.shapeData.shape == Physics::PhysBodyShape::PB_OBB)
                {                

                    if (Physics::testPointInsideOBB2D(raycastDirection, entity->physBody.shapeData.obb, entity->physBody.orientationMat, entity->physBody.position))
                    {
                        glm::vec3 anchor(raycastDirection.x, raycastDirection.y, 0);

                        Physics::PhysBodyDef physDef = {};
                        physDef.halfDim = glm::vec3(0.01f, 0.01f, 0.01);
                        physDef.mass = 1;
                        physDef.flags = Physics::PhysBodyFlag_Static;
                        physDef.pos = anchor;

                        Entity* mouseEnt = addEntity(gameState, Box, physDef);

                        addMouseJoint(gameState, anchor, &mouseEnt->physBody, &entity->physBody,  true);

                    //    gameState->mouseJointIndex = gameState->numJoints - 1;
                        gameState->mouseJointEntityIndex = gameState->numEntities - 1;

                        
                        jointcounter++;
                        if (jointcounter == 2)
                        {
                    //        debug = true;
                        }
                        break;
                    }  
                }
            }
        }
    }


    void demo1Init(GameState* gameState)
    {
        /*
        we are using a memorySection to help us intalize memory sections

        The idea is that we interpret the entire memoryStage as a giant memorySection, and then we
        are just gonna assign memorySections from it. This is becuz we have all the logic/math for memory calculate written already
        for memorySection, and we might as well use it
        */


        MemorySection masterMemorySection = {};
                
        InitializeMemorySection(&masterMemorySection, gameState->gameMemory.memoryStorage, gameState->gameMemory.memoryStorageSize);
        InitalizeSubMemorySection(&gameState->simMemorySection, &masterMemorySection, Megabytes(2));
        InitalizeSubMemorySection(&gameState->assetMemorySection, &masterMemorySection, GetRemainingMemorySize(&masterMemorySection));


//        gameState.simMemorySection = initializeMemorySection(;




        srand(0);

        gameState->frameCount = 0;
        gameState->worldWidth = 50;
        gameState->worldHeight = 50;

        /*
        gameState->numEntities = 0;
        int entitySize = sizeof(Entity);
        gameState->entities = (Entity*)malloc(256 * entitySize); // new Entity[4096];
        */

        gameState->numEntities = 0;
        gameState->entities = PushArray(&gameState->simMemorySection, 256, Entity);

        gameState->numDebugContactManifolds = 0;
        gameState->debugContactManifolds = PushArray(&gameState->simMemorySection, 1024, Physics::ContactManifold);

        gameState->numContacts = 0;
        gameState->contacts = PushArray(&gameState->simMemorySection, 1024, Physics::ContactManifold);

        gameState->numJoints = 0;
        gameState->joints = PushArray(&gameState->simMemorySection, 64, Physics::Joint);
//        gameState->joints = new Physics::Joint[64];


        gameState->mouseJointEntityIndex = -1;
        gameState->angle = 0;

        float scale = 0;
        Entity* entity = NULL;
        Physics::PhysBody* pb = NULL;
        int index = 0;
        glm::mat4 om;
        int x, y;


        scale = 100.0;
        index = gameState->numEntities++;
        entity = &gameState->entities[index];
        entity->init();
        entity->id = index;
        entity->entityType = EntityType::XYZAxis;
        entity->setModel(global.modelMgr->get(ModelEnum::xyzAxis));
        pb = &entity->physBody;
        pb->Init();
        pb->flags = Physics::PhysBodyFlag_Static;
        pb->scale = glm::vec3(scale, scale, scale);


        
        // floor needs to be at the last

        // the floor 
        index = gameState->numEntities++;
        Entity* floor = &gameState->entities[index];
        floor->init();
        floor->id = index;
        floor->entityType = EntityType::Floor;
    //    floor->isDead = false;

        pb = &floor->physBody;
        pb->Init();
        pb->shapeData.shape = Physics::PhysBodyShape::PB_PLANE;
        pb->shapeData.plane.normal = glm::vec3(0, 1, 0);
        pb->shapeData.plane.point = glm::vec3(0, 0, 0);       // dot(glm::vec3(0, 1, 0),  glm::vec3(0, 0, 0));
        pb->id = index;

        pb->position = glm::vec3(0, 0, 0);
        pb->scale = glm::vec3(gameState->worldWidth, 0.2, 0.2);

        pb->flags = Physics::PhysBodyFlag_Collides | Physics::PhysBodyFlag_Static;
        floor->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));

        
        for (int i = 0; i < 2; i++)
        {
            addRandomBox(gameState, i);
        }
        
    //    addRagdoll(gameState);


     //   prev = &floor->physBody;


        /*
        Physics::Joint* joint = &gameState->joints[gameState->numJoints];
        gameState->numJoints++;

        glm::vec3 worldAnchorPoint = glm::vec3(0, startY, 0);

        joint->a = &first->physBody;
        joint->b = prev;

        joint->aLocalAnchor = computeLocalAnchorPoint(worldAnchorPoint, &first->physBody);
        joint->bLocalAnchor = computeLocalAnchorPoint(worldAnchorPoint, prev);
        */




        int a = 1;
        /*
        Physics::Joint* joint = &gameState->joints[0];
        gameState->numJoints++;

        glm::vec3 worldAnchorPoint = glm::vec3(0, 10, 0);

        joint->a = &ent0->physBody;
        joint->b = &floor->physBody;

        joint->aLocalAnchor = computeLocalAnchorPoint(worldAnchorPoint, &ent0->physBody);
        joint->bLocalAnchor = computeLocalAnchorPoint(worldAnchorPoint, &floor->physBody);

        */


     //   utl::debug("localAnchorA", joint->aLocalAnchor);
     //   utl::debug("localAnchorB", joint->bLocalAnchor);


        /*
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
        */

        /*
        gameState->numJoints = 0;
        gameState->cycloneJoints = new Physics::CycloneJoint[64];
        */

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




    void integrateVelocity(Physics::PhysBody* pb, float dt_s)
    {
   //     std::cout << "awake " << entity->isAwake << std::endl;

   //     if (!entity->isAwake)
   //         return;

        float linearDamping = 0.0f;
        float angularDamping = 0.0f;

        pb->velocity += (pb->forceAccum / pb->mass) * dt_s;
     //   utl::debug("pbvelociuty", pb->velocity);
        // apply linear damping        
     //   entity->velocity = entity->velocity * 1.0f / (1.0f + linearDamping * dt_s);

        pb->angularVelocity += pb->inverseInertiaTensor * pb->torqueAccum * dt_s;
        // apply angular damping
    //    entity->angularVelocity = entity->angularVelocity * 1.0f / (1.0f + linearDamping * dt_s);


    }


    void integratePosition(Physics::PhysBody* pb, float dt_s, int i)
    {
  //      cout << "Integrate Position " << pb->id <<  endl;
//        if (glm::dot(entity->velocity, entity->velocity) > 0.001)
        if (true)
        {

            glm::vec3 translation = dt_s * pb->velocity;
            //    std::cout << "glm::length(translation) " << glm::length(translation) << std::endl;



            if (glm::dot(translation, translation) > (MAX_TRANSLATION * MAX_TRANSLATION))
            {

                //    utl::debug("        clamping velocity");
                float ratio = MAX_TRANSLATION / glm::length(translation);
                pb->velocity *= ratio;
            }
            /*
            utl::debug("        entityVelocity", entity->velocity);
            float vellength = glm::dot(entity->velocity, entity->velocity);
            utl::debug("        vellength", vellength);

            utl::debug("        entity->velocity * dt_s", entity->velocity * dt_s);
            */

            if (Physics::hasPolygonsCollided == true)
            {
                float angle = acos(pb->orientationMat[0][0]);
                utl::debug("        before pb->velocity", pb->velocity);
                utl::debug("        before pb->position", pb->position);
                utl::debug("        before pb->angularVelocity", pb->angularVelocity);
                utl::debug("        before pb->angle ", angle);
            }
            
            pb->position += pb->velocity * dt_s;
            if (Physics::hasPolygonsCollided == true)
            {
                
             //   utl::debug("            pb->velocity", pb->velocity);


                
            }
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
            /*
            if (Physics::hasPolygonsCollided == true)
            {
                utl::debug("            pb->angularVelocity", pb->angularVelocity);
            }
            */
            // https://math.stackexchange.com/questions/22437/combining-two-3d-rotations

            glm::vec3 rotation = dt_s * pb->angularVelocity;
            if (glm::dot(rotation, rotation) > MAX_ROTATION_SQUARED)
            {
                float ratio = MAX_ROTATION / glm::length(rotation);
                pb->angularVelocity *= ratio;
            }

            pb->addRotation(pb->angularVelocity, dt_s);
            pb->transformInertiaTensor();

            if (Physics::hasPolygonsCollided == true)
            {
                float angle = acos(pb->orientationMat[0][0]);
                utl::debug("            after pb->velocity", pb->velocity);
                utl::debug("            after pb->position", pb->position);
                utl::debug("            after pb->angularVelocity", pb->angularVelocity);
                utl::debug("            after pb->angle ", angle);
                cout << endl;

            }

        }
    }

    void CopyContactManifold(GameState* gameState, Physics::ContactManifold* contact)
    {
        gameState->debugContactManifolds[gameState->numDebugContactManifolds] = *contact;
        gameState->numDebugContactManifolds++;
    }


    void ReplaceExistingContactManifoldPoints(GameState* gameState, Physics::ContactManifold& oldContact, 
                                                                    Physics::ContactManifold& newContact)
    {
        for (int j = 0; j < newContact.numContactPoints; j++)
        {
            Physics::ContactPoint* newPoint = &newContact.contactPoints[j];

            for (int k = 0; k < oldContact.numContactPoints; k++)
            {
                Physics::ContactPoint* oldPoint = &oldContact.contactPoints[k];

                if (oldPoint->id.key == newPoint->id.key)
                {
                    // copy it over 
                    newPoint->normalImpulse = oldPoint->normalImpulse;
                    newPoint->tangentImpulse = oldPoint->tangentImpulse;

                //    cout << "replacing old contact point" << endl;

                    break;
                }
            }
        }
    }



    // if we remove items, we need to think about how to maintaint contacts->a Body and contacts->b Body
    void tryAddContactManifold(GameState* gameState, Physics::ContactManifold newContact)
    {
        bool found = false;
        for (int i = 0; i < gameState->numContacts; i++)
        {
            Physics::ContactManifold oldContact = gameState->contacts[i];

            if (oldContact.a == newContact.a && oldContact.b == newContact.b)
            {
                gameState->contacts[i] = newContact;

                ReplaceExistingContactManifoldPoints(gameState, oldContact, gameState->contacts[i]);
                found = true;
            }
        }

        if (!found)
        {

       //     cout << "adding new contact point" << endl;


            // add to my contact list
            assert(gameState->numContacts < 1024);

            int index = gameState->numContacts;
            gameState->contacts[index] = newContact;
            gameState->numContacts++;

            gameState->contacts[index].a->SetAwake(true);
            gameState->contacts[index].b->SetAwake(true);
        }

    }


    void tryRemovingInvalidContacts(GameState* gameState, Physics::PhysBody* a, Physics::PhysBody* b, vector<Physics::ContactManifold*>& manifoldsToRemove)
    {
        for (int i = 0; i < gameState->numContacts; i++)
        {
            Physics::ContactManifold* oldContact = &gameState->contacts[i];

            if (oldContact->a == a && oldContact->b == b || oldContact->a == b && oldContact->b == a)
            {
                // remove it
                // swap with last 
                int last = gameState->numContacts - 1;
                gameState->contacts[i] = gameState->contacts[last];
                gameState->numContacts--;
                break;
            }
        }
    }

    void warmStartJoint(Physics::Joint* joint)
    {
        joint->a->velocity -= joint->impulse * joint->a->invMass;
        joint->a->angularVelocity -= joint->a->inverseInertiaTensor * glm::cross(joint->rA, joint->impulse);

        if (!(joint->b->flags & Physics::PhysBodyFlag_Static))
        {
            joint->b->velocity += joint->impulse * joint->b->invMass;
            joint->b->angularVelocity += joint->b->inverseInertiaTensor * glm::cross(joint->rB, joint->impulse);
        }
    }


    void warmStart(GameState* gameState)
    {
        for (int i = 0; i < gameState->numContacts; i++)
        {
            Physics::ContactManifold* manifold = &gameState->contacts[i];
            Physics::PhysBody* a = manifold->a;
            Physics::PhysBody* b = manifold->b;

            for (int j = 0; j < manifold->numContactPoints; j++)
            {
                Physics::ContactPoint* cp = &manifold->contactPoints[j];
                glm::vec3 impulse = cp->normalImpulse * manifold->normal + cp->tangentImpulse * manifold->tangent;
#if DEBUGGING
                utl::debug("         cp->normalImpulse", cp->normalImpulse);
                utl::debug("         cp->tangentImpulse", cp->tangentImpulse);
                utl::debug("         impulse", impulse);

                utl::debug("         before a->velocity", a->velocity);
                utl::debug("         before a->angularVelocity", a->angularVelocity);
#endif
                glm::vec3 angularChange = a->inverseInertiaTensor * glm::cross(cp->relativeContactPositions[0], impulse);

#if DEBUGGING
                utl::debug("         cp->relativeContactPositions[0]", cp->relativeContactPositions[0]);
                utl::debug("         angularChange", angularChange);
#endif


                a->velocity -= impulse * a->invMass;
                a->angularVelocity -= a->inverseInertiaTensor * glm::cross(cp->relativeContactPositions[0], impulse);




                if (!(b->flags & Physics::PhysBodyFlag_Static))
                {
                    b->velocity += impulse * b->invMass;
                    b->angularVelocity += b->inverseInertiaTensor * glm::cross(cp->relativeContactPositions[1], impulse);
                }

#if DEBUGGING

                utl::debug("         after a->velocity", a->velocity);
                utl::debug("         after a->angularVelocity", a->angularVelocity);
#endif


            }
        }
    }

    bool ShouldCollide(GameState* gameState, Entity* a, Entity* b)
    {      
        if (a->physBody.DoesCollides() == false || b->physBody.DoesCollides() == false)
        {
            return false;
        }

        /*
        if ((a->physBody.flags & Physics::PhysBodyFlag_Collides) == false ||
            (b->physBody.flags & Physics::PhysBodyFlag_Collides) == false)
        {
            return false;
        }
        */

        for (int i = 0; i < gameState->numJoints; i++)
        {
            /*
            if (gameState->joints[i].isDead)
            {
                continue;
            }
            */

            if (gameState->joints[i].ignoreCollision)
            {
                if (gameState->joints[i].a == &a->physBody && gameState->joints[i].b == &b->physBody ||
                    gameState->joints[i].b == &a->physBody && gameState->joints[i].a == &b->physBody)
                {
                    return false;
                }
            }
        }
        return true;
    }

    /*
    void FogOfWar::onMouseBtnUp(GameState* gameState)
    {
        int tmpx, tmpy;
        SDL_GetMouseState(&tmpx, &tmpy);
        tmpy = utl::SCREEN_HEIGHT - tmpy;

        GameCode::RemoveMouseJoint(gameState);
    }

    void FogOfWar::onMouseBtnHold(GameState* gameState)
    {
        int tmpx, tmpy;
        SDL_GetMouseState(&tmpx, &tmpy);
        tmpy = utl::SCREEN_HEIGHT - tmpy;

        glm::vec2 screenPoint = glm::vec2(tmpx, tmpy);
        glm::vec3 worldPoint = GameCode::screenToWorldPoint(gameState, screenPoint);
        glm::vec2 tempWorldPoint = glm::vec2(worldPoint.x, worldPoint.y);

        GameCode::MoveMouseJoint(gameState, tempWorldPoint, FIXED_UPATE_TIME_s);
    }


    void onMouseBtnDown(GameState* gameState)
    {
        int tmpx, tmpy;
        SDL_GetMouseState(&tmpx, &tmpy);
        tmpy = utl::SCREEN_HEIGHT - tmpy;

        glm::vec2 screenPoint = glm::vec2(tmpx, tmpy);

        glm::vec3 worldPoint = GameCode::screenToWorldPoint(gameState, screenPoint);
        glm::vec3 raycastDir = glm::vec3(worldPoint.x, worldPoint.y, -1);

        // move this into GameCode
        cout << "OnMouseBtnDown" << endl;
        utl::debug("raycastDir", raycastDir);
        GameCode::ProcessInputRaycast(gameState, raycastDir);
    }
    */

    void beginInteract(GameState& gameState, GameInput& gameInput)
    {
        gameState.isInteracting = true;

        glm::vec2 screenPoint = glm::vec2(gameInput.mousePosition.x, gameInput.mousePosition.y);
        glm::vec3 worldPoint = GameCode::screenToWorldPoint(&gameState, screenPoint);
        glm::vec3 raycastDir = glm::vec3(worldPoint.x, worldPoint.y, -1);

        ProcessInputRaycast(&gameState, raycastDir);
    }

    void endInteract(GameState& gameState, GameInput& gameInput)
    {
        gameState.isInteracting = false;
        RemoveMouseJoint(&gameState);
    }

    void onInteracting(GameState& gameState, GameInput& gameInput)
    {
        glm::vec2 screenPoint = glm::vec2(gameInput.mousePosition.x, gameInput.mousePosition.y);
        glm::vec3 worldPoint = GameCode::screenToWorldPoint(&gameState, screenPoint);
        glm::vec2 tempWorldPoint = glm::vec2(worldPoint.x, worldPoint.y);

        MoveMouseJoint(&gameState, tempWorldPoint);
    }



    void handleGameInputMouseClicks(GameState& gameState, GameInput& gameInput)
    {
        if (gameState.isInteracting)
        {
            onInteracting(gameState, gameInput);

            for (int i = gameInput.mouseButtons[LEFT].halfTransitionCount; i > 1; i -= 2)
            {
                endInteract(gameState, gameInput);
                beginInteract(gameState, gameInput);
            }

            if (!gameInput.mouseButtons[LEFT].endedDown)
            {
                endInteract(gameState, gameInput);
            }
        }
        else
        {
            // two half Transitions is a full click, so we just do both
            for (int i = gameInput.mouseButtons[LEFT].halfTransitionCount; i > 1; i-=2)
            {
                beginInteract(gameState, gameInput);
                endInteract(gameState, gameInput);
            }

            if (gameInput.mouseButtons[LEFT].endedDown)
            {
                beginInteract(gameState, gameInput);
            }
        }
    }


    void tick(GameState* gameState, GameInput& gameInput)
    {
        handleGameInputMouseClicks(*gameState, gameInput);


        gameState->numDebugContactManifolds = 0;

        if(GRAVITY_ACTIVE)
        {
        // cout << "gameState->numEntities " << gameState->numEntities << endl;
            for (int i = 0; i < gameState->numEntities; i++)
            {

//                if (  (!(gameState->entities[i].physBody.flags & Physics::PhysBodyFlag_Static)) && (!gameState->entities[i].isDead))
                if ((!(gameState->entities[i].physBody.flags & Physics::PhysBodyFlag_Static)))
                {
                    Entity* entity = &gameState->entities[i];
                    {
                        //         if (i != 0)
                        {
                            gameState->entities[i].physBody.addForce(gameState->entities[i].physBody.mass * GRAVITY, false);
                        }
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
        }
        


        vector<Physics::ContactManifold*> manifoldsToRemove;

        if (COLLISION_ACTIVE)
        {
            for (int i = 0; i < gameState->numEntities; i++)
            {
                Entity* ent0 = &gameState->entities[i];

                for (int j = i + 1; j < gameState->numEntities; j++)
                {
                    Entity* ent1 = &gameState->entities[j];

                    if (ShouldCollide(gameState, ent0, ent1))
                    {
                        //    cout << i <<  " " << j << endl;



        //                bool activeA = ent0->physBody.isAwake && (!(ent0->physBody.flags & Physics::PhysBodyFlag_Static));
        //                bool activeB = ent1->physBody.isAwake && (!(ent1->physBody.flags & Physics::PhysBodyFlag_Static));

                        bool activeA = (ent0->physBody.flags & Physics::PhysBodyFlag_Static);
                        bool activeB = (ent1->physBody.flags & Physics::PhysBodyFlag_Static);


                        // if no one is active, we dont do shit
                        if (activeA && activeB)
                        {
                            continue;
                        }

                        Physics::ContactManifold newContact = {};

                     //   if ( gameState->entities[i].isDead == false && gameState->entities[j].isDead == false)
                        {
                            Physics::GenerateContactInfo(&gameState->entities[i].physBody, &gameState->entities[j].physBody, newContact);
                            //    cout << newContact.numContactPoints << endl;
                        }

                        if (newContact.numContactPoints > 0)
                        {
                            // if doenst exists in contacts list, add it
                            tryAddContactManifold(gameState, newContact);
                        }
                        else
                        {
                            // remove contact between the two if there are any in our old list
                            tryRemovingInvalidContacts(gameState, &gameState->entities[i].physBody,
                                &gameState->entities[j].physBody, manifoldsToRemove);
                        }                        
                    }
                }
            }
        }



        for (int i = 0; i < gameState->numEntities; i++)
        {
            /*
            if (gameState->entities[i].isDead)
            {
                continue;
            }
            */
//            if (!(gameState->entities[i].physBody.flags & Physics::PhysBodyFlag_Static) && gameState->entities[i].physBody.isAwake )
            if (!(gameState->entities[i].physBody.flags & Physics::PhysBodyFlag_Static))
            {
                integrateVelocity(&gameState->entities[i].physBody, gameInput.dt_s);
            }
        }

        if (debug)
        {
            cout << ">>>>>>>>>>> line3" << endl;
            utl::debug("id ", gameState->entities[3].id);
            utl::debug("vel ", gameState->entities[3].physBody.velocity);
            utl::debug("pos ", gameState->entities[3].physBody.position);
        }

        for (int i = 0; i < gameState->numContacts; i++)
        {
            if (gameState->contacts[i].numContactPoints > 0)
            {
                Physics::InitVelocityConstraints(gameState->contacts[i], gameState->contacts[i].a, gameState->contacts[i].b);
            }
        }


        warmStart(gameState);



        for (int i = 0; i < gameState->numJoints; i++)
        {
            Physics::Joint* joint = &gameState->joints[i];
            {
                Physics::InitJointVelocityConstraints(joint);
                warmStartJoint(joint);
            }
        }



   //     cout << "gameState->numContacts" << gameState->numContacts << endl;

        int velocityIterations = 4;
        for (int i = 0; i < velocityIterations; i++)
        {

            for (int j = 0; j < gameState->numJoints; j++)
            {
          //      if (!gameState->joints[j].isDead)
                {
                    Physics::SolveJointVelocityConstraints(gameState->joints[j]);
                }
            }

            for (int j = 0; j < gameState->numContacts; j++)
            {
                if (gameState->contacts[j].numContactPoints > 0)
                {
                    Physics::ResolveVelocity(gameState->contacts[j], gameState->contacts[j].a, gameState->contacts[j].b, gameInput.dt_s);
                }
            }
        }

        if (debug)
        {
            cout << ">>>>>>>>>>> line4" << endl;
            utl::debug("id ", gameState->entities[3].id);
            utl::debug("vel ", gameState->entities[3].physBody.velocity);
            utl::debug("pos ", gameState->entities[3].physBody.position);
        }


                //          cout << "Colliding" << i << " " << j << endl;

 

        
        /*
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
                Physics::ContactManifold contact = {};


                
                Physics::GenerateContactInfo(&gameState->entities[i].physBody, &gameState->entities[j].physBody, contact);
                if (contact.numContactPoints > 0)
                {                    

          //          cout << "Colliding" << i << " " << j << endl;

                    CopyContactManifold(gameState, &contact);
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
        */

        
        // by applying velocity first, we may get to skip doing position resolution. so this saves us some computation
        for (int i = 0; i < gameState->numEntities; i++)
        {/*
            if (gameState->entities[i].isDead)
            {
                continue;
            }
            */
//            if (!(gameState->entities[i].physBody.flags & Physics::PhysBodyFlag_Static) && gameState->entities[i].physBody.isAwake)
            if (!(gameState->entities[i].physBody.flags & Physics::PhysBodyFlag_Static))
            {
                integratePosition(&gameState->entities[i].physBody, gameInput.dt_s, i);
            }
        }


        if (debug)
        {
            cout << ">>>>>>>>>>> line5" << endl;
            utl::debug("vel ", gameState->entities[3].physBody.velocity);
            utl::debug("pos ", gameState->entities[3].physBody.position);
        }

        int positionIterations = 3;
        bool positionDone = false;
        for (int i = 0; i < positionIterations; i++)
        {            
            positionDone = Physics::ResolvePosition(gameState->contacts, gameState->numContacts, gameInput.dt_s, i);

            for (int j = 0; j < gameState->numJoints; j++)
            {
          //      if (!gameState->joints[j].isDead)
                {
                    positionDone &= Physics::SolveJointPositionConstraints(gameState->joints[j]);
                }
            }

            if (positionDone)
            {
                break;
            }
        }

        if (debug)
        {
            cout << ">>>>>>>>>>> line6" << endl;
            utl::debug("vel ", gameState->entities[3].physBody.velocity);
            utl::debug("pos ", gameState->entities[3].physBody.position);
        }

        // check for sleeping 
        if (ALLOW_SLEEPING)
        {

            float angToleranceSqr = ANGULAR_SLEEP_TOLERANCE * ANGULAR_SLEEP_TOLERANCE;
            float linToleranceSqr = LINEAR_SLEEP_TOLERANCE * LINEAR_SLEEP_TOLERANCE;


            for (int i = 0; i < gameState->numEntities; i++)
            {
                /*
                if (gameState->entities[i].isDead)
                {
                    continue;
                }
                */
                if (gameState->entities[i].physBody.flags & Physics::PhysBodyFlag_Static)
                {
                    continue;
                }

                Physics::PhysBody* pb = &gameState->entities[i].physBody;

                if (glm::dot(pb->angularVelocity, pb->angularVelocity) > angToleranceSqr ||
                    glm::dot(pb->velocity, pb->velocity) > linToleranceSqr)
                {
                    pb->dormantTimer = 0.0;
                }
                else
                {
                    pb->dormantTimer += gameInput.dt_s;
                }

                if (pb->dormantTimer > DORMANT_TIME_TO_SLEEP)
                {
                    pb->SetAwake(false);
                }

            }



        }




        for (int i = 0; i < gameState->numEntities; i++)
        {
            if (!(gameState->entities[i].physBody.flags & Physics::PhysBodyFlag_Static))
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
        gameState->frameCount++;
    }
};


#endif