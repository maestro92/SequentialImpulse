
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

    int numDebugContactManifolds;
    Physics::ContactManifold* debugContactManifolds;

    int numContacts;
    Physics::ContactManifold* contacts;

    Camera mainCamera;

    Entity* draggedEntity;

    float angle;
    int boxIndex;

    int frameCount;
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
        */
    void addRandomBox(GameState* gameState, int height)
    {


        float size = 2;
        float halfWidth = size;
        float halfHeight = size;
        float halfDepth = 0.5;

//        float halfWidth = size * utl::randInt(1, 3);
//        float halfHeight = size * utl::randInt(1, 3);
//        float halfDepth = 0.5;

        int index = gameState->numEntities++;
        Entity* entity = &gameState->entities[index];

      
        float x = 0;
        float y = 5 + height * 5;// utl::randFloat(5, 15);
     
        float rot = 0;// utl::randFloat(0, 360);
       
        /*
        float x = utl::randFloat(-20, 20);
        float y = utl::randFloat(5, 30);
        float rot = utl::randFloat(0, 360);
        */

        entity->init();
        entity->id = index;
        entity->entityType = EntityType::Box;

        entity->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));

        Physics::PhysBody* pb = &entity->physBody;
        pb->Init();
        pb->id = index;
        pb->flags = Physics::PhysBodyFlag_Collides;
        pb->mass = 5;
        pb->invMass = 1 / (float)pb->mass;
        pb->position = glm::vec3(x, y, 0);
        glm::mat4 om = glm::rotate(rot, glm::vec3(0, 0, 1));
        //         om = glm::rotate(0.0f, glm::vec3(0, 0, 1));

     //   om = glm::rotate(60.0f, glm::vec3(0, 0, 1));




        pb->orientation = glm::toQuat(om);
        pb->SyncOrientationMat();
        pb->scale = glm::vec3(halfWidth, halfHeight, halfDepth);



        float angle = atan2(pb->orientationMat[1][0], pb->orientationMat[0][0]) * 180.0f / 3.14f;
        utl::debug("        before a->angle ", angle);


        pb->velocityDamping = 0.95f;
        pb->angularDamping = 0.80f;

        pb->inertiaTensor = Physics::GetBoxInertiaTensor(pb->mass, halfWidth * 2, halfHeight * 2, halfDepth * 2);
        pb->transformInertiaTensor();


        pb->shapeData.shape = Physics::PhysBodyShape::PB_OBB;
        pb->shapeData.obb.center = glm::vec3(0, 0, 0);
        pb->shapeData.obb.axes[0] = glm::vec3(1, 0, 0);
        pb->shapeData.obb.axes[1] = glm::vec3(0, 1, 0);
        pb->shapeData.obb.axes[2] = glm::vec3(0, 0, 1);
        pb->shapeData.obb.halfEdges = glm::vec3(halfWidth, halfHeight, halfDepth);


    }


    void ProcessInputRaycast(GameState* gameState, glm::vec3 raycastDirection)
    {
        for (int i = 0; i < gameState->numEntities; i++)
        {
            Entity* entity = &gameState->entities[i];
            if (!(entity->physBody.flags & Physics::PhysBodyFlag_Static))
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
        srand(0);

        gameState->frameCount = 0;
        gameState->worldWidth = 50;
        gameState->worldHeight = 50;
        gameState->numEntities = 0;

        int entitySize = sizeof(Entity);
        gameState->entities = (Entity*)malloc(256 * entitySize); // new Entity[4096];

        gameState->numDebugContactManifolds = 0;
        //    gameState->contactManifold = new contactManifoldglm::vec3[1024];

        gameState->debugContactManifolds = new Physics::ContactManifold[256];

        gameState->numContacts = 0;
        gameState->contacts = new Physics::ContactManifold[256];

        gameState->angle = 0;
        gameState->draggedEntity = NULL;

        float scale = 100.0;
        Entity* entity = NULL;
        Physics::PhysBody* pb = NULL;
        int index = 0;
      
        
        /*
        index = gameState->numEntities++;
        entity = &gameState->entities[index];
        entity->init();
        entity->id = gameState->numEntities;
        entity->entityType = EntityType::XYZAxis;

        entity->setModel(global.modelMgr->get(ModelEnum::xyzAxis));

        pb = &entity->physBody;
        pb->Init();
        pb->flags = Physics::PhysBodyFlag_Static;
        pb->scale = glm::vec3(scale, scale, scale);
        */
        

        // the box
        glm::mat4 om;
        int x, y;


        x = 0;
        y = 2;
        float size = 0.5;
        float halfWidth = size * 2;
        float halfHeight = size * 2;
        float halfDepth = 1;





#if 0
        // the box
        x = 0;
        y = 19.5; // 4.5
        size = 1;
        halfWidth = size;
        halfHeight = size;
        halfDepth = 1;
        index = gameState->numEntities++;
        entity = &gameState->entities[index];
        entity->init();
        entity->id = index;
        entity->entityType = EntityType::Box;

        entity->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));

        pb = &entity->physBody;
        pb->Init();
        pb->id = index;
        pb->mass = 1;
        pb->invMass = 1 / (float)pb->mass;
        pb->position = glm::vec3(x, y, 0);
        pb->flags = Physics::PhysBodyFlag_Collides;

        om = glm::rotate(50.0f, glm::vec3(0, 0, 1));
        //   om = glm::rotate(0.0f, glm::vec3(0, 0, 1));

        pb->orientation = glm::toQuat(om);
        pb->SyncOrientationMat();
        pb->scale = glm::vec3(halfWidth, halfHeight, halfDepth);

        pb->velocityDamping = 0.95f;
        pb->angularDamping = 0.80f;

        pb->inertiaTensor = Physics::GetBoxInertiaTensor(pb->mass, halfWidth * 2, halfHeight * 2, halfDepth);
        pb->transformInertiaTensor();


        pb->shapeData.shape = Physics::PhysBodyShape::PB_OBB;
        pb->shapeData.obb.center = glm::vec3(0, 0, 0);
        pb->shapeData.obb.axes[0] = glm::vec3(1, 0, 0);
        pb->shapeData.obb.axes[1] = glm::vec3(0, 1, 0);
        pb->shapeData.obb.axes[2] = glm::vec3(0, 0, 1);
        pb->shapeData.obb.halfEdges = glm::vec3(halfWidth, halfHeight, halfDepth);



        x = 0;
        y = 10;
        index = gameState->numEntities++;
        entity = &gameState->entities[index];
        entity->init();
        entity->id = index;
        entity->entityType = EntityType::Box;

        entity->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));


        pb = &entity->physBody;
        pb->Init();
        pb->id = index;

        pb->flags = Physics::PhysBodyFlag_Collides;
        pb->mass = 5;
        pb->invMass = 1 / (float)pb->mass;
        pb->position = glm::vec3(x, y, 0);
        om = glm::rotate(0.0f, glm::vec3(0, 0, 1));
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
        pb->scale = glm::vec3(halfWidth, halfHeight, halfDepth);

        pb->velocityDamping = 0.95f;
        pb->angularDamping = 0.80f;

        pb->inertiaTensor = Physics::GetBoxInertiaTensor(pb->mass, halfWidth * 2, halfHeight * 2, halfDepth * 2);
        pb->transformInertiaTensor();


        pb->shapeData.shape = Physics::PhysBodyShape::PB_OBB;
        pb->shapeData.obb.center = glm::vec3(0, 0, 0);
        pb->shapeData.obb.axes[0] = glm::vec3(1, 0, 0);
        pb->shapeData.obb.axes[1] = glm::vec3(0, 1, 0);
        pb->shapeData.obb.axes[2] = glm::vec3(0, 0, 1);
        pb->shapeData.obb.halfEdges = glm::vec3(halfWidth, halfHeight, halfDepth);

#endif 

        for (int i = 0; i < 10; i++)
        {
            addRandomBox(gameState, i);
        }
        // addRandomBox(gameState);
        // addRandomBox(gameState);
        /*
        addRandomBox(gameState);
        addRandomBox(gameState);
        addRandomBox(gameState);
        addRandomBox(gameState);
        addRandomBox(gameState);
        */


        
        
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
//        pb->shapeData.plane.offset = 0;       // dot(glm::vec3(0, 1, 0),  glm::vec3(0, 0, 0));
        pb->shapeData.plane.point = glm::vec3(0,0,0);       // dot(glm::vec3(0, 1, 0),  glm::vec3(0, 0, 0));
        pb->id = index;

        pb->position = glm::vec3(0, 0, 0);
        pb->scale = glm::vec3(gameState->worldWidth, 0.02, 0.2);

        pb->flags = Physics::PhysBodyFlag_Collides | Physics::PhysBodyFlag_Static;
        entity->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));
        

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
        float maxTranslation = 2.0f;

  //      cout << "Integrate Position " << pb->id <<  endl;
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
            assert(gameState->numContacts < 256);

            int index = gameState->numContacts;
            gameState->contacts[index] = newContact;
            gameState->numContacts++;
        }

    }


    void tryRemovingInvalidContacts(GameState* gameState, Physics::PhysBody* a, Physics::PhysBody* b, vector<Physics::ContactManifold*>& manifoldsToRemove)
    {
        for (int i = 0; i < gameState->numContacts; i++)
        {
            Physics::ContactManifold* oldContact = &gameState->contacts[i];

            if (oldContact->a == a && oldContact->b == b)
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

    
    void tick(GameInput gameInput, GameState* gameState)
    {
        
        gameState->numDebugContactManifolds = 0;


        // cout << "gameState->numEntities " << gameState->numEntities << endl;
        for (int i = 0; i < gameState->numEntities; i++)
        {
            if (!(gameState->entities[i].physBody.flags & Physics::PhysBodyFlag_Static) && &gameState->entities[i] != gameState->draggedEntity)
            {
                Entity* entity = &gameState->entities[i];
                {
                //    if (i != 0)
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
        


    //    cout << "########## newTick " << gameState->frameCount<< endl;
        if (gameState->frameCount  == 78)
        {
            int c = 1;
        }

        vector<Physics::ContactManifold*> manifoldsToRemove;

        for (int i = 0; i < gameState->numEntities; i++)
        {
            if (!gameState->entities[i].physBody.flags & Physics::PhysBodyFlag_Static)
            {
                continue;
            }

            for (int j = i + 1; j < gameState->numEntities; j++)
            {
                Physics::ContactManifold newContact = {};

                Physics::GenerateContactInfo(&gameState->entities[i].physBody, &gameState->entities[j].physBody, newContact);
            //    cout << newContact.numContactPoints << endl;

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



        for (int i = 0; i < gameState->numEntities; i++)
        {
            if (!(gameState->entities[i].physBody.flags & Physics::PhysBodyFlag_Static))
            {
                integrateVelocity(&gameState->entities[i].physBody, gameInput.dt_s);
        //        integratePosition(&gameState->entities[i], gameInput.dt_s);
            }
        }

        for (int i = 0; i < gameState->numContacts; i++)
        {
            if (gameState->contacts[i].numContactPoints > 0)
            {
                Physics::PrepareContactPoints(gameState->contacts[i], gameState->contacts[i].a, gameState->contacts[i].b);
            }
        }

        warmStart(gameState);


   //     cout << "gameState->numContacts" << gameState->numContacts << endl;

        int velocityIterations = 4;
        for (int i = 0; i < velocityIterations; i++)
        {
            for (int j = 0; j < gameState->numContacts; j++)
            {

                if (gameState->contacts[j].numContactPoints > 0)
                {
                //    CopyContactManifold(gameState, &gameState->contacts[j]);
                    Physics::ResolveVelocity(gameState->contacts[j], gameState->contacts[j].a, gameState->contacts[j].b, gameInput.dt_s);

                }

            }
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
        {
            if (!(gameState->entities[i].physBody.flags & Physics::PhysBodyFlag_Static))
            {
                integratePosition(&gameState->entities[i].physBody, gameInput.dt_s, i);
            }
        }


        int positionIterations = 3;
        bool positionDone = false;
        for (int i = 0; i < positionIterations; i++)
        {            
            positionDone = Physics::ResolvePosition(gameState->contacts, gameState->numContacts, gameInput.dt_s, i);
            if (positionDone)
            {
                break;
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