
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
    Player mainPlayer;
};


namespace GameCode
{
    glm::vec3 GRAVITY = glm::vec3(0, -0.01, 0);

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

    void demo1Init(GameState* gameState)
    {
        gameState->numEntities = 0;
        gameState->worldWidth = 50;
        gameState->worldHeight = 50;
        gameState->entities = new Entity[4096];

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
        entity->position = glm::vec3(x, y, 0);
        entity->scale = glm::vec3(size, size, size);
        entity->setModel(global.modelMgr->get(ModelEnum::unitCenteredQuad));
        
        entity->physBody.type = Physics::PhysBodyType::PB_OBB;
        entity->physBody.obb.center = glm::vec3(0, 0, 0);
        entity->physBody.obb.axes[0] = glm::vec3(1, 0, 0);
        entity->physBody.obb.axes[1] = glm::vec3(0, 1, 0);
        entity->physBody.obb.axes[2] = glm::vec3(0, 0, 1);
        entity->physBody.obb.halfEdges = glm::vec3(size, size, size);
        

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





    void initPlayer(GameState* gameState)
    {
        gameState->mainPlayer.render.setModel(global.modelMgr->get(ModelEnum::centeredQuad));
        gameState->mainPlayer.transform.setScale(0.8);

        gameState->mainPlayer.vision = 8;
        gameState->mainPlayer.simPos = glm::vec2(0, 0);
        gameState->mainPlayer.transform.position = glm::vec3(gameState->mainPlayer.simPos.x, gameState->mainPlayer.simPos.y, 0);

        //	vector<FogCell> dirtyFogCells;

        //	fogManager.setSource(map.simPos2GridCoord(mainPlayer.simPos), mainPlayer.vision, FogManager::VISIBLE, dirtyFogCells);
        //	fogView.addDirtyCells(dirtyFogCells);
    }

    void init(GameState* gameState)
    {
        demo1Init(gameState);        
    }




    void tick(GameInput gameInput, GameState* gameState)
    {
        
        // cout << "gameState->numEntities " << gameState->numEntities << endl;
        for (int i = 0; i < gameState->numEntities; i++)
        {
            if (!(gameState->entities[i].flags & EntityFlag_Static))
            {
                gameState->entities[i].velocity += GRAVITY;
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
                gameState->entities[i].position += gameInput.dt_s * gameState->entities[i].velocity;
            }
        }

    }
};


#endif