#ifndef GAME_RENDERING_U_
#define GAME_RENDERING_U_


#include "global.h"
#include "renderer_manager.h"
#include "game.h"


namespace GameRendering
{

    void render(GameState* gameState)
    {
        gameState->mainCamera.getPipeline().setMatrixMode(MODEL_MATRIX);
        glBindFramebuffer(GL_FRAMEBUFFER, RENDER_TO_SCREEN);
        glViewport(0, 0, utl::SCREEN_WIDTH, utl::SCREEN_HEIGHT);

        gameState->mainCamera.getPipeline().setMatrixMode(VIEW_MATRIX);
        gameState->mainCamera.getPipeline().loadIdentity();
        gameState->mainCamera.getPipeline().translate(0.0f, 0.0f, 5.0f);


        gameState->mainCamera.getPipeline().setMatrixMode(MODEL_MATRIX);

        glClearColor(1.0, 1.0, 1.0, 1.0);
        glClear(GL_COLOR_BUFFER_BIT);
        glDepthMask(true);

        glCullFace(GL_BACK);
        glClear(GL_DEPTH_BUFFER_BIT);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        //	mapView.render(mainCamera.getPipeline());

        Entity* entities = gameState->entities;

        // Rendering wireframes
        Renderer* p_renderer = &global.rendererMgr->r_fullVertexColor;
        p_renderer->enableShader();

        gameState->entities[0].renderCore(gameState->mainCamera.getPipeline(), p_renderer);
        p_renderer->disableShader();

        p_renderer = &global.rendererMgr->r_fullColor;
        p_renderer->enableShader();
        p_renderer->setData(R_FULL_COLOR::u_color, COLOR_RED);

        for (int i = 1; i < gameState->numEntities; i++)
        {
            if (entities[i].flags & EntityFlag_Static)
            {
                p_renderer->setData(R_FULL_COLOR::u_color, COLOR_GRAY);
            }
            else if (entities[i].entityType == EntityType::Box)
            {
                p_renderer->setData(R_FULL_COLOR::u_color, COLOR_BLUE);
            }

            gameState->entities[i].renderCore(gameState->mainCamera.getPipeline(), p_renderer);
        }
        /*
        if (currentRay != NULL && currentRay->canRender())
        {
        p_renderer->setData(R_FULL_COLOR::u_color, COLOR_RED);
        currentRay->renderGroup(mainCamera.getPipeline(), p_renderer);
        }

        if (sourcePoint.canRender())
        {
        p_renderer->setData(R_FULL_COLOR::u_color, COLOR_TEAL);
        sourcePoint.renderGroup(mainCamera.getPipeline(), p_renderer);
        }

        if (endPoint.canRender())
        {
        p_renderer->setData(R_FULL_COLOR::u_color, COLOR_GREEN);
        endPoint.renderGroup(mainCamera.getPipeline(), p_renderer);
        }
        */
        p_renderer->disableShader();

    }
};
#endif // ! GAME_RENDERING_U_