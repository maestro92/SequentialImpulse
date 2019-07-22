#ifndef RENDERER_MANAGER_H_
#define RENDERER_MANAGER_H_

#include "renderer_constants.h"
#include "renderer.h"
#include "global.h"

class RendererManager
{
    public:
        RendererManager();
        ~RendererManager();

        Renderer		r_fullVertexColor;
		Renderer		r_fullColor;
		Renderer		r_fullTexture;
		Renderer		r_map;
		Renderer		r_fow;
		Renderer		r_fogEdgeBlur;
		Renderer		r_fogFadeUpdate;
		void init(int width, int height);
};

#endif
