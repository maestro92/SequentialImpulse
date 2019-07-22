
#include "renderer_manager.h"


RendererManager::RendererManager()
{

}

RendererManager::~RendererManager()
{

}


void RendererManager::init(int width, int height)
{
	char* filename = "renderer/renderer_data.json";

	Value vContent = utl::readJsonFileToVector(filename);
	const Array& vArray = vContent.get_array();

	Renderer::initRendererWrapper(vArray, &r_fullVertexColor, "r_fullVertexColor");
	Renderer::initRendererWrapper(vArray, &r_fullColor, "r_fullColor");
	Renderer::initRendererWrapper(vArray, &r_fullTexture, "r_fullTexture");
	Renderer::initRendererWrapper(vArray, &r_map, "r_map");
	Renderer::initRendererWrapper(vArray, &r_fow, "r_fow");
	Renderer::initRendererWrapper(vArray, &r_fogEdgeBlur, "r_fogEdgeBlur");
	Renderer::initRendererWrapper(vArray, &r_fogFadeUpdate, "r_fogFadeUpdate");
}

