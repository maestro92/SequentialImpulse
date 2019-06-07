#ifndef MODEL_H_
#define	MODEL_H_


#define NO_SDL_GLEXT
#include <GL/glew.h>

#include "define.h"
#include "utility_gl.h"
#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <string>
#include <sstream>
#include <vector>

#include "texture.h"

using namespace std;

#define INVALID_MATERIAL 0xFFFFFFFF
#define INVALID_GL_VALUE -1

#include "model_constant.h"


/*
http://gamedev.stackexchange.com/questions/19560/what-is-the-best-way-to-store-meshes-or-3d-models-in-a-class
*/







#include "Mesh.h"

class Model
{
    public:
		
        // TODO: remove CubeModel and QuadModel, make them functional

        Model();
        virtual ~Model();

        void addMesh(Mesh m);
		void setTextures(vector<string> textureFiles);
		void setMeshRandTextureIdx();
		virtual void render();
		void setModelGemoetry(GLuint geometry);

        void clear();



		vector<TextureData> m_textures;
			
		bool isAnimated();

	protected:

		vector<Mesh> m_meshes;

		bool m_isAnimated;
		GLuint m_modelGeometry;
};


#endif // MESH37_H
