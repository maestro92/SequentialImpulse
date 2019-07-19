
#include <assert.h>
#include "model.h"

using namespace std;



Model::Model()
{
	m_isAnimated = false;
    m_modelGeometry = GL_TRIANGLES;
}


Model::~Model()
{
    clear();
}


void Model::clear()
{

}


void Model::addMesh(Mesh m)
{
    m_meshes.push_back(m);
}




void Model::render()
{
    /// this means we're going to use these two buffer object
    for (int i=0; i<m_meshes.size(); i++)
    {
        Mesh m = m_meshes[i];

        glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.ind);

        glVertexAttribPointer(POSITION_VERTEX_ATTRIB,   3, GL_FLOAT, GL_FALSE, sizeof(VertexData), 0);
        glVertexAttribPointer(NORMAL_VERTEX_ATTRIB,   3, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)(3*sizeof(float)));
        glVertexAttribPointer(COLOR_VERTEX_ATTRIB,    3, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)(6*sizeof(float)));
        glVertexAttribPointer(UV_VERTEX_ATTRIB,       2, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)(9*sizeof(float)));


        const unsigned int texIndex = m_meshes[i].textureIndex;


        if (texIndex < m_textures.size() && m_textures[texIndex].m_id != -1)
        {
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, m_textures[texIndex].m_id);
        }


        /// glDrawElements, you need to supply an index buffer
        /// glDrawArrays submits the vertices in linear order
        glDrawElements(m_modelGeometry, m.numIndices, GL_UNSIGNED_INT, 0);
    }
}


#if 0
void Model::renderSingle()
{
	enableVertexAttribArrays();
	render();
	/*
		/// this means we're going to use these two buffer object
	for (int i = 0; i<m_meshes.size(); i++)
	{
		Mesh m = m_meshes[i];

		glBindBuffer(GL_ARRAY_BUFFER, m.m_vbo);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.m_ind);

		glVertexAttribPointer(POSITION_VERTEX_ATTRIB, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), 0);
		glVertexAttribPointer(NORMAL_VERTEX_ATTRIB, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)(3 * sizeof(float)));
		glVertexAttribPointer(COLOR_VERTEX_ATTRIB, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)(6 * sizeof(float)));
		glVertexAttribPointer(UV_VERTEX_ATTRIB, 2, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)(9 * sizeof(float)));


		const unsigned int texIndex = m_meshes[i].m_textureIndex;


		if (texIndex < m_textures.size() && m_textures[texIndex].m_id != -1)
		{
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, m_textures[texIndex].m_id);
		}


		/// glDrawElements, you need to supply an index buffer
		/// glDrawArrays submits the vertices in linear order
		glDrawElements(m_modelGeometry, m.m_numIndices, GL_UNSIGNED_INT, 0);
	}
	*/
	disableVertexAttribArrays();
}
#endif

void Model::setTextures(vector<string> textureFiles)
{
	m_textures.clear();
	for (int i = 0; i < textureFiles.size(); i++)
	{
		TextureData tex;
		tex.m_id = utl::loadTexture(textureFiles[i], GL_LINEAR_MIPMAP_LINEAR, GL_REPEAT, true);
		m_textures.push_back(tex);
	}
}


void Model::setMeshRandTextureIdx()
{
	int texCount = m_textures.size();

	if (texCount <= 0)
		return;

	for (int i = 0; i < m_meshes.size(); i++)
		m_meshes[i].textureIndex = 0;
	
}

void Model::setModelGemoetry(GLuint geometry)
{
	m_modelGeometry = geometry;
}


bool Model::isAnimated()
{
	return m_isAnimated;
}

