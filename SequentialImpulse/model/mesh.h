#ifndef MESH_H_
#define MESH_H_

#include "vertex_data.h"

#include <GL/glew.h>

#include <vector>
#include <string>

using namespace std;


struct Mesh
{
    GLuint vbo;
    GLuint ind;
    int numIndices;
    unsigned int textureIndex;

    Mesh()
    {}

    Mesh(vector<VertexData>& Vertices, vector<unsigned int>& Indices) : Mesh(Vertices, Indices, -1)
    { }

    Mesh(vector<VertexData>& Vertices, vector<unsigned int>& Indices, unsigned int texIndex)
    {
        numIndices = Indices.size();

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(VertexData) * Vertices.size(), &Vertices[0], GL_STATIC_DRAW);

        glGenBuffers(1, &ind);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ind);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * numIndices, &Indices[0], GL_STATIC_DRAW);

        /// we unbind the buffers, so that no one accidentally unbind the buffers
        glBindBuffer(GL_ARRAY_BUFFER,0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
        textureIndex = texIndex;
    }
};

#endif // Mesh_H_
