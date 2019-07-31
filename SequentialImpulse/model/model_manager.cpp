#include "model_manager.h"

/*

when working with models in blender
the -z axis in game is the y axis in blender, 

so to be sure to orient the models in the right direction


*/


namespace Asset
{

};

void ModelManager::init()
{
    buildQuadModel(&m_models[ModelEnum::unitCenteredQuad], glm::vec3(-1,-1, -1), glm::vec3(1, 1, 1));
    buildQuadOutlineModel(&m_models[ModelEnum::unitCenteredQuadOutline], glm::vec3(-1, -1, -1), glm::vec3(1, 1, 1), 0.1);
    buildXYZModel(&m_models[ModelEnum::xyzAxis]);
    buildCircleOutlineModel(&m_models[ModelEnum::circleOutline], 1, 0.1);
    buildArrowModel(&m_models[ModelEnum::arrow]);
}

void ModelManager::enableVertexAttribArrays()
{
	glEnableVertexAttribArray(POSITION_VERTEX_ATTRIB);
	glEnableVertexAttribArray(NORMAL_VERTEX_ATTRIB);
	glEnableVertexAttribArray(COLOR_VERTEX_ATTRIB);
	glEnableVertexAttribArray(UV_VERTEX_ATTRIB);
	glEnableVertexAttribArray(BONE_IDS_ATTRIB);
	glEnableVertexAttribArray(BONE_WEIGHTS_ATTRIB);
}

void ModelManager::disableVertexAttribArrays()
{
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glDisableVertexAttribArray(POSITION_VERTEX_ATTRIB);
	glDisableVertexAttribArray(NORMAL_VERTEX_ATTRIB);
	glDisableVertexAttribArray(COLOR_VERTEX_ATTRIB);
	glDisableVertexAttribArray(UV_VERTEX_ATTRIB);
	glDisableVertexAttribArray(BONE_IDS_ATTRIB);
	glDisableVertexAttribArray(BONE_WEIGHTS_ATTRIB);
}

void ModelManager::buildLinesBetweenTwoPoints2D(glm::vec2 p0, glm::vec2 p1, float thickness,
    vector<VertexData>& vertices,
    vector<unsigned int>& indices)
{
    glm::vec3 pos0 = glm::vec3(p0.x, p0.y, 0);
    glm::vec3 pos1 = glm::vec3(p1.x, p1.y, 0);
    buildLinesBetweenTwoPoints3D(pos0, pos1, thickness, vertices, indices);
}

void ModelManager::buildLinesBetweenTwoPoints3D(glm::vec3 p0, glm::vec3 p1, float thickness,
	vector<VertexData>& vertices,
	vector<unsigned int>& indices)
{
	VertexData tmp;

	glm::vec3 dir = p1 - p0;
	dir = glm::normalize(dir);

	glm::vec3 Z_AXIS = glm::vec3(0.0f, 0.0f, -1.0f);

	glm::vec3 normal = glm::cross(dir, Z_AXIS);
	normal = glm::normalize(normal);


	glm::vec3 pos0 = p1 - normal * thickness;
	glm::vec3 pos1 = p0 - normal * thickness;
	glm::vec3 pos2 = p1 + normal * thickness;
	glm::vec3 pos3 = p0 + normal * thickness;
	
	int indicesStart = vertices.size();

	/// 0. bot left
	tmp.position = pos0;
	vertices.push_back(tmp);

	/// 1. bot right
	tmp.position = pos1;
	vertices.push_back(tmp);

	/// 2. top right
	tmp.position = pos2;
	vertices.push_back(tmp);

	/// 3. top left
	tmp.position = pos3;
	vertices.push_back(tmp);

	indices.push_back(indicesStart + 1);
	indices.push_back(indicesStart + 2);
	indices.push_back(indicesStart + 0);

	indices.push_back(indicesStart + 1);
	indices.push_back(indicesStart + 3);
	indices.push_back(indicesStart + 2);
}

void ModelManager::buildQuad2D(glm::vec2 min, glm::vec2 max, glm::vec3 color,
	vector<VertexData>& vertices,
	vector<unsigned int>& indices)
{
	glm::vec3 pos0 = glm::vec3(min.x, min.y, 0);
	glm::vec3 pos1 = glm::vec3(max.x, max.y, 0);
	buildQuad3D(pos0, pos1, color, vertices, indices);
}


void ModelManager::buildQuad3D(glm::vec3 min, glm::vec3 max, glm::vec3 color,
	vector<VertexData>& vertices,
	vector<unsigned int>& indices)
{
	VertexData tmp;

	glm::vec3 pos0 = glm::vec3(min.x, min.y, 0);
	glm::vec3 pos1 = glm::vec3(max.x, min.y, 0);
	glm::vec3 pos2 = glm::vec3(max.x, max.y, 0);
	glm::vec3 pos3 = glm::vec3(min.x, max.y, 0);


	float uv_x0 = 0;  float uv_x1 = 1;
	float uv_y0 = 0;  float uv_y1 = 1;

	int indicesStart = vertices.size();

	/// 0. bot left
	tmp.position = pos0;
	tmp.color = color;
	tmp.uv = glm::vec2(uv_x0, uv_y0);
	vertices.push_back(tmp);


	/// 1. bot right
	tmp.position = pos1;
	tmp.color = color;
	tmp.uv = glm::vec2(uv_x1, uv_y0);
	vertices.push_back(tmp);

	/// 2. top right
	tmp.position = pos2;
	tmp.color = color;
	tmp.uv = glm::vec2(uv_x1, uv_y1);
	vertices.push_back(tmp);

	/// 3. top left
	tmp.position = pos3;
	tmp.color = color;
	tmp.uv = glm::vec2(uv_x0, uv_y1);
	vertices.push_back(tmp);

	indices.push_back(indicesStart + 1);
	indices.push_back(indicesStart + 2);
	indices.push_back(indicesStart + 0);

	indices.push_back(indicesStart + 2);
	indices.push_back(indicesStart + 3);
	indices.push_back(indicesStart + 0);
}

void ModelManager::buildQuadModel(Model* model, glm::vec3 min, glm::vec3 max)
{
    std::vector<VertexData> vertices;
    std::vector<unsigned int> indices;

    glm::vec3 color = glm::vec3(0, 0, 0);
    buildQuad3D(min, max, color, vertices, indices);

    Mesh m(vertices, indices);
    model->addMesh(m);
}

void ModelManager::buildQuadOutlineModel(Model* model, glm::vec3 min, glm::vec3 max, float thickness)
{
    std::vector<VertexData> vertices;
    std::vector<unsigned int> indices;

    // top
    glm::vec2 min0 = glm::vec2(min.x, max.y - thickness);
    glm::vec2 max0 = glm::vec2(max.x, max.y);
    glm::vec3 color = glm::vec3(0, 0, 0);
    buildQuad2D(min0, max0, color, vertices, indices);

    // right
    min0 = glm::vec2(max.x - thickness, min.y);
    max0 = glm::vec2(max.x, max.y);
    color = glm::vec3(0, 0, 0);
    buildQuad2D(min0, max0, color, vertices, indices);

    // bottom
    min0 = glm::vec2(min.x, min.y);
    max0 = glm::vec2(max.x, min.y + thickness);
    color = glm::vec3(0, 0, 0);
    buildQuad2D(min0, max0, color, vertices, indices);

    // left
    min0 = glm::vec2(min.x, min.y);
    max0 = glm::vec2(min.x + thickness, max.y);
    color = glm::vec3(0, 0, 0);
    buildQuad2D(min0, max0, color, vertices, indices);

    Mesh m(vertices, indices);
    model->addMesh(m);
}


void ModelManager::buildCubeModel(Model* model, glm::vec3 maxP, glm::vec3 minP)
{
    vector<unsigned int> indices;
    vector<VertexData> vertices;
    VertexData tmp;

    tmp.position = glm::vec3(minP.x, maxP.y, maxP.z); tmp.color = glm::vec3(1.0, 0.0, 0.0);
    vertices.push_back(tmp);

    tmp.position = glm::vec3(minP.x, minP.y, maxP.z);	tmp.color = glm::vec3(0.0, 1.0, 0.0);
    vertices.push_back(tmp);

    tmp.position = glm::vec3(maxP.x, minP.y, maxP.z);	tmp.color = glm::vec3(0.0, 0.0, 1.0);
    vertices.push_back(tmp);

    tmp.position = glm::vec3(maxP.x, maxP.y, maxP.z);	tmp.color = glm::vec3(1.0, 1.0, 0.0);
    vertices.push_back(tmp);

    tmp.position = glm::vec3(minP.x, maxP.y, minP.z); tmp.color = glm::vec3(1.0, 0.0, 1.0);
    vertices.push_back(tmp);

    tmp.position = glm::vec3(minP.x, minP.y, minP.z);	tmp.color = glm::vec3(1.0, 0.0, 0.0);
    vertices.push_back(tmp);

    tmp.position = glm::vec3(maxP.x, minP.y, minP.z);	tmp.color = glm::vec3(1.0, 1.0, 1.0);
    vertices.push_back(tmp);

    tmp.position = glm::vec3(maxP.x, maxP.y, minP.z);	tmp.color = glm::vec3(0.0, 1.0, 1.0);
    vertices.push_back(tmp);

    //front face
    indices.push_back(0);   indices.push_back(2);	indices.push_back(1);
    indices.push_back(0);  	indices.push_back(3);   indices.push_back(2);

    //left face
    indices.push_back(2);	indices.push_back(3);	indices.push_back(6);
    indices.push_back(3);	indices.push_back(7);	indices.push_back(6);

    //back face
    indices.push_back(4);	indices.push_back(5);	indices.push_back(6);
    indices.push_back(4);	indices.push_back(7);	indices.push_back(6);

    //right face
    indices.push_back(0);	indices.push_back(1);	indices.push_back(5);
    indices.push_back(0);	indices.push_back(4);	indices.push_back(5);

    //top face
    indices.push_back(0);	indices.push_back(3);	indices.push_back(4);
    indices.push_back(3);	indices.push_back(4);	indices.push_back(7);

    //bottom face
    indices.push_back(1);	indices.push_back(2);	indices.push_back(6);
    indices.push_back(1);	indices.push_back(5);	indices.push_back(6);

    Mesh m(vertices, indices);
    model->addMesh(m);
}


void ModelManager::buildXYZModel(Model* model)
{
    model->setModelGemoetry(GL_LINES);

    std::vector<VertexData> vertices;
    std::vector<unsigned int> indices;
    VertexData v;

    float scale = 1;

    /// X axis
    v.position = glm::vec3(0.0, 0.0, 0.0);
    v.color = glm::vec3(1.0, 0.0, 0.0);
    vertices.push_back(v);

    v.position = glm::vec3(scale, 0.0, 0.0);
    v.color = glm::vec3(1.0, 0.0, 0.0);
    vertices.push_back(v);

    /// Y axis
    v.position = glm::vec3(0.0, 0.0, 0.0);
    v.color = glm::vec3(0.0, 1.0, 0.0);
    vertices.push_back(v);;

    v.position = glm::vec3(0.0, scale, 0.0);
    v.color = glm::vec3(0.0, 1.0, 0.0);
    vertices.push_back(v);;

    /// Z axis
    v.position = glm::vec3(0.0, 0.0, 0.0);
    v.color = glm::vec3(0.0, 0.0, 1.0);
    vertices.push_back(v);;

    v.position = glm::vec3(0.0, 0.0, scale);
    v.color = glm::vec3(0.0, 0.0, 1.0);
    vertices.push_back(v);;


    indices.push_back(0);
    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(3);
    indices.push_back(4);
    indices.push_back(5);

    Mesh m(vertices, indices);
    model->addMesh(m);
}

void ModelManager::buildCircleOutlineModel(Model* model, float radius, float thickness)
{
    std::vector<VertexData> vertices;
    std::vector<unsigned int> indices;

    for (float i = 0; i < 360; i += 0.5f)
    {
        //phys
        float cos1 = cos(i * utl::DEGREE_TO_RADIAN);
        float sin1 = sin(i * utl::DEGREE_TO_RADIAN);

        float cos2 = cos((i + 1) * utl::DEGREE_TO_RADIAN);
        float sin2 = sin((i + 1) * utl::DEGREE_TO_RADIAN);

        float wx = radius * cos1;
        float wy = radius * sin1;
        glm::vec2 simPos0 = glm::vec2(wx, wy);

        wx = radius * cos2;
        wy = radius * sin2;
        glm::vec2 simPos1 = glm::vec2(wx, wy);

        buildLinesBetweenTwoPoints2D(simPos0, simPos1, thickness, vertices, indices);
    }

    Mesh m(vertices, indices);
    model->addMesh(m);
}

void ModelManager::buildArrowModel(Model* model)
{
    model->setModelGemoetry(GL_LINES);

    std::vector<VertexData> vertices;
    std::vector<unsigned int> indices;
    VertexData v;

    float scale = 1;

    /// X axis
    v.position = glm::vec3(0.0, 0.0, 0.0);
    vertices.push_back(v);

    v.position = glm::vec3(scale, 0.0, 0.0);
    vertices.push_back(v);

    /// Y axis
    v.position = glm::vec3(1.0, 0.0, 0.0);
    vertices.push_back(v);;

    v.position = glm::vec3(0.90, 0.15, 0.0);
    vertices.push_back(v);;

    /// Z axis
    v.position = glm::vec3(1.0, 0.0, 0.0);
    vertices.push_back(v);;

    v.position = glm::vec3(0.90, -0.15, 0.0);
    vertices.push_back(v);;

    indices.push_back(0);
    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(3);
    indices.push_back(4);
    indices.push_back(5);

    Mesh m(vertices, indices);
    model->addMesh(m);
}

void ModelManager::shutDown()
{

}

Model* ModelManager::get(int modelEnum)
{
	return &m_models[modelEnum];
}

// http://strike-counter.com/cs-go-stats/weapons-stats



/*
QuadModel::QuadModel(float l, float r,
    float b, float t,
    float uv_x, float uv_y, float uv_w)
{
    init(l, r,
        b, t,
        uv_x, uv_y, uv_w);
}

void QuadModel::init(float l, float r,
    float b, float t,
    float uv_x, float uv_y, float uv_w)
{
    std::vector<VertexData> vertices;
    std::vector<unsigned int> indices;
    VertexData tmp;

    float uv_x0 = uv_x;  float uv_x1 = uv_x + 1;
    float uv_y0 = uv_y;  float uv_y1 = uv_y + 1;


    /// 0. bot left
    tmp.position = glm::vec3(l, b, 0.0);
    tmp.uv = glm::vec2(uv_x0 * uv_w, uv_y0 * uv_w);
    vertices.push_back(tmp);
    /// 1. bot right
    tmp.position = glm::vec3(r, b, 0.0);
    tmp.uv = glm::vec2(uv_x1 * uv_w, uv_y0 * uv_w);
    vertices.push_back(tmp);

    /// 2. top right
    tmp.position = glm::vec3(r, t, 0.0);
    tmp.uv = glm::vec2(uv_x1 * uv_w, uv_y1 * uv_w);
    vertices.push_back(tmp);
    /// 3. top left
    tmp.position = glm::vec3(l, t, 0.0);
    tmp.uv = glm::vec2(uv_x0 * uv_w, uv_y1 * uv_w);
    vertices.push_back(tmp);

    initIndices(indices);

    Mesh m(vertices, indices);
    m_meshes.push_back(m);
}
*/


/*
CubeModel::CubeModel(glm::vec3 maxP, glm::vec3 minP, glm::vec3 color)
{
    vector<unsigned int> indices;
    vector<VertexData> vertices;
    VertexData tmp;

    tmp.color = color;
    tmp.position = glm::vec3(minP.x, maxP.y, maxP.z);
    vertices.push_back(tmp);

    tmp.position = glm::vec3(minP.x, minP.y, maxP.z);
    vertices.push_back(tmp);

    tmp.position = glm::vec3(maxP.x, minP.y, maxP.z);
    vertices.push_back(tmp);

    tmp.position = glm::vec3(maxP.x, maxP.y, maxP.z);
    vertices.push_back(tmp);

    tmp.position = glm::vec3(minP.x, maxP.y, minP.z);
    vertices.push_back(tmp);

    tmp.position = glm::vec3(minP.x, minP.y, minP.z);
    vertices.push_back(tmp);

    tmp.position = glm::vec3(maxP.x, minP.y, minP.z);
    vertices.push_back(tmp);

    tmp.position = glm::vec3(maxP.x, maxP.y, minP.z);
    vertices.push_back(tmp);

    //front face
    indices.push_back(0);   indices.push_back(2);	indices.push_back(1);
    indices.push_back(0);  	indices.push_back(3);   indices.push_back(2);

    //left face
    indices.push_back(2);	indices.push_back(3);	indices.push_back(6);
    indices.push_back(3);	indices.push_back(7);	indices.push_back(6);

    //back face
    indices.push_back(4);	indices.push_back(5);	indices.push_back(6);
    indices.push_back(4);	indices.push_back(7);	indices.push_back(6);

    //right face
    indices.push_back(0);	indices.push_back(1);	indices.push_back(5);
    indices.push_back(0);	indices.push_back(4);	indices.push_back(5);

    //top face
    indices.push_back(0);	indices.push_back(3);	indices.push_back(4);
    indices.push_back(3);	indices.push_back(4);	indices.push_back(7);

    //bottom face
    indices.push_back(1);	indices.push_back(2);	indices.push_back(6);
    indices.push_back(1);	indices.push_back(5);	indices.push_back(6);

    Mesh m(vertices, indices);
    m_meshes.push_back(m);
}
*/
