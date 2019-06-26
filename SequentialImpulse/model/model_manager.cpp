#include "model_manager.h"

/*

when working with models in blender
the -z axis in game is the y axis in blender, 

so to be sure to orient the models in the right direction


*/


namespace Asset
{
    void buildLinesBetweenTwoPoints2D(glm::vec3 p0, glm::vec3 p1, float thickness,
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

    void buildLinesBetweenTwoPoints2D(glm::vec2 p0, glm::vec2 p1, float thickness,
        vector<VertexData>& vertices,
        vector<unsigned int>& indices)
    {
        glm::vec3 pos0 = glm::vec3(p0.x, p0.y, 0);
        glm::vec3 pos1 = glm::vec3(p1.x, p1.y, 0);
        buildLinesBetweenTwoPoints2D(pos0, pos1, thickness, vertices, indices);
    }



    void buildCircle(float radius, float thickness, vector<VertexData>& vertices, vector<unsigned int>& indices)
    {        
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
    }
};

ModelManager::ModelManager()
{

}

ModelManager::~ModelManager()
{

}

void ModelManager::init()
{
	m_quad = new QuadModel(1, 1);
	m_centeredQuad = new QuadModel(-0.5, 0.5, -0.5, 0.5);
    m_unitCenteredQuad = new QuadModel(-1, 1, -1, 1);
    m_xyzAxis = buildXYZModel(); 
    
    std::vector<VertexData> vertices;
    std::vector<unsigned int> indices;
    Asset::buildCircle(1, 0.05, vertices, indices);
    Mesh m = Mesh(vertices, indices);   
    Model* circleModel = new Model();
    circleModel->addMesh(m);
    

	m_models.resize(ModelEnum::NUM_MODELS);
	m_models[ModelEnum::quad] = m_quad;
    m_models[ModelEnum::centeredQuad] = m_centeredQuad;
	m_models[ModelEnum::unitCenteredQuad] = m_unitCenteredQuad;
	m_models[ModelEnum::xyzAxis] = m_xyzAxis;
    m_models[ModelEnum::circle] = circleModel;
    m_models[ModelEnum::arrow] = buildArrowModel();

}


Model* ModelManager::buildXYZModel()
{
    Model* model = new Model();

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

    //   worldAxis = new mesh(&axisVertices, &axisIndices);
    Mesh m(vertices, indices);
    model->addMesh(m);

    return model;
}


Model* ModelManager::buildArrowModel()
{
    Model* model = new Model();

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

    //   worldAxis = new mesh(&axisVertices, &axisIndices);
    Mesh m(vertices, indices);
    model->addMesh(m);

    return model;
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




void ModelManager::shutDown()
{


}

Model* ModelManager::get(int modelEnum)
{
	return m_models[modelEnum];
}

// http://strike-counter.com/cs-go-stats/weapons-stats
