#ifndef MODEL_MANAGER_H_
#define MODEL_MANAGER_H_

using namespace std;

#include <unordered_map>
#include "model_enum.h"

#include "utility.h"

#include "model_constant.h"
#include "model.h"
#include "quad_model.h"


#include "json_spirit.h"
#include "json_spirit_reader_template.h"
#include "json_spirit_writer_template.h"
#include <cassert>
#include <fstream>

#ifndef JSON_SPIRIT_MVALUE_ENABLED
#error Please define JSON_SPIRIT_MVALUE_ENABLED for the mValue type to be enabled 
#endif

using namespace std;
using namespace json_spirit;

// to resolve circular depenency issue
// class Weapon;








class ModelManager
{


	public:

		ModelManager();
		~ModelManager();

		void init();
		void shutDown();

		Model* get(int modelEnum);
 

		static void enableVertexAttribArrays();
		static void disableVertexAttribArrays();

		// min and max are in world position
		static void buildQuad2D(glm::vec2 min, glm::vec2 max, glm::vec3 color,
			vector<VertexData>& vertices,
			vector<unsigned int>& indices);

		static void buildQuad3D(glm::vec3 min, glm::vec3 max, glm::vec3 color,
			vector<VertexData>& vertices,
			vector<unsigned int>& indices);

		static void buildLinesBetweenTwoPoints2D(glm::vec2 p0, glm::vec2 p1, float thickness,
			vector<VertexData>& vertices,
			vector<unsigned int>& indices);

		static void buildLinesBetweenTwoPoints3D(glm::vec3 p0, glm::vec3 p1, float thickness,
			vector<VertexData>& vertices,
			vector<unsigned int>& indices);

// change all these to just data
	private:

        Model* buildXYZModel();

		Model* m_quad;
		Model* m_centeredQuad;
        Model* m_unitCenteredQuad;
        Model* m_xyzAxis;

		vector<Model*> m_models;
};



#endif