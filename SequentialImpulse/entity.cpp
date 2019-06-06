#include "entity.h"
#include "model_manager.h"

Entity::Entity()
{
	active = true;
	// m_instanceId = utl::createUniqueObjectID();

    position = glm::vec3(0.0, 0.0, 0.0);
    velocity = glm::vec3(0.0, 0.0, 0.0);
    scale = glm::vec3(1.0, 1.0, 1.0);
    orientation = glm::mat4(1.0);
	
	m_model = NULL;
}


Entity* Entity::getOne()
{
	Entity* obj = new Entity();
	return obj;
}




void Entity::renderCore(Pipeline& p, Renderer* r)
{
	p.pushMatrix();
		p.translate(position);
		p.addMatrix(orientation);
		p.scale(scale);
		r->setUniLocs(p);
		m_model->render();
	p.popMatrix();
}


bool Entity::canRender()
{
	return m_model != NULL;
}


bool Entity::shouldRender()
{
	return true;
}

