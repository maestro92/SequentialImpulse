#include "entity.h"
#include "model_manager.h"



void Entity::init()
{
    m_model = NULL;
}

/*
void Entity::renderCore(Pipeline& p, Renderer* r)
{
    p.pushMatrix();
        p.translate(physBody.position);
        p.addMatrix(physBody.orientationMat);
        p.scale(physBody.scale);
        r->setUniLocs(p);
        m_model->render();
    p.popMatrix();
}
*/

bool Entity::canRender()
{
	return m_model != NULL;
}


bool Entity::shouldRender()
{
	return true;
}

