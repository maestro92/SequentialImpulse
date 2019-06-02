#pragma once

#include "pipeline.h"
#include "transform_component.h"

const float CAMERA_POS_DELTA = 1;
const float CAMERA_ZOOM_DELTA = 1;

class Camera
{
	public:
		Camera();	
		void setPos(glm::vec3 pos);
		void setZoom(float zoom);		
		void updateOrtho();
		
		void forceSetOrtho(float left, float right, float bottom, float top, float nearIn, float farIn);

		void setPanningBounds(glm::vec3 worldMapMin, glm::vec3 worldMapMax);


		void zoomOut();
		void zoomIn();
		glm::vec3 screenToWorldPoint(glm::vec2 screenPoint);
		glm::vec3 worldToScreen(glm::vec3 pos);


		TransformComponent transform;

		Pipeline& getPipeline();

        Pipeline pipeline;
	private:
		void updateBounds();

		glm::vec3 m_curMinBound;
		glm::vec3 m_curMaxBound;
		glm::vec3 m_minBound;
		glm::vec3 m_maxBound;
		float m_cameraZoom;


};