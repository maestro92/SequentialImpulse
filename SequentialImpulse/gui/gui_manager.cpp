#include "gui_manager.h"


void GUIManager::init(int screenWidth, int screenHeight)
{
	m_fpsLabel = NULL;
    m_GUIComponentsFlags = 0;

    m_screenWidth = screenWidth;
    m_screenHeight = screenHeight;

	uiCamera.forceSetOrtho(0, m_screenWidth, m_screenHeight, 0, -1, 1);


	uiCamera.getPipeline().setMatrixMode(MODEL_MATRIX);
	uiCamera.getPipeline().loadIdentity();

   // m_textureQuad = QuadModel(1,1);


    ModelManager::buildQuadModel(&m_textureQuad, glm::vec3(0, 0, 0), glm::vec3(1, 1, 1));

	char* filename = "gui/gui_renderer_data.json";

	Value vContent = utl::readJsonFileToVector(filename);
	const Array& vArray = vContent.get_array();

	string path = "gui_shaders/";

	Renderer::initRendererWrapper(vArray, &r_texture, "r_texture", path);
	Renderer::initRendererWrapper(vArray, &r_depthTexture, "r_depthTexture", path);
	Renderer::initRendererWrapper(vArray, &r_coloredRect, "r_coloredRect", path);
	Renderer::initRendererWrapper(vArray, &r_texturedRect, "r_texturedRect", path);
	Renderer::initRendererWrapper(vArray, &r_listBoxItemHighlight, "r_listBoxItemHighlight", path);
	Renderer::initRendererWrapper(vArray, &r_text, "r_text", path);

	Control::r_coloredRect = r_coloredRect;
	Control::r_texturedRect = r_texturedRect;
	Control::r_listBoxItemHighlight = r_listBoxItemHighlight;
	Control::m_textEngine.r_textRenderer = r_text;


	int xOffset = 55;
	int yOffset = 570;

	int BAR_WIDTH = 60;
	int BAR_HEIGHT = 10;

	xOffset = 0; yOffset = 0;
	m_fpsLabel = new Label("90", xOffset, yOffset, 50, 50, COLOR_GRAY);

	float width = 50;
	float height = 50;
    xOffset = 0; yOffset = 50;
	m_debugLabel = new Label("fdasdf", xOffset, yOffset, width, height, COLOR_GRAY);
    m_debugLabel->setFontSize(15);
	addGUIComponent(m_fpsLabel);
	addGUIComponent(m_debugLabel);


	utl::debug("GUI manager initing");
}



glm::vec3 GUIManager::screenToUISpace(glm::vec2 screenPoint)
{
	glm::vec4 viewPort = glm::vec4(0, 0, utl::SCREEN_WIDTH, utl::SCREEN_HEIGHT);
	glm::vec3 temp = glm::vec3(screenPoint.x, screenPoint.y, 0);

	glm::vec3 worldPoint = glm::unProject(temp, (uiCamera.getPipeline().getModelViewMatrix()), uiCamera.getPipeline().getProjectionMatrix(), viewPort);

	return worldPoint;
}



void GUIManager::setFPS(int fps)
{
    if (m_fpsLabel != NULL)
    {
        m_fpsLabel->setText(utl::intToStr(fps));
    }
}

/*
void GUIManager::update(GameState* gameState)
{
    if (m_debugLabel != NULL)
    {
        int numBodies = 0;
        for (int i = 0; i < gameState->numEntities; i++)
        {
            if (!gameState->entities[i].isDead)
            {
                numBodies++;
            }
        }


        int numJoints = 0;
        for (int i = 0; i < gameState->numJoints; i++)
        {
            if (!gameState->joints[i].isDead)
            {
                numJoints++;
            }
        }


        int numContacts = 0;
        for (int i = 0; i < gameState->numContacts; i++)
        {
            numContacts++;
        }
        string s = "bodies/contacts/joints ";
        m_debugLabel->setText(s + utl::intToStr(numBodies) + "/" + utl::intToStr(numJoints) + "/" + utl::intToStr(numContacts));
    }
}
*/
void GUIManager::update(int numBodies, int numJoints, int numContacts)
{
    string s = "bodies/joints/contacts ";
    m_debugLabel->setText(s + utl::intToStr(numBodies) + "/" + utl::intToStr(numJoints) + "/" + utl::intToStr(numContacts));
}


void GUIManager::initGUIRenderingSetup()
{
	setupRenderToScreen(0, 0, m_screenWidth, m_screenHeight);
}

void GUIManager::setupRenderToScreen(int x, int y, int width, int height)
{
	glViewport(x, y, width, height);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
}

void GUIManager::renderTextureFullScreen(GLuint textureId)
{
	renderTextureFullScreen(textureId, RENDER_TO_SCREEN);
}

void GUIManager::renderTextureFullScreen(GLuint textureId, GLuint fboTarget)
{
	renderTexture(textureId, fboTarget, 0, 0, m_screenWidth, m_screenHeight, r_texture);
}

void GUIManager::renderDepthTextureFullScreen(GLuint textureId)
{
	renderDepthTextureFullScreen(textureId, RENDER_TO_SCREEN);
}

void GUIManager::renderDepthTextureFullScreen(GLuint textureId, GLuint fboTarget)
{
	renderTexture(textureId, fboTarget, 0, 0, m_screenWidth, m_screenHeight, r_depthTexture);
}

void GUIManager::renderTexture(GLuint textureId, int x, int y, int width, int height, Renderer& r)
{
	renderTexture(textureId, RENDER_TO_SCREEN, x, y, width, height, r);
}

void GUIManager::renderTexture(GLuint textureId, GLuint fboTarget, int x, int y, int width, int height, Renderer& r)
{
	setupRenderToScreen(x, y, width, height);
	r.enableShader();
	r.setData(R_TEXTURE::u_texture, 0, GL_TEXTURE_2D, textureId);

	uiCamera.getPipeline().pushMatrix();
		uiCamera.getPipeline().translate(x, y, 0);
		uiCamera.getPipeline().scale(width, height, 1.0);

		r.setUniLocs(uiCamera.getPipeline());
        m_textureQuad.render();
	uiCamera.getPipeline().popMatrix();
	r.disableShader();
}


void GUIManager::renderTexture(GLuint TextureId, GLuint FboTarget, Rect rect)
{
    renderTexture(TextureId, FboTarget, rect.x, rect.y, rect.w, rect.h, r_texture);
}



void GUIManager::addGUIComponent(Control* control)
{
	control->setID(m_GUIComponentsID);
	m_GUIComponents.push_back(control);
}

void GUIManager::addDebugLabel(Control* control)
{
	control->setID(m_GUIComponentsID);
	m_debugLabels.push_back(control);
}

void GUIManager::removeDebugLabels()
{
	for (int i = 0; i < m_debugLabels.size(); i++)
	{
		delete m_debugLabels[i];
	}
	m_debugLabels.clear();
}

int GUIManager::getNumGUIComponent()
{
	return m_GUIComponents.size();
}


void GUIManager::updateAndRender(GameInput& mouseState)
{
    for(int i=0; i<m_GUIComponents.size(); i++)
    {
		Control* control = m_GUIComponents[i];
		control->update(mouseState);
		control->render();
	}

	for (int i = 0; i<m_debugLabels.size(); i++)
	{
		Control* control = m_debugLabels[i];
		control->update(mouseState);
		control->render();
	}
}
