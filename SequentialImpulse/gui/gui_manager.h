#ifndef GUI_MANAGER_H_
#define GUI_MANAGER_H_

#include "pipeline.h"
#include "utility.h"
#include "utility_math.h"
#include "Rect.h"
#include "Slider.h"
#include "list_box.h"
#include "button.h"
#include "control.h"
#include "renderer.h"
#include "camera.h"

#include "gui_renderer_constants.h"
#include "custom_gui/bar.h"
#include "renderer_constants.h"


class GUIManager
{
    public:
        void init(int screenWidth, int screenHeight);

		void initGUIRenderingSetup();
		void setupRenderToScreen(int x, int y, int width, int height);
		void renderTextureFullScreen(GLuint textureId);
		void renderTextureFullScreen(GLuint textureId, GLuint fboTarget);
		void renderDepthTextureFullScreen(GLuint textureId);
		void renderDepthTextureFullScreen(GLuint textureId, GLuint fboTarget);
		void renderTexture(GLuint textureId, int x, int y, int width, int height, Renderer& r);
		void renderTexture(GLuint textureId, GLuint fboTarget, int x, int y, int width, int height, Renderer& r);
		void renderTexture(GLuint textureId, GLuint fboTarget, Rect rect);

		Renderer r_texture;
		Renderer r_depthTexture;
		Renderer r_coloredRect;
		Renderer r_texturedRect;
		Renderer r_listBoxItemHighlight;
		Renderer r_text;


        void updateAndRender(GameInput& mouseState);
        void addGUIComponent(Control* control);
		int getNumGUIComponent();

        void setFPS(int fps);
        void update(int numBodies, int numJoints, int numContacts);

		void addDebugLabel(Control* control);
		void removeDebugLabels();

		Pipeline& getPipeline();
		glm::vec3 screenToUISpace(glm::vec2 screenPoint);


		Camera uiCamera;

	private:

        int m_GUIComponentsID;
        int m_GUIComponentsFlags;

        Label* m_fpsLabel;
        Label* m_debugLabel;

        vector<Control*> m_GUIComponents;
		vector<Control*> m_debugLabels;


        Model* guiModel;


        Model m_textureQuad;
        int m_screenWidth;
        int m_screenHeight;
};

#endif
