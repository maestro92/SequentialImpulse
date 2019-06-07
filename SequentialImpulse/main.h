#ifndef MAIN_H_
#define MAIN_H_


#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <string>
#include <sstream>
#include <unordered_set>
#include "SDL.h"
#include "SDL_image.h"
#include "define.h"
#include "utility.h"


#include <queue>

#include <GL/glew.h>

#include "utility.h"
#include "shader.h"

#include "label.h"
#include "gui_manager.h"
#include <chrono>

#include "pipeline.h"
#include "camera.h"
#include <ft2build.h>

#include "renderer_manager.h"
#include "renderer.h"
#include "renderer_constants.h"

#include "quad_model.h"
#include "xyz_axis_model.h"
#include "entity.h"
#include "model_manager.h"

#define FRAME_VALUES 10
#include <list>

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include "game.h"
#include <vector>

using namespace std;
/*

For style
use http://google-styleguide.googlecode.com/svn/trunk/cppguide.html#Function_Names

C++ style
http://geosoft.no/development/cppstyle.html

http://stackoverflow.com/questions/1228161/why-use-prefixes-on-member-variables-in-c-classes

I use:

m for members
c for constants/readonlys
p for pointer (and pp for pointer to pointer)
v for volatile
s for static
i for indexes and iterators
e for events
*/





const int TIME_PROFILER_BUFFER = 10;
const int FPS_PROFILER_BUFFER = 20;




class FogOfWar
{
	public:
		Renderer*					p_renderer;
		
		/// GUI
		long long m_runningTime;
		Uint32 m_nextGameTick = 0;
		MouseState m_mouseState;


		float m_fps;
		float m_iterRefreshRate;
		float m_curIter;
		unsigned int m_frameCount;
		unsigned int m_frameTicks[FRAME_VALUES];
		unsigned int m_frameTicksIndex;
		unsigned int m_prevFrameTick;

		bool isRunning;


		// models
		Model*          p_model;


		bool containedFlag;


		GUIManager m_gui;
		bool loadData;
		int frameNum;
		float m_zoom;
		float m_range;
	public:

		long long m_currentTimeMillis;

		int timeProfilerIndex;
		long long timeProfiler[TIME_PROFILER_BUFFER];

		int fpsProfilerIndex;
		int fpsProfiler[FPS_PROFILER_BUFFER];

		void initPlayer();
//		void debugDrawing();
     //   void debugDrawing(GameState* gameState);


		/// init functions
		void init();

		void initGUI();

		Entity* constructLine(glm::vec2 p0, glm::vec2 p1, float width) const;
		void UpdatingCurrentRayNewEndPoint(glm::vec2 end);

		int endWithError(char* msg, int error = 0);
 

		void start();
		void update(GameState* gameState);

		int getAverageFPS();


		void render(GameState* gameState);
        void onMouseBtnUp(GameState* gameState);
        void onMouseBtnHold(GameState* gameState);
        void onMouseBtnDown(GameState* gameState);



		void GetTimeProfilerAverages();


		void renderGUI();


		long long getCurrentTimeMillis();

	private:
		long long startedTime;
};

#endif