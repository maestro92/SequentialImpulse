#include "main.h"


#include <stdio.h>
#include <string.h>
#include "game.h"
#include "game_rendering.h"
#include "global.h"

#define RENDER_DEBUG_FLAG 0





#define PI 3.14159265

// the server simluates the game in descirete time steps called ticks


// http://stackoverflow.com/questions/4845410/error-lnk2019-unresolved-external-symbol-main-referenced-in-function-tmainc
#undef main


using namespace std;
//using namespace std::placeholders;
// https://www.youtube.com/watch?v=tlXM8qDOS3U
// Screen dimension constants


// frame rate
// https://sites.google.com/site/sdlgamer/intemediate/lesson-7
// FPS is 50
// So Interval is 1000/50 which is 20ms
// meaning my while loop runs 50 frames per second
// which is every other 20 ms
const int FRAMES_PER_SECOND = 60;
const float FIXED_UPATE_TIME_s = 1 / (float)FRAMES_PER_SECOND;


//display surface
SDL_Surface* pDisplaySurface = NULL;
//event structure
SDL_Event event;


const float GAME_SPEED = 1.0f;
const float _FIXED_UPDATE_TIME_s = 0.01667f;
const float FIXED_UPDATE_TIME_s = _FIXED_UPDATE_TIME_s / GAME_SPEED;
const float FIXED_UPDATE_TIME_ms = FIXED_UPDATE_TIME_s * 1000;

const float MOUSE_DIST_THRESHOLD = 0.05;

/*
const int SV_FRAMES_PER_SECOND = 20;
const float SV_FIXED_UPATE_TIME_s = 1 / SV_FRAMES_PER_SECOND;
const long long SV_FIXED_UPATE_TIME_ms = 1000 / SV_FRAMES_PER_SECOND;
*/


const float SPAWN_POSITION_UNIT_OFFSET = 40.0f;

const int INVALID_OBJECT = 0x7FFFFFFF;

// link
// http://lodev.org/cgtutor/raycasting.html



void FogOfWar::init()
{
	frameNum = 0;

	global.modelMgr = new ModelManager();
	global.modelMgr->init();

	// renderMgr has to init after the lightMgr
	global.rendererMgr = new RendererManager();
	global.rendererMgr->init(utl::SCREEN_WIDTH, utl::SCREEN_HEIGHT);


	isRunning = true;

	containedFlag = false;

	timeProfilerIndex = 0;
	fpsProfilerIndex = 0;


	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	// initGUI depends on the m_defaultPlayerID, so initNetworkLobby needs to run first
	initGUI();

	for (int i = 0; i < FPS_PROFILER_BUFFER; i++)
	{
		fpsProfiler[i] = 0;
	}


	//Initialize clear color
	glClearColor(0.0f, 0.5f, 0.0f, 1.0f);

/*
	mainCamera.setPos(glm::vec3(5, 5, 0));
	mainCamera.setZoom(60);
	mainCamera.updateOrtho();
	*/
	


	glCullFace(GL_BACK);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);

	SDL_WM_SetCaption("MatchThree", NULL);


	loadData = false;
	bool runningTests = false;



	if (runningTests == true)
	{

	}
	else
	{
        /*
		if (loadData)
		{
			map.saveLatest = false;
			map.load("board1.txt");
			fogManager.init(map.getWidth(), map.getHeight());
			mapView.init(&world, &map);
			debugDrawing();
		}
		else
        */
		{

		//	debugDrawing();
		}
	}




	ModelManager::enableVertexAttribArrays();

}

/*
void onRightMouseBtnDown(GameState* gameState)
{
    int tmpx, tmpy;
    SDL_GetMouseState(&tmpx, &tmpy);
    tmpy = utl::SCREEN_HEIGHT - tmpy;

    glm::vec2 screenPoint = glm::vec2(tmpx, tmpy);
    glm::vec3 worldPoint = gameState->mainCamera.screenToWorldPoint(screenPoint);
    glm::vec2 tempWorldPoint = glm::vec2(worldPoint.x, worldPoint.y);
}



GLuint tempTexture;


void FogOfWar::debugDrawing(GameState* gameState)
{
	m_gui.removeDebugLabels();
	float size = 15;
	//	float size = m_cameraZoom * 0.8;
	//	float size = 300 / m_cameraZoom;

	
	for (int y = 0; y < map.getHeight(); y+=5)
	{
		for (int x = 0; x < map.getWidth(); x+=5)
		{
			glm::vec2 temp = map.getCellCenter(glm::ivec2(x, y));
			glm::vec3 pos = glm::vec3(temp.x, temp.y, 0);
			glm::vec3 screenPos = gameState->mainCamera.worldToScreen(glm::vec3(pos.x, pos.y, 0));
			glm::vec3 labelPos = m_gui.screenToUISpace(glm::vec2(screenPos.x, screenPos.y));

			string s = utl::intToStr(x) + " " + utl::intToStr(y);

			Label* coordLabel = new Label(s, labelPos.x + 20, labelPos.y, 0, 0, COLOR_WHITE);
			coordLabel->setFont(size, COLOR_BLACK);
			m_gui.addDebugLabel(coordLabel);
		}
	}
	
}
*/



void FogOfWar::GetTimeProfilerAverages()
{
	long long total = 0;
	for (int i = 0; i < TIME_PROFILER_BUFFER; i++)
	{
		total += timeProfiler[i];
	}
	cout << "average is " << total / TIME_PROFILER_BUFFER << endl;
}


void FogOfWar::start()
{
	cout << "Start" << endl;

	long long dt = 0;
	long long oldTime = utl::getCurrentTime_ms(); 
	long long newTime = 0;
	
    GameState gameState;
    GameCode::init(&gameState);


    gameState.mainCamera.setPanningBounds(glm::vec3(-50,-50,-50), glm::vec3(50, 50, 50));
    gameState.mainCamera.setPos(glm::vec3(0, 10, 0));
    gameState.mainCamera.setZoom(50);



	Uint32 time0 = 0;
	Uint32 time1 = 0;
	startedTime = utl::getCurrentTime_ms();

    GameInput gameInput = {};
    gameInput.dt_s = FIXED_UPATE_TIME_s;
    
	while (isRunning)
	{
		time0 = SDL_GetTicks();

		newTime = utl::getCurrentTime_ms();

		dt = newTime - oldTime;
		update(&gameState);


        GameCode::tick(gameInput, &gameState);



        render(&gameState);


		oldTime = newTime;
		
		time1 = SDL_GetTicks();
		
		
		// cout << fpsProfilerIndex << endl;
		if (fpsProfilerIndex == FPS_PROFILER_BUFFER)
		{
			fpsProfilerIndex = 0;
		}
		fpsProfiler[fpsProfilerIndex] = (int)(time1 - time0);
		++fpsProfilerIndex;
		
		int fps = getAverageFPS();
		// cout << fps << endl;
		
		/*
		++fpsProfilerIndex;
		if (fpsProfilerIndex > 1000)
		{
			fpsProfilerIndex = 0;
		}
		*/
		
		// fpsProfilerIndex = 1;
		m_gui.setFPS(fps);
	}
}



int FogOfWar::getAverageFPS()
{
	float averageFrameTime = 0;
	for (int i = 0; i < FPS_PROFILER_BUFFER; i++)
	{
		averageFrameTime += fpsProfiler[i];
	}

	if (averageFrameTime == 0)
	{
		return 0;
	}
	else
	{
		averageFrameTime = averageFrameTime / FPS_PROFILER_BUFFER;

		int fps = 1000 / averageFrameTime;

	//	cout << averageFrameTime << " " << fps << endl;
		return fps;
	}
}

/*
void FogOfWar::updateCamera()
{
	m_pipeline.ortho(m_cameraCenter.x - m_cameraZoom, 
					m_cameraCenter.x + m_cameraZoom,
					m_cameraCenter.y - m_cameraZoom,
					m_cameraCenter.y + m_cameraZoom, utl::Z_NEAR, utl::Z_FAR);

//	debugDrawing(curDrawing);
}
*/


// VBO with dynamically changing number of points
// https://www.opengl.org/discussion_boards/showthread.php/178828-VBO-with-dynamically-changing-number-of-points-%21%21
// will need to store it both CPU and GPU
// need it on CPU to process enclosed data
// need it on GPU for rendering
// thickness is inversely proportional


// method1: load your sprites, then render them as textured quad.
// method2: glBufferData Way



/*
void FogOfWar::updateFogByMainPlayer(glm::ivec2 prevGc)
{
	glm::ivec2 curGc = map.simPos2GridCoord(mainPlayer.simPos);

	if (prevGc != curGc)
	{		
		vector<FogCell> dirtyFogCells;
		fogManager.setSource(prevGc, mainPlayer.vision, FogManager::HIDDEN, dirtyFogCells);
		fogManager.setSource(curGc, mainPlayer.vision, FogManager::VISIBLE, dirtyFogCells);
		
		cout << "printing dirty fog cells" << endl;
		for (int i = 0; i < dirtyFogCells.size(); i++)
		{
			cout << "		" << dirtyFogCells[i].coord.x << " " << dirtyFogCells[i].coord.y << ", " << dirtyFogCells[i].data << endl;
		}
		
		fogView.addDirtyCells(dirtyFogCells);
	}
}
*/

void FogOfWar::update(GameState* gameState)
{
	int mx, my;
	SDL_GetMouseState(&mx, &my);

	// need this for GUI
	m_mouseState.m_pos = glm::vec2(mx, utl::SCREEN_HEIGHT - my);


	SDL_Event event;

	while (SDL_PollEvent(&event))
	{
		switch (event.type)
		{

			case SDL_KEYUP:
				switch (event.key.keysym.sym)
				{
					case SDLK_w:
					case SDLK_a:
					case SDLK_s:
					case SDLK_d:
						break;


				}
				break;

			case SDL_KEYDOWN:
				switch (event.key.keysym.sym)
				{
					case SDLK_ESCAPE:
						isRunning = false;
						break;

					case SDLK_q:
                        gameState->mainCamera.zoomOut();
						break;

					case SDLK_n:
						break;


					case SDLK_z:
						break;

					case SDLK_UP:
					//	m_cameraCenter.y += CAMERA_POS_DELTA;
					//	updateCamera();
						break;
					case SDLK_DOWN:
					//	m_cameraCenter.y -= CAMERA_POS_DELTA;
					//	updateCamera();
						break;
					case SDLK_LEFT:
					//	m_cameraCenter.x -= CAMERA_POS_DELTA;
					//	updateCamera();
						break;
					case SDLK_RIGHT:
					//	m_cameraCenter.x += CAMERA_POS_DELTA;
					//	updateCamera();
						break;

					default:
						break;
				}
				break;
			
			case SDL_MOUSEBUTTONDOWN:
				switch (event.button.button)
				{
					case SDL_BUTTON_LEFT:
						onMouseBtnDown(gameState);
						break;

					case SDL_BUTTON_RIGHT:
						onMouseBtnDown(gameState);
						break;
					case SDL_BUTTON_WHEELUP:
						// m_cameraZoom -= CAMERA_ZOOM_DELTA;
					//	updateCamera();
						break;
					case SDL_BUTTON_WHEELDOWN:
						// m_cameraZoom += CAMERA_ZOOM_DELTA;
					//	updateCamera();
						break;
				}
				break;

			case SDL_MOUSEBUTTONUP:
				switch (event.button.button)
				{
					case SDL_BUTTON_LEFT:
						onMouseBtnUp(gameState);
						break;
				}
				break;
		}
	}

	onMouseBtnHold(gameState);

//	glm::vec2 centerGridCoord =

}


void FogOfWar::onMouseBtnUp(GameState* gameState)
{
	int tmpx, tmpy;
	SDL_GetMouseState(&tmpx, &tmpy);
	tmpy = utl::SCREEN_HEIGHT - tmpy;

    GameCode::RemoveMouseJoint(gameState);
}


void FogOfWar::onMouseBtnHold(GameState* gameState)
{
    int tmpx, tmpy;
    SDL_GetMouseState(&tmpx, &tmpy);
    tmpy = utl::SCREEN_HEIGHT - tmpy;

    glm::vec2 screenPoint = glm::vec2(tmpx, tmpy);
    glm::vec3 worldPoint = GameCode::screenToWorldPoint(gameState, screenPoint);
    glm::vec2 tempWorldPoint = glm::vec2(worldPoint.x, worldPoint.y);

    GameCode::MoveMouseJoint(gameState, tempWorldPoint, FIXED_UPATE_TIME_s);
}

void FogOfWar::onMouseBtnDown(GameState* gameState)
{
    int tmpx, tmpy;
    SDL_GetMouseState(&tmpx, &tmpy);
    tmpy = utl::SCREEN_HEIGHT - tmpy;

    glm::vec2 screenPoint = glm::vec2(tmpx, tmpy);

    glm::vec3 worldPoint = GameCode::screenToWorldPoint(gameState, screenPoint);
    glm::vec3 raycastDir = glm::vec3(worldPoint.x, worldPoint.y, -1);
    
    // move this into GameCode
    cout << "OnMouseBtnDown" << endl;
    utl::debug("raycastDir", raycastDir);
    GameCode::ProcessInputRaycast(gameState, raycastDir);
}




/*
fixing the first and end point,

combine points that can do a line fit
*/
void FogOfWar::render(GameState* gameState)
{
	
//	fogView.render(mainCamera.getPipeline());

    GameRendering::render(gameState);

	long long timeNowMillis = getCurrentTimeMillis();

	int deltaTimeMillis = (unsigned int)(timeNowMillis - m_currentTimeMillis);
	m_currentTimeMillis = timeNowMillis;



	int fps = getAverageFPS();
	m_gui.setFPS(fps);

	m_gui.initGUIRenderingSetup();



	glDisable(GL_BLEND);


	m_gui.updateAndRender(m_mouseState);

	SDL_GL_SwapBuffers();
	frameNum++;
}


long long FogOfWar::getCurrentTimeMillis()
{
#ifdef WIN32
	return GetTickCount();
#else
	timeval t;
	gettimeofday(&t, NULL);

	long long ret = t.tv_sec * 1000 + t.tv_usec / 1000;
	return ret;
#endif
}



int main(int argc, char *argv[])
{
	utl::debug("Game Starting"); 
	utl::initSDL(utl::SCREEN_WIDTH, utl::SCREEN_HEIGHT, pDisplaySurface);
	utl::initGLEW();

	FogOfWar Martin;
    cout << sizeof(Entity) << endl;

	Martin.init();
	Martin.start();

	utl::exitSDL(pDisplaySurface);
	//normal termination

	while (1)
	{

	}

	cout << "Terminating normally." << endl;
	return EXIT_SUCCESS;
}


int FogOfWar::endWithError(char* msg, int error)
{
	//Display error message in console
	cout << msg << "\n";
	system("PAUSE");
	return error;
}


// http://kcat.strangesoft.net/mpstream.c



void FogOfWar::initGUI()
{
	// run this before m_gui.init, so the textEngine is initialized
	// need to comeback and re-organize the gui the minimize dependencies
	Control::init("", 25, utl::SCREEN_WIDTH, utl::SCREEN_HEIGHT);
	m_gui.init(utl::SCREEN_WIDTH, utl::SCREEN_HEIGHT);
}


void FogOfWar::renderGUI()
{

	m_gui.initGUIRenderingSetup();
	/// http://sdl.beuc.net/sdl.wiki/SDL_Average_FPS_Measurement
	//	unsigned int getTicks = SDL_GetTicks();

	//	static string final_str = "(" + utl::floatToStr(m_mouseState.m_pos.x) + ", " + utl::floatToStr(m_mouseState.m_pos.y) + ")";
	m_gui.updateAndRender(m_mouseState);

	// healthbar and text


}








