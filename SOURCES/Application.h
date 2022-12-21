#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include "GlobalVariables.h"
#include "SDLRenderWindow.h"
#include "PhysicEngine.h"
#include "Renderer.h"
#include "SceneManager.h"
#include "World.h"

void InitApplication(int width, int height, float worldHeight)
{
	gVars = new SGlobalVariables();

	gVars->pRenderWindow = new CSDLRenderWindow(width, height);
	gVars->pRenderer = new CRenderer(worldHeight);
	gVars->pSceneManager = new CSceneManager();
	gVars->pPhysicEngine = new CPhysicEngine();

	gVars->bDebug = false;
	gVars->bDebugElem = false;
	gVars->bToggleAABB = false;
	gVars->bToggleMinkoskiCreationDraw = false;
	gVars->bToggleMinkoskiShapeDraw = false;
	gVars->bToggleLastSimplexDraw = false;
	gVars->bToggleEPADebug = false;
	gVars->bToggleCollision = true;
}

void RunApplication()
{
	gVars->pRenderWindow->Init();
}

#endif