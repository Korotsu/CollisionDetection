// CollisionEngine.cpp : définit le point d'entrée pour l'application console.
//

#pragma comment(lib, "legacy_stdio_definitions.lib")
#include "stdafx.h"

#include <iostream>
#include <string>

#include "Application.h"
#include "SceneManager.h"

#include "Scenes/SceneDebugCollisions.h"
#include "Scenes/SceneBouncingPolys.h"
#include "Scenes/SceneSimplePhysic.h"
#include "Scenes/SceneSpheres.h"
#include "Scenes/SceneComplexPhysic.h"
#include "Scenes/SceneSmallPhysic.h"
#include "SceneFluid.h"

extern "C" { FILE __iob_func[3] = { *stdin,*stdout,*stderr }; }
/*
* Entry point
*/
int _tmain(int argc, char** argv)
{
	InitApplication(1260, 768, 50.0f);

	gVars->pSceneManager->AddScene(new CSceneDebugCollisions());
	gVars->pSceneManager->AddScene(new CSceneBouncingPolys(2, Vec2(5.0f, 10.0f)));
	gVars->pSceneManager->AddScene(new CSceneBouncingPolys(200));
	gVars->pSceneManager->AddScene(new CSceneSpheres());
	gVars->pSceneManager->AddScene(new CSceneSmallPhysic());
	gVars->pSceneManager->AddScene(new CSceneSimplePhysic());
	gVars->pSceneManager->AddScene(new CSceneComplexPhysic(25));
	//gVars->pSceneManager->AddScene(new CSceneFluid());

	RunApplication();
	return 0;
}