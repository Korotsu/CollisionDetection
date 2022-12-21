#ifndef _GLOBAL_VARIABLES_H_
#define _GLOBAL_VARIABLES_H_

struct SGlobalVariables
{
	class CRenderWindow* pRenderWindow;
	class CRenderer* pRenderer;
	class CWorld* pWorld;
	class CSceneManager* pSceneManager;
	class CPhysicEngine* pPhysicEngine;

	bool					bDebug;
	bool					bDebugElem;
	bool					bToggleAABB;
	bool					bToggleMinkoskiCreationDraw;
	bool					bToggleMinkoskiShapeDraw;
	bool					bToggleLastSimplexDraw;
	bool					bToggleEPADebug;
	bool					bToggleCollision;
};

extern SGlobalVariables* gVars;

#endif