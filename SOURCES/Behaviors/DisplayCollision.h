#ifndef _DISPLAY_COLLISION_H_
#define _DISPLAY_COLLISION_H_

#include "../Behavior.h"
#include "../PhysicEngine.h"
#include "../GlobalVariables.h"
#include "../Renderer.h"
#include "../RenderWindow.h"
#include "../World.h"

#include <string>
#include <iostream>

class CDisplayCollision : public CBehavior
{
public:
	CPolygonPtr polyA;
	CPolygonPtr polyB;

private:
	virtual void Update(float frameTime) override
	{
		//gVars->pPhysicEngine->Activate(false);
		SCollision collisionInfo;
		collisionInfo.polyA = polyA;
		collisionInfo.polyB = polyB;

		//Draw center cross.
		gVars->pRenderer->DrawLine(Vec2(-1,-1), Vec2(1,1), 0.0f, 0.0f, 0.0f);
		gVars->pRenderer->DrawLine(Vec2(-1, 1), Vec2(1,-1), 0.0f, 0.0f, 0.0f);

		if (gVars->bDebugElem)
		{
			std::vector<Vec2> outResult;
			Vec2 otherResult = Vec2();
			if (polyA->CheckCollisionDebug(*polyB, collisionInfo, otherResult, outResult))
			{
				gVars->pRenderer->DisplayText("Collision distance : " + std::to_string(collisionInfo.distance), 50, 50);
				
				if (gVars->bToggleEPADebug)
				{
					gVars->pRenderer->DisplayTextWorld("pt1", collisionInfo.point);
					gVars->pRenderer->DrawLine(collisionInfo.point, collisionInfo.point - collisionInfo.normal * (collisionInfo.distance - EPSILON), 1.0f, 0.0f, 1.0f);

					gVars->pRenderer->DisplayTextWorld("pt2", otherResult);
					gVars->pRenderer->DrawLine(otherResult, otherResult + collisionInfo.normal * (collisionInfo.distance - EPSILON), 1.0f, 0.0f, 1.0f);

				}
				else
				{
					gVars->pRenderer->DisplayTextWorld("pt", collisionInfo.point);
					gVars->pRenderer->DrawLine(collisionInfo.point, collisionInfo.point - collisionInfo.normal * (collisionInfo.distance - EPSILON), 1.0f, 0.0f, 1.0f);
				}

				if (gVars->bToggleLastSimplexDraw)
				{
					for (size_t i = outResult.size() - 3; i < outResult.size() - 2; i++)
					{
						gVars->pRenderer->DrawLine(outResult[i], outResult[i + 1], 0.0f, 0.0f, 0.0f);
						gVars->pRenderer->DrawLine(outResult[i + 1], outResult[i + 2], 0.0f, 0.0f, 0.0f);
						gVars->pRenderer->DrawLine(outResult[i + 2], outResult[i], 0.0f, 0.0f, 0.0f);
					}
				}
				
			}
			if (gVars->bToggleMinkoskiCreationDraw || gVars->bToggleMinkoskiShapeDraw)
			{
				std::vector<Vec2> outA;
				std::vector<Vec2> outB;
				std::vector<Vec2> result = polyA->MinkovskiDiff(*polyB, outA, outB);
				for (size_t i = 1; i < result.size(); i++)
				{
					if (gVars->bToggleMinkoskiCreationDraw)
					{
						gVars->pRenderer->DrawLine(outA[i - 1], Vec2(0, 0), 1.0f, 0.0f, 0.0f);
						gVars->pRenderer->DrawLine(Vec2(0, 0), outA[i - 1] * -1						, 1.0f, 0.0f, 0.0f);
						gVars->pRenderer->DrawLine(Vec2(0, 0), outB[i - 1]							, 0.0f, 1.0f, 1.0f);
						gVars->pRenderer->DrawLine(outA[i - 1] * -1, outA[i - 1] * -1 + outB[i - 1]	, 0.0f, 1.0f, 1.0f);
						gVars->pRenderer->DrawLine(outA[i], Vec2(0, 0)								, 1.0f, 0.0f, 0.0f);
						gVars->pRenderer->DrawLine(Vec2(0, 0), outA[i] * -1							, 1.0f, 0.0f, 0.0f);
						gVars->pRenderer->DrawLine(Vec2(0, 0), outB[i]								, 0.0f, 1.0f, 1.0f);
						gVars->pRenderer->DrawLine(outA[i] * -1, outA[i] * -1 + outB[i]				, 0.0f, 1.0f, 1.0f);
					}
					if (gVars->bToggleMinkoskiShapeDraw)
						gVars->pRenderer->DrawLine(result[i - 1], result[i]							, 0.0f, 0.0f, 1.0f);
				}
			}
		}
		else if (polyA->CheckCollision(*polyB, collisionInfo))
		{
			gVars->pRenderer->DisplayText("Collision distance : " + std::to_string(collisionInfo.distance), 50, 50);

			gVars->pRenderer->DisplayTextWorld("pt", collisionInfo.point);
			gVars->pRenderer->DrawLine(collisionInfo.point, collisionInfo.point - collisionInfo.normal * (collisionInfo.distance - EPSILON), 1.0f, 0.0f, 1.0f);
		}
	}
};


#endif