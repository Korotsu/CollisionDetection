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
		// gVars->pPhysicEngine->Activate(false);


		Vec2 point, normal;
		float dist;
		/*if (polyA->CheckCollision(*polyB, point, normal, dist))
		{
			gVars->pRenderer->DisplayTextWorld("collision point", point);
			gVars->pRenderer->DisplayText("Collision distance : " + std::to_string(dist), 50, 50);

			//gVars->pRenderer->DrawLine(point, point + normal * dist, 0.0f, 1.0f, 0.0f);
		}*/
		if (gVars->bDebugElem)
		{
			std::vector<Vec2> outA;
			std::vector<Vec2> outB;
			std::vector<Vec2> result = polyA->MinkovskiDiff(*polyB, outA, outB);
			for (int i = 1; i < result.size(); i++)
			{
				gVars->pRenderer->DrawLine(outA[i - 1], Vec2(0, 0), 1.0f, 0.0f, 0.0f);
				gVars->pRenderer->DrawLine(Vec2(0, 0), outA[i - 1] * -1, 1.0f, 0.0f, 0.0f);
				gVars->pRenderer->DrawLine(Vec2(0, 0), outB[i - 1], 0.0f, 1.0f, 0.0f);
				gVars->pRenderer->DrawLine(outA[i - 1] * -1, outA[i - 1] * -1 + outB[i - 1], 0.0f, 1.0f, 0.0f);
				gVars->pRenderer->DrawLine(outA[i], Vec2(0, 0), 1.0f, 0.0f, 0.0f);
				gVars->pRenderer->DrawLine(Vec2(0, 0), outA[i] * -1, 1.0f, 0.0f, 0.0f);
				gVars->pRenderer->DrawLine(Vec2(0, 0), outB[i], 0.0f, 1.0f, 0.0f);
				gVars->pRenderer->DrawLine(outA[i] * -1, outA[i] * -1 + outB[i], 0.0f, 1.0f, 0.0f);
				gVars->pRenderer->DrawLine(result[i - 1], result[i], 0.0f, 0.0f, 1.0f);
			}
		}
	}
};


#endif