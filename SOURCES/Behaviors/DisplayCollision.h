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
		//polyA->CheckCollision(*polyB, point, normal, dist);
		if (gVars->bDebugElem)
		{
			std::vector<Vec2> outA;
			std::vector<Vec2> outB;
			std::vector<Vec2> outC;
			std::vector<Vec2> outD;
			std::vector<Vec2> result = polyA->MinkovskiDiff(*polyB, outA, outB);
			std::vector<Vec2> outResult;
			polyA->CheckCollisionWithDebug(*polyB, point, normal, dist, outResult, outC, outD);

			for (size_t i = outResult.size() - 3; i < outResult.size() - 2; i++)
			{
				gVars->pRenderer->DrawLine(outResult[i], outResult[i + 1], 0.0f, 0.0f, 0.0f);
				gVars->pRenderer->DrawLine(outResult[i + 1], outResult[i + 2], 0.0f, 0.0f, 0.0f);
				gVars->pRenderer->DrawLine(outResult[i + 2], outResult[i], 0.0f, 0.0f, 0.0f);
			}
			//gVars->pRenderer->DrawLine(outResult[outResult.size() - 1], outResult.back(), 0.0f, 0.0f, 0.0f);
			gVars->pRenderer->DrawLine(outC.back(), Vec2(0, 0), 1.0f, 0.0f, 0.0f);
			gVars->pRenderer->DrawLine(Vec2(0, 0), outC.back() * -1, 1.0f, 0.0f, 0.0f);
			gVars->pRenderer->DrawLine(outD.back(), Vec2(0, 0), 0.0f, 1.0f, 0.0f);
			gVars->pRenderer->DrawLine(outC.back(), outC.back() + outD.back(), 0.0f, 1.0f, 0.0f);

			for (size_t i = 1; i < result.size(); i++)
			{
				/*gVars->pRenderer->DrawLine(outA[i - 1], Vec2(0, 0), 1.0f, 0.0f, 0.0f);
				gVars->pRenderer->DrawLine(Vec2(0, 0), outA[i - 1] * -1, 1.0f, 0.0f, 0.0f);
				gVars->pRenderer->DrawLine(Vec2(0, 0), outB[i - 1], 0.0f, 1.0f, 0.0f);
				gVars->pRenderer->DrawLine(outA[i - 1] * -1, outA[i - 1] * -1 + outB[i - 1], 0.0f, 1.0f, 0.0f);
				gVars->pRenderer->DrawLine(outA[i], Vec2(0, 0), 1.0f, 0.0f, 0.0f);
				gVars->pRenderer->DrawLine(Vec2(0, 0), outA[i] * -1, 1.0f, 0.0f, 0.0f);
				gVars->pRenderer->DrawLine(Vec2(0, 0), outB[i], 0.0f, 1.0f, 0.0f);
				gVars->pRenderer->DrawLine(outA[i] * -1, outA[i] * -1 + outB[i], 0.0f, 1.0f, 0.0f);*/
				gVars->pRenderer->DrawLine(result[i - 1], result[i], 0.0f, 0.0f, 1.0f);
			}
		}
	}
};


#endif