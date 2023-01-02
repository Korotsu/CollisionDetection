#ifndef _SCENE_BOUNCING_POLYS_H_
#define _SCENE_BOUNCING_POLYS_H_

#include "BaseScene.h"

#include "Behaviors/SimplePolygonBounce.h"
#include "Behaviors/CollisionResponse.h"

class CSceneBouncingPolys : public CBaseScene
{
private:
	Vec2 polySize = Vec2(1.0f, 1.0f);
public:
	CSceneBouncingPolys(size_t polyCount, Vec2 _polySize = Vec2(1.0f, 1.0f))
		: m_polyCount(polyCount), polySize(_polySize){}

protected:
	virtual void Create() override
	{
		CBaseScene::Create();

		//gVars->pWorld->AddBehavior<CSimplePolygonBounce>(nullptr);
		gVars->pWorld->AddBehavior<CCollisionResponse>(nullptr);

		float width = gVars->pRenderer->GetWorldWidth();
		float height = gVars->pRenderer->GetWorldHeight();

		SRandomPolyParams params;
		params.minRadius = polySize.x;
		params.maxRadius = polySize.y;
		params.minBounds = Vec2(-width * 0.5f + params.maxRadius * 3.0f, -height * 0.5f + params.maxRadius * 3.0f);
		params.maxBounds = params.minBounds * -1.0f;
		params.minPoints = 3;
		params.maxPoints = 8;
		params.minSpeed = 1.0f;
		params.maxSpeed = 3.0f;
		
		for (size_t i = 0; i < m_polyCount; ++i)
		{
			gVars->pWorld->AddRandomPoly(params);// ->density = 0.0f;
		}
	}

private:
	size_t m_polyCount;
};

#endif