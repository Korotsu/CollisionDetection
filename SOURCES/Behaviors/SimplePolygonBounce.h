#ifndef _SIMPLE_COLLISION_BOUNCE_H_
#define _SIMPLE_COLLISION_BOUNCE_H_

#include "Behavior.h"
#include "PhysicEngine.h"
#include "GlobalVariables.h"
#include "Renderer.h"
#include "World.h"

class CSimplePolygonBounce : public CBehavior
{
private:
	virtual void Update(float frameTime) override
	{
		gVars->pPhysicEngine->ForEachCollision([&](const SCollision& collision)
		{
			collision.polyA->AddPosition(collision.normal * collision.distance * -0.5f);
			collision.polyB->AddPosition(collision.normal * collision.distance * 0.5f);

			collision.polyA->speed.Reflect(collision.normal);
			collision.polyB->speed.Reflect(collision.normal);
		});

		float hWidth = gVars->pRenderer->GetWorldWidth() * 0.5f;
		float hHeight = gVars->pRenderer->GetWorldHeight() * 0.5f;

		gVars->pWorld->ForEachPolygon([&](CPolygonPtr poly)
		{
			poly->AddPosition(poly->speed * frameTime);
			Vec2 result = poly->position;
			if (result.x < -hWidth)
			{
				result.x = -hWidth;
				poly->speed.x *= -1.0f;
			}
			else if (result.x > hWidth)
			{
				result.x = hWidth;
				poly->speed.x *= -1.0f;
			}
			if (result.y < -hHeight)
			{
				result.y = -hHeight;
				poly->speed.y *= -1.0f;
			}
			else if (result.y > hHeight)
			{
				result.y = hHeight;
				poly->speed.y *= -1.0f;
			}
			poly->SetPosition(result);
		});
	}
};

#endif