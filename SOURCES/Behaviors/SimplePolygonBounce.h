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
			if (gVars->bToggleCollision)
			{
				collision.polyA->AddPosition(collision.normal * collision.distance * 0.5f);
				collision.polyB->AddPosition(collision.normal * collision.distance * -0.5f);

				collision.polyA->speed.Reflect(collision.normal);
				collision.polyB->speed.Reflect(collision.normal * -1.0f);
			}
			if (gVars->bDebugElem && gVars->bToggleEPADebug)
			{
				gVars->pRenderer->DisplayTextWorld("ptA", collision.polyA->position + collision.normal * collision.distance);
				gVars->pRenderer->DrawLine(collision.polyA->position, collision.polyA->position + collision.normal * collision.distance, 1.0f, 0.0f, 1.0f);

				gVars->pRenderer->DisplayTextWorld("ptB", collision.polyB->position - collision.normal * collision.distance);
				gVars->pRenderer->DrawLine(collision.polyB->position, collision.polyB->position - collision.normal * collision.distance, 1.0f, 0.0f, 1.0f);

				gVars->pRenderer->DisplayText("Collision distance : " + std::to_string(collision.distance), 50, 50);

				gVars->pRenderer->DisplayTextWorld("pt", collision.point);
				gVars->pRenderer->DrawLine(collision.point, collision.point - collision.normal * (collision.distance - EPSILON), 1.0f, 0.0f, 1.0f);
			}
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