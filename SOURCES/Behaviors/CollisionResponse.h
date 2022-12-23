#ifndef _COLLISION_RESPONSE_H_
#define _COLLISION_RESPONSE_H_

#include "Behavior.h"
#include "PhysicEngine.h"
#include "GlobalVariables.h"
#include "Renderer.h"
#include "World.h"

class CCollisionResponse : public CBehavior
{
private:
	virtual void Update(float frameTime) override
	{
		gVars->pPhysicEngine->ForEachCollision([&](const SCollision& collision)
		{
			if (gVars->bToggleCollision)
			{
				//if (collision.polyA->GetMass() != 0 && collision.polyB->GetMass() != 0)
				//{
				if (collision.polyA->GetMass() == 0 && collision.polyB->GetMass() == 0)
					return;
				else if (collision.polyA->GetMass() == 0)
					collision.polyB->AddPosition(collision.normal * collision.distance * -1.0f);
				else if (collision.polyB->GetMass() == 0)
					collision.polyA->AddPosition(collision.normal * collision.distance * 1.0f);
				else
				{
					collision.polyA->AddPosition(collision.normal * collision.distance * 0.5f);
					collision.polyB->AddPosition(collision.normal * collision.distance * -0.5f);
				}

				Vec2 rAi = collision.point - collision.polyA->position;
				Vec2 rBi = collision.point - collision.polyB->position;

				Vec2 vAi = collision.polyA->GetPointVelocity(rAi);
				Vec2 vBi = collision.polyB->GetPointVelocity(rBi);

				float invLocalIA = collision.polyA->GetInertiaTensor();
				float invLocalIB = collision.polyB->GetInertiaTensor();

				Mat2 invWorldIA = collision.polyA->rotation * invLocalIA * collision.polyA->rotation.GetInverse();
				Mat2 invWorldIB = collision.polyB->rotation * invLocalIB * collision.polyB->rotation.GetInverse();

				Vec2 momentumA = invWorldIA * (collision.normal - rAi).GetNormal();
				Vec2 momentumB = invWorldIB * (collision.normal - rBi).GetNormal();

				float rotA = (rAi - momentumA).GetNormal() | collision.normal;
				float rotB = (rBi - momentumB).GetNormal() | collision.normal;
				
				float invMassA = collision.polyA->GetMass();
				float invMassB = collision.polyB->GetMass();

				float rVel = (vAi - vBi) | collision.normal;
				if (rVel >= 0)
					return;

				float impulse = ComputeImpulse(invMassA, invMassB, rotA, rotB, rVel, 0.0f);
				
				collision.polyA->speed += collision.normal * (impulse * invMassA);
				collision.polyB->speed -= collision.normal * (impulse * invMassB);

				if (true)
				{

				}
				float testA = Clamp(collision.normal | momentumA.Normalized(),-1.0f,1.0f);
				float testB = Clamp(collision.normal | momentumB.Normalized(),-1.0f,1.0f);

				collision.polyA->angularVelocity += /*momentumA.GetLength() */ impulse;
				collision.polyB->angularVelocity -= /*momentumB.GetLength() */ impulse;
				//}

				/*else if (collision.polyA->GetMass() == 0)
				{
					collision.polyA->AddPosition(collision.normal * collision.distance * 0.5f);
					collision.polyB->AddPosition(collision.normal * collision.distance * -0.5f);

					Vec2 rAi = collision.point - collision.polyA->position;
					Vec2 rBi = collision.point - collision.polyB->position;

					Vec2 vAi = collision.polyA->GetPointVelocity(rAi);
					Vec2 vBi = collision.polyB->GetPointVelocity(rBi);

					float invLocalIA = 1 / collision.polyA->GetInertiaTensor();
					float invLocalIB = 1 / collision.polyB->GetInertiaTensor();

					Mat2 invWorldIA = collision.polyA->rotation * invLocalIA * collision.polyA->rotation.GetInverse();
					Mat2 invWorldIB = collision.polyB->rotation * invLocalIB * collision.polyB->rotation.GetInverse();

					Vec2 momentumA = invWorldIA * (collision.normal - rAi).GetNormal();
					Vec2 momentumB = invWorldIB * (collision.normal - rBi).GetNormal();

					float rotA = (rAi - momentumA).GetNormal() | collision.normal;
					float rotB = (rBi - momentumB).GetNormal() | collision.normal;

					float invMassA = 1 / collision.polyA->GetMass();
					float invMassB = 1 / collision.polyB->GetMass();

					float rVel = (vAi - vBi) | collision.normal;
					if (rVel >= 0)
						return;

					float impulse = ComputeImpulse(invMassA, invMassB, rotA, rotB, rVel, 0.0f);

					collision.polyA->speed += collision.normal * (impulse * invMassA);
					collision.polyB->speed -= collision.normal * (impulse * invMassB);

					collision.polyA->angularVelocity += /*momentumA.GetLength() */ // impulse;
					//collision.polyB->angularVelocity -= /*momentumB.GetLength() */ impulse;
				//}

				/*else if (collision.polyB->GetMass() == 0)
				{
						
				}*/
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
			//poly->AddPosition(poly->speed * frameTime);
			
			//poly->rotation.Rotate(poly->angularVelocity * frameTime);
			//poly->SetRotation(poly->rotation);

			/*Vec2 result = poly->position;
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
			poly->SetPosition(result);*/
		});
	}

	inline float ComputeImpulse(const float invMassA, const float invMassB, const float rotA, const float rotB, const float rVel, const float bounciness = 1.0f) const
	{
		return ( -1.0f * ( bounciness + 1.0f ) * rVel ) /
				(invMassA + invMassB + rotA + rotB );
	}

};

#endif