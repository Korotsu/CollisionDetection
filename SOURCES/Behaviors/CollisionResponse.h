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
	size_t	nbVelocityIteration = 1;
	size_t	nbPositionIteration = 1;
	std::vector<SCollision> lastFrameCollidingPairs = std::vector<SCollision>();
	Vec2 gravity = Vec2(0, -9.8f);

	virtual void Update(float frameTime) override
	{
		gVars->pWorld->ForEachPolygon([&](CPolygonPtr poly)
		{
			if (poly->density == 0.0f)
				return;
			if (gVars->bToggleGravity)
				poly->speed += gravity * frameTime;
			//
			poly->rotation.Rotate(RAD2DEG(/*Clamp(poly->angularVelocity, -1.0f, 1.0f)*/poly->angularVelocity * frameTime));
			poly->SetRotation(poly->rotation);

			poly->AddPosition(poly->speed * frameTime);
			//poly->angularVelocity = Clamp(poly->angularVelocity, -1.0f, 1.0f);
		});
		if (gVars->bToggleCollision)
		{
			//PreSolve();
			//WarmStart();
			/*for (size_t i = 0; i < nbVelocityIteration; i++)
			{
				SolveVelocity();
			}*/
			/*gVars->pWorld->ForEachPolygon([&](CPolygonPtr poly)
			{
				if (poly->density == 0.0f)
					return;

				poly->rotation.Rotate(RAD2DEG(/*Clamp(poly->angularVelocity, -1.0f, 1.0f)*//*poly->angularVelocity* frameTime));
				poly->SetRotation(poly->rotation);

				poly->AddPosition(poly->speed * frameTime);
			});*/
			/*for (size_t i = 0; i < nbPositionIteration; i++)
			{
				SolvePosition();
			}*/
			gVars->pPhysicEngine->ForEachCollision([&](SCollision& collision)
			{
				SolveVelocity(collision);
				//SolvePosition(collision);
			});
			//PostSolve();
		}

	}

	inline void PreSolve()
	{
		gVars->pPhysicEngine->ForEachCollision([&](SCollision& collision)
			{
				collision.lastCollisionPoint = Vec2(0.0f, 0.0f);
				collision.lastNormalImpulse = 0.0f;
				collision.lastTangentImpulse = 0.0f;
				for (const SCollision& lastPair : lastFrameCollidingPairs)
				{
					if (collision.index == lastPair.index)
					{
						collision.lastCollisionPoint = lastPair.point;
						collision.lastNormalImpulse = lastPair.lastNormalImpulse;
						collision.lastTangentImpulse = lastPair.lastTangentImpulse;
						break;
					}
				}

				float invMassA = collision.polyA->GetMass();
				float invMassB = collision.polyB->GetMass();

				Vec2 rAi = collision.point - collision.polyA->position;
				Vec2 rBi = collision.point - collision.polyB->position;

				float invLocalIA = collision.polyA->GetInertiaTensor();
				float invLocalIB = collision.polyB->GetInertiaTensor();

				Mat2 invWorldIA = collision.polyA->rotation * invLocalIA * collision.polyA->rotation.GetInverse();
				Mat2 invWorldIB = collision.polyB->rotation * invLocalIB * collision.polyB->rotation.GetInverse();

				float momentumA = (rAi ^ collision.normal);
				float momentumB = (rBi ^ collision.normal);

				float normalWeightedRotA = /*invLocalIA */ Vec2::Cross(momentumA, collision.normal) | collision.normal;
				float normalWeightedRotB = /*invLocalIB */ Vec2::Cross(momentumB, collision.normal) | collision.normal;

				collision.normalMass = (invMassA + invMassB + normalWeightedRotA + normalWeightedRotB);

				float tangentWeightedRotA = /*invLocalIA */ (rAi ^ collision.tangent) * (rAi ^ collision.tangent);
				float tangentWeightedRotB = /*invLocalIB */ (rBi ^ collision.tangent) * (rBi ^ collision.tangent);

				collision.tangentMass = (invMassA + invMassB /* + tangentWeightedRotA + tangentWeightedRotB*/);

				Vec3 collisionPoint = Vec3(collision.point.x, collision.point.y, 0.0f);
				Vec3 polyAPosition = Vec3(collision.polyA->position.x, collision.polyA->position.y, 0.0f);
				Vec3 polyBPosition = Vec3(collision.polyB->position.x, collision.polyB->position.y, 0.0f);
				Vec3 rAi2 = collisionPoint - polyAPosition;
				Vec3 rBi2 = collisionPoint - polyBPosition;
				Vec3 normal = Vec3(collision.normal.x, collision.normal.y, 0.0f);

				//Vec3 momentumA = invWorldIA * (rAi2 ^ normal);
				//Vec3 momentumB = invWorldIB * (rBi2 ^ normal);

				//float rotA = (momentumA ^ rAi2) | normal;
				//float rotB = (momentumB ^ rBi2) | normal;
			});
		lastFrameCollidingPairs.clear();
	}

	inline void WarmStart()
	{
		gVars->pPhysicEngine->ForEachCollision([&](const SCollision& collision)
			{
				if (collision.lastCollisionPoint.IsZero() || collision.lastNormalImpulse <= 0.0f || collision.lastTangentImpulse <= 0.0f)
					return;

				//Vec2 tangentImpulse = collision.tangent * collision.lastTangentImpulse;
				//Vec2 impulse = normalImpulse + tangentImpulse;

				ApplyImpulse(collision.polyA, collision.point, collision.tangent, collision.lastTangentImpulse);
				ApplyImpulse(collision.polyB, collision.point, collision.tangent, collision.lastTangentImpulse * -1.0f);
				
				//Vec2 normalImpulse = collision.normal * collision.lastNormalImpulse;

				ApplyImpulse(collision.polyA, collision.point, collision.normal, collision.lastNormalImpulse);
				ApplyImpulse(collision.polyB, collision.point, collision.normal, collision.lastNormalImpulse * -1.0f);
			});
	}

	inline void SolveVelocity(SCollision& collision)
	{
		//gVars->pPhysicEngine->ForEachCollision([&](SCollision& collision)
			//{
				if (collision.polyA->GetMass() == 0 && collision.polyB->GetMass() == 0 /*|| !collision.polyA->CheckCollision(*collision.polyB, collision) */ )
					return;

				float restitution = collision.polyA->bounciness * collision.polyB->bounciness;
				float friction = Min(collision.polyA->friction, collision.polyB->friction);

				Vec2 relativeVelocity = collision.polyA->speed - collision.polyB->speed;

				collision.tangent = relativeVelocity - collision.normal;

				//Tangent Impulse:
				float tangentRelVel = (relativeVelocity | collision.tangent);

				collision.tangentMass = (collision.polyA->GetMass() + collision.polyB->GetMass());
				float tangentImpulseDelta = -tangentRelVel / collision.tangentMass;
				float absMaxFriction = /*abs(collision.lastNormalImpulse) */abs(collision.distance) * friction;
				float newImpulse = Clamp(tangentImpulseDelta /*+ collision.lastTangentImpulse*/, -absMaxFriction, absMaxFriction);
				//tangentImpulseDelta = newImpulse - collision.lastTangentImpulse;
				//collision.lastTangentImpulse = newImpulse;

				//Vec2 tangentImpulse = collision.tangent * tangentImpulseDelta;

				if (collision.polyA->GetMass() != 0)
					collision.polyA->speed += collision.tangent * newImpulse * collision.polyA->GetMass();
				if (collision.polyB->GetMass() != 0)
					collision.polyB->speed -= collision.tangent * newImpulse * collision.polyA->GetMass();

				/*if (collision.polyA->GetMass() != 0)
					ApplyImpulse(collision.polyA, collision.point, collision.tangent, tangentImpulseDelta);
				if (collision.polyB->GetMass() != 0)
					ApplyImpulse(collision.polyB, collision.point, collision.tangent, -tangentImpulseDelta);*/

				float invMassA = collision.polyA->GetMass();
				float invMassB = collision.polyB->GetMass();

				Vec2 rAi = collision.point - collision.polyA->position;
				Vec2 rBi = collision.point - collision.polyB->position;

				float momentumA = (rAi ^ collision.normal);
				float momentumB = (rBi ^ collision.normal);

				float normalWeightedRotA = /*invLocalIA */ Vec2::Cross(momentumA, collision.normal) | collision.normal;
				float normalWeightedRotB = /*invLocalIB */ Vec2::Cross(momentumB, collision.normal) | collision.normal;

				collision.normalMass = (invMassA + invMassB + normalWeightedRotA + normalWeightedRotB);

				relativeVelocity = getRelativeVelocity(collision);
				float normalRelVel = (relativeVelocity | collision.normal);
				float normalImpulseDelta = (-1.0f * (restitution + 1.0f) * normalRelVel) / collision.normalMass;
				//float newNormalImpulse = Max(/*collision.lastNormalImpulse + */ normalImpulseDelta, 0.0f);
				//normalImpulseDelta = newNormalImpulse - collision.lastNormalImpulse;
				//collision.lastNormalImpulse = newNormalImpulse;

				//Vec2 normalImpulse = collision.normal * normalImpulseDelta;

				if (collision.polyA->GetMass() != 0)
					ApplyImpulse(collision.polyA, collision.point, collision.normal, normalImpulseDelta);
				if (collision.polyB->GetMass() != 0)
					ApplyImpulse(collision.polyB, collision.point, collision.normal * -1.0f, normalImpulseDelta);

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
			//});
	}

	inline void SolvePosition(SCollision& collision)
	{
		//gVars->pPhysicEngine->ForEachCollision([&](SCollision& collision)
			//{
				if (collision.polyA->GetMass() == 0 && collision.polyB->GetMass() == 0 /* || !collision.polyA->CheckCollision(*collision.polyB, collision)*/)
					return;

				//collision.polyA->CheckCollision(*collision.polyB, collision);
				float invMassA = collision.polyA->GetMass();
				float invMassB = collision.polyB->GetMass();

				Vec2 rAi = collision.point - collision.polyA->position;
				Vec2 rBi = collision.point - collision.polyB->position;

				//float distance = collision.distance; //collision.baseSeparation - (rAi.GetLength() + rBi.GetLength());

				// Apply position correction.
				float damping = 0.2f /* nbPositionIteration*/;
				float maxCorrection = 5.0f;
				float tolerance = 0.0f;
				float steeringForce = damping * (collision.distance + tolerance);//Clamp(damping * (collision.distance + tolerance), 0.1f, maxCorrection);
				//Vec3 impulse = collision.normal * ((steeringForce / (invMassA + invMassB)) / nbPositionIteration);
				float correction = (collision.distance * damping) / (invMassA + invMassB);
				if (collision.polyA->GetMass() != 0)
				{
					//collision.polyA->AddPosition(impulse * invMassA);
					collision.polyA->AddPosition(collision.normal * invMassA * correction);
					//collision.polyA->rotation.Rotate(rAi ^ impulse);
					//collision.polyA->SetRotation(collision.polyA->rotation);

				}

				if (collision.polyB->GetMass() != 0)
				{
					collision.polyB->AddPosition(collision.normal * invMassB * -correction);
					//collision.polyB->AddPosition(impulse * -invMassB);
					//collision.polyB->rotation.Rotate(rBi ^ (impulse * -1.0f));
					//collision.polyB->SetRotation(collision.polyB->rotation);
				}
			//});
	}

	inline void PostSolve()
	{
		gVars->pPhysicEngine->ForEachCollision([&](SCollision& collision)
			{
				lastFrameCollidingPairs.push_back(collision);
			});
	}

	inline void ApplyImpulse(CPolygonPtr object, Vec2 collisionPoint, Vec2 impulse)
	{
		if (object->GetMass() <= 0)
			return;

		Vec2 rAi = collisionPoint - object->position;
		object->speed += impulse * object->GetMass();
		object->angularVelocity += rAi ^ impulse;
	}

	inline void ApplyImpulse(CPolygonPtr object, Vec2 collisionPoint, Vec2 axis, float impulse)
	{
		if (object->GetMass() <= 0)
			return;

		Vec2 rAi = collisionPoint - object->position;
		object->speed += axis * impulse * object->GetMass();
		object->angularVelocity += impulse * (rAi ^ axis);
	}

	inline Vec2 getRelativeVelocity(SCollision& collision)
	{
		Vec2 rAi = collision.point - collision.polyA->position;
		Vec2 rBi = collision.point - collision.polyB->position;

		Vec2 velA = collision.polyA->speed + Vec2::Cross(collision.polyA->angularVelocity, rAi);
		Vec2 velB = collision.polyB->speed + Vec2::Cross(collision.polyB->angularVelocity, rBi);
		Vec2 result = (velA - velB);
		return result;//Vec2(abs(result.x), abs(result.y));
	}

	/*inline Vec2 getNormalMass(SCollision& collision)
	{
		Vec2 rAi = collision.point - collision.polyA->position;
		Vec2 rBi = collision.point - collision.polyB->position;

		Vec2 velA = collision.polyA->speed + (rAi ^ collision.polyA->angularVelocity);
		Vec2 velB = collision.polyB->speed + (rBi ^ collision.polyB->angularVelocity);
		Vec2 result = (velB - velA);
		return Vec2(abs(result.x), abs(result.y));
	}

	inline Vec2 getTangentMass(SCollision& collision)
	{
		Vec2 rAi = collision.point - collision.polyA->position;
		Vec2 rBi = collision.point - collision.polyB->position;

		Vec2 velA = collision.polyA->speed + (rAi ^ collision.polyA->angularVelocity);
		Vec2 velB = collision.polyB->speed + (rBi ^ collision.polyB->angularVelocity);
		Vec2 result = (velB - velA);
		return Vec2(abs(result.x), abs(result.y));
	}*/

};

#endif