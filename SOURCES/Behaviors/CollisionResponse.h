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
	size_t	nbVelocityIteration = 6;
	size_t	nbPositionIteration = 1;
	std::vector<SCollision> lastFrameCollidingPairs = std::vector<SCollision>();
	Vec2 gravity = Vec2(0, -9.8f);

	virtual void Update(float frameTime) override
	{
		if (gVars->bToggleCollision)
		{
			PreSolve();
			WarmStart();
			for (size_t i = 0; i < nbVelocityIteration; i++)
			{
				SolveVelocity();
			}
			gVars->pWorld->ForEachPolygon([&](CPolygonPtr poly)
			{
				if (poly->density == 0.0f)
					return;
				if (gVars->bToggleGravity)
					poly->speed += gravity * frameTime;

				poly->rotation.Rotate(RAD2DEG(poly->angularVelocity * frameTime));
				poly->SetRotation(poly->rotation);
				poly->AddPosition(poly->speed * frameTime);
			});
			for (size_t i = 0; i < nbPositionIteration; i++)
			{
				SolvePosition();
			}
			PostSolve();
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

			Vec2 rAi = collision.point - collision.polyA->position;
			Vec2 rBi = collision.point - collision.polyB->position;

			collision.baseSeparation = collision.distance + rAi.GetLength() + rBi.GetLength();
		});
		lastFrameCollidingPairs.clear();
	}

	inline void WarmStart()
	{
		gVars->pPhysicEngine->ForEachCollision([&](const SCollision& collision)
		{
			if (collision.lastCollisionPoint.IsZero() || collision.lastNormalImpulse <= 0.0f || collision.lastTangentImpulse <= 0.0f)
				return;

			Vec2 normalImpulse = collision.normal * collision.lastNormalImpulse;
			Vec2 tangentImpulse = collision.tangent * collision.lastTangentImpulse * 0.2f;
			Vec2 impulse = normalImpulse + tangentImpulse;

			ApplyImpulse(collision.polyA, collision.point, impulse.Normalized(), impulse.GetLength());
		});
	}

	inline void SolveVelocity()
	{
		gVars->pPhysicEngine->ForEachCollision([&](SCollision& collision)
		{
			if (collision.polyA->GetMass() == 0 && collision.polyB->GetMass() == 0)
				return;

			float restitution = collision.polyA->bounciness * collision.polyB->bounciness;
			float friction = Min(collision.polyA->friction, collision.polyB->friction);

			Vec2 relativeVelocity = collision.polyB->speed - collision.polyA->speed;

			collision.tangent = relativeVelocity - collision.normal;

			//Tangent Impulse:
			float tangentRelVel = ((relativeVelocity * -1.0f)| collision.tangent);

			collision.tangentMass = (collision.polyA->GetMass() + collision.polyB->GetMass());
			float tangentImpulseDelta = tangentRelVel / collision.tangentMass;
			float absMaxFriction = abs(collision.lastNormalImpulse) * friction;
			float newImpulse = Clamp(collision.lastTangentImpulse + tangentImpulseDelta, -absMaxFriction, absMaxFriction);
			tangentImpulseDelta = newImpulse - collision.lastTangentImpulse;
			collision.lastTangentImpulse = newImpulse;
			if (collision.polyA->GetMass() != 0)
				ApplyImpulse(collision.polyA, collision.point, collision.tangent, -tangentImpulseDelta);
			if (collision.polyB->GetMass() != 0)
				ApplyImpulse(collision.polyB, collision.point, collision.tangent, tangentImpulseDelta);

			float invMassA = collision.polyA->GetMass();
			float invMassB = collision.polyB->GetMass();

			Vec2 rAi = collision.point - collision.polyA->position;
			Vec2 rBi = collision.point - collision.polyB->position;

			float momentumA = (rAi ^ collision.normal);
			float momentumB = (rBi ^ collision.normal);

			float normalWeightedRotA = Vec2::Cross(momentumA, rAi) | collision.normal;
			float normalWeightedRotB = Vec2::Cross(momentumB, rBi) | collision.normal;

			collision.normalMass = (invMassA + invMassB + normalWeightedRotA + normalWeightedRotB);

			relativeVelocity = getRelativeVelocity(collision);
			float normalRelVel = (relativeVelocity | collision.normal);
			float normalImpulseDelta = (-1.0f * (restitution + 1.0f) * normalRelVel) / collision.normalMass;
			float newNormalImpulse = Max(collision.lastNormalImpulse + normalImpulseDelta, 0.0f);
			normalImpulseDelta = newNormalImpulse - collision.lastNormalImpulse;
			collision.lastNormalImpulse = newNormalImpulse;

			if (collision.polyA->GetMass() != 0)
				ApplyImpulse(collision.polyA, collision.point, collision.normal, normalImpulseDelta * -1.0f);
			if (collision.polyB->GetMass() != 0)
				ApplyImpulse(collision.polyB, collision.point, collision.normal, normalImpulseDelta);

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
	}

	inline void SolvePosition()
	{
		gVars->pPhysicEngine->ForEachCollision([&](SCollision& collision)
		{
			if (collision.polyA->GetMass() == 0 && collision.polyB->GetMass() == 0)
				return;
			float invMassA = collision.polyA->GetMass();
			float invMassB = collision.polyB->GetMass();

			Vec2 rAi = collision.point - collision.polyA->position;
			Vec2 rBi = collision.point - collision.polyB->position;

			float distance = collision.baseSeparation - (rAi.GetLength() + rBi.GetLength());

			// Apply position correction.
			float damping = 0.2f;
			float maxCorrection = -5.0f;
			float tolerance = 0.01f;
			float separation = -1.0f * distance;
			float steeringForce = Clamp(damping * (separation + tolerance), maxCorrection, 0.0f);
			Vec2 correction = collision.normal * ((-steeringForce) / (invMassA + invMassB));
			if (collision.polyA->GetMass() != 0)
			{
				collision.polyA->AddPosition(correction * invMassA * -1.0f);
				collision.polyA->rotation.Rotate((rAi ^ correction) * -1.0f);
				collision.polyA->SetRotation(collision.polyA->rotation);

			}

			if (collision.polyB->GetMass() != 0)
			{
				collision.polyB->AddPosition(correction * invMassB);
				collision.polyB->rotation.Rotate(rBi ^ correction);
				collision.polyB->SetRotation(collision.polyB->rotation);
			}
		});
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
		return (velB - velA);
	}
};

#endif