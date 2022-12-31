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
	size_t	nbVelocityIteration = 8;
	size_t	nbPositionIteration = 3;
	std::vector<SCollision> lastFrameCollidingPairs = std::vector<SCollision>();

	virtual void Update(float frameTime) override
	{
		if (gVars->bToggleCollision)
		{
			PreSolve();
			//WarmStart();
			for (size_t i = 0; i < nbVelocityIteration; i++)
			{
				SolveVelocity();
			}
			for (size_t i = 0; i < nbPositionIteration; i++)
			{
				SolvePosition();
			}
			PostSolve();
		}
	}

	inline float ComputeImpulse(const float invMassA, const float invMassB, const float rotA, const float rotB, const float rVel, const float bounciness = 1.0f) const
	{
		return (-1.0f * (bounciness + 1.0f) * rVel) /
			(invMassA + invMassB + rotA + rotB);
	}

	inline void ApplyTangentImpulse(CPolygonPtr A, CPolygonPtr B, Vec3 relvel, Vec3 normal, float colImpulse, float frictionCoef = 0.5f) const
	{
		Vec3 tangent = Vec3::GetTangent(relvel, normal);
		float vTangent = relvel | tangent;
		float J = -vTangent / (A->GetMass() + B->GetMass());
		J = Clamp(J, - abs(colImpulse) * frictionCoef, abs(colImpulse) * frictionCoef);
		Vec3 resultA = tangent * (J * A->GetMass());
		Vec3 resultB = tangent * (J * B->GetMass());
		A->speed += Vec2(resultA.x, resultA.y);
		B->speed -= Vec2(resultB.x, resultB.y);
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
			Vec2 tangentImpulse = collision.tangent * collision.lastTangentImpulse;

			Vec2 impulse = normalImpulse + tangentImpulse;

			ApplyImpulse(collision.polyA, collision.lastCollisionPoint, impulse);
			ApplyImpulse(collision.polyB, collision.lastCollisionPoint, impulse * -1.0f);
		});
	}

	inline void SolveVelocity()
	{
		gVars->pPhysicEngine->ForEachCollision([&](SCollision& collision)
		{
			//if (gVars->bToggleCollision)
			//{
				if (collision.polyA->GetMass() == 0 && collision.polyB->GetMass() == 0)
					return;

				float restitution = collision.polyA->bounciness * collision.polyB->bounciness;
				float friction = Min(collision.polyA->friction, collision.polyB->friction);

				Vec3 collisionPoint = Vec3(collision.point.x, collision.point.y, 0.0f);
				Vec3 polyAPosition = Vec3(collision.polyA->position.x, collision.polyA->position.y, 0.0f);
				Vec3 polyBPosition = Vec3(collision.polyB->position.x, collision.polyB->position.y, 0.0f);

				float invMassA = collision.polyA->GetMass();
				float invMassB = collision.polyB->GetMass();

				Vec3 rAi = collisionPoint - polyAPosition;
				Vec3 rBi = collisionPoint - polyBPosition;

				Vec3 vA = Vec3(collision.polyA->speed.x, collision.polyA->speed.y, 0.0f);
				Vec3 vB = Vec3(collision.polyB->speed.x, collision.polyB->speed.y, 0.0f);

				Mat2 matRotA;
				matRotA.SetAngle(collision.polyA->angularVelocity);

				Mat2 matRotB;
				matRotB.SetAngle(collision.polyB->angularVelocity);

				Vec3 wA = Vec3(matRotA.X.x, matRotA.Y.y, 0.0f);
				Vec3 wB = Vec3(matRotB.X.x, matRotB.Y.y, 0.0f);

				Vec3 vAi = vA + wA ^ rAi;
				Vec3 vBi = vB + wB ^ rBi;

				float invLocalIA = collision.polyA->GetInertiaTensor();
				float invLocalIB = collision.polyB->GetInertiaTensor();

				Mat2 invWorldIA = collision.polyA->rotation * invLocalIA * collision.polyA->rotation.GetInverse();
				Mat2 invWorldIB = collision.polyB->rotation * invLocalIB * collision.polyB->rotation.GetInverse();

				Vec3 normal = Vec3(collision.normal.x, collision.normal.y, 0.0f);

				Vec3 momentumA = invWorldIA * (rAi ^ normal);
				Vec3 momentumB = invWorldIB * (rBi ^ normal);

				float rotA = (momentumA ^ rAi) | normal;
				float rotB = (momentumB ^ rBi) | normal;

				float normalRelVel = collision.relativeVelocity | collision.normal;
				if (normalRelVel >= 0)
					return;

				//float normalImpulse = Max(ComputeImpulse(invMassA, invMassB, rotA, rotB, normalRelVel, 0.0f), 0.0f);

				float normalImpulseDelta = (-1.0f * (restitution + 1.0f) * normalRelVel) / (invMassA + invMassB + rotA + rotB);
				float newNormalImpulse = Max(collision.lastNormalImpulse + normalImpulseDelta, 0.0f);
				normalImpulseDelta = newNormalImpulse - collision.lastNormalImpulse;
				collision.lastNormalImpulse = newNormalImpulse;

				Vec2 normalImpulse = collision.normal * normalImpulseDelta;

				// Apply Friction.
				/*ApplyTangentImpulse(collision.polyA, collision.polyB, (vA - vB), normal, normalImpulse);

				Vec3 aSpeed = vA + normal * (normalImpulse * invMassA);
				Vec3 bSpeed = vB - normal * (normalImpulse * invMassB);

				Vec3 aAngSpeed = wA + momentumA * normalImpulse;
				Vec3 bAngSpeed = wB + momentumB * normalImpulse;*/

				//Tangent Impulse:
				float vTangent = collision.relativeVelocity | collision.tangent;
				float tangentImpulseDelta = -vTangent / (invMassA + invMassB + rotA + rotB);
				float absMaxFriction = abs(collision.lastNormalImpulse) * friction;
				float newImpulse = Clamp(tangentImpulseDelta + collision.lastTangentImpulse, -absMaxFriction, absMaxFriction);
				tangentImpulseDelta = newImpulse - collision.lastTangentImpulse;
				collision.lastTangentImpulse = newImpulse;

				Vec2 tangentImpulse = collision.tangent * tangentImpulseDelta;

				Vec2 impulse = normalImpulse + tangentImpulse;
				if (collision.polyA->GetMass() != 0)
				{
					/*collision.polyA->speed += collision.normal * (normalImpulse * invMassA);
					collision.polyA->angularVelocity += aAngSpeed.z;*/
					ApplyImpulse(collision.polyA, collision.point, impulse);
				}
				if (collision.polyB->GetMass() != 0)
				{
					/*collision.polyB->speed -= collision.normal * (normalImpulse * invMassB);
					collision.polyB->angularVelocity -= bAngSpeed.z;*/
					ApplyImpulse(collision.polyB, collision.point, impulse * -1.0f);
				}
				//}
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

		Vec3 collisionPoint = Vec3(collision.point.x, collision.point.y, 0.0f);
		Vec3 polyAPosition = Vec3(collision.polyA->position.x, collision.polyA->position.y, 0.0f);
		Vec3 polyBPosition = Vec3(collision.polyB->position.x, collision.polyB->position.y, 0.0f);

		float invMassA = collision.polyA->GetMass();
		float invMassB = collision.polyB->GetMass();

		Vec3 rAi = collisionPoint - polyAPosition;
		Vec3 rBi = collisionPoint - polyBPosition;

		Vec3 vA = Vec3(collision.polyA->speed.x, collision.polyA->speed.y, 0.0f);
		Vec3 vB = Vec3(collision.polyB->speed.x, collision.polyB->speed.y, 0.0f);

		Mat2 matRotA;
		matRotA.SetAngle(collision.polyA->angularVelocity);

		Mat2 matRotB;
		matRotB.SetAngle(collision.polyB->angularVelocity);

		Vec3 wA = Vec3(matRotA.X.x, matRotA.Y.y, 0.0f);
		Vec3 wB = Vec3(matRotB.X.x, matRotB.Y.y, 0.0f);

		Vec3 vAi = vA + wA ^ rAi;
		Vec3 vBi = vB + wB ^ rBi;

		float invLocalIA = collision.polyA->GetInertiaTensor();
		float invLocalIB = collision.polyB->GetInertiaTensor();

			Mat2 invWorldIA = collision.polyA->rotation * invLocalIA * collision.polyA->rotation.GetInverse();
			Mat2 invWorldIB = collision.polyB->rotation * invLocalIB * collision.polyB->rotation.GetInverse();

			Vec3 normal = Vec3(collision.normal.x, collision.normal.y, 0.0f);

			Vec3 momentumA = invWorldIA * (rAi ^ normal);
			Vec3 momentumB = invWorldIB * (rBi ^ normal);

			float rotA = (momentumA ^ rAi) | normal;
			float rotB = (momentumB ^ rBi) | normal;

			// Apply position correction.
			//float damping = 0.2f;
			//float correction = (collision.distance * damping) / (invMassA + invMassB);
			/*if (collision.polyA->GetMass() != 0)
				collision.polyA->AddPosition(collision.normal * correction * invMassA);
			if (collision.polyB->GetMass() != 0)
				collision.polyB->AddPosition(collision.normal * correction * -invMassB);*/

			// Apply position correction.
			float damping = 0.2f;
			// Limit the amount of correction at once for stability
			float maxCorrection = 5.0f;
			float steeringForce = Clamp(damping * collision.distance, 0.0f, maxCorrection);
			Vec3 impulse2 = collision.normal * (steeringForce / (invMassA + invMassB + rotA + rotB));
			if (collision.polyA->GetMass() != 0)
			{
				collision.polyA->AddPosition(impulse2 * invMassA);
				//collision.polyA->rotation.Rotate((invWorldIA * (rAi ^ impulse2)).z);
				collision.polyA->rotation.Rotate((wA + invWorldIA * (rAi ^ impulse2)).z);
				collision.polyA->SetRotation(collision.polyA->rotation);
				//bodyA.xf.rotation -= point.aToContact.cross(impulse2) * bodyA.inverseInertia;
			}

			if (collision.polyB->GetMass() != 0)
			{
				collision.polyB->AddPosition(impulse2 * invMassB * -1.0f);
				collision.polyB->rotation.Rotate((wB + invWorldIB * (rBi ^ impulse2)).z);
				collision.polyB->SetRotation(collision.polyB->rotation);
				//bodyB.xf.rotation += point.bToContact.cross(impulse2) * bodyB.inverseInertia;
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

	inline void ApplyImpulse(CPolygonPtr object, Vec3 collisionPoint, Vec2 impulse) 
	{
		if (object->GetMass() <= 0)
			return;

		Mat2 matRotA;
		matRotA.SetAngle(object->angularVelocity);
		Vec3 wA = Vec3(matRotA.X.x, matRotA.Y.y, 0.0f);
		Vec3 rAi = collisionPoint - Vec3(object->position);
		float invLocalIA = object->GetInertiaTensor();
		Mat2 invWorldIA = object->rotation * invLocalIA * object->rotation.GetInverse();
		Vec3 aAngSpeed = wA + invWorldIA * (rAi ^ impulse);
		object->speed += impulse * object->GetMass();
		object->angularVelocity += aAngSpeed.z /* object->GetInertiaTensor()*/;
	}

};

#endif