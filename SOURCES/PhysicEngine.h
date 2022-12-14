#ifndef _PHYSIC_ENGINE_H_
#define _PHYSIC_ENGINE_H_

#include <vector>
#include <unordered_map>
#include "Maths.h"
#include "Polygon.h"

class IBroadPhase;

struct SPolygonPair
{
	SPolygonPair(CPolygonPtr _polyA, CPolygonPtr _polyB) : polyA(_polyA), polyB(_polyB) {}

	CPolygonPtr	polyA;
	CPolygonPtr	polyB;
};

struct SCollision
{
	SCollision() = default;
	SCollision(CPolygonPtr _polyA, CPolygonPtr _polyB, Vec2	_point, Vec2 _normal, float _distance)
		: polyA(_polyA), polyB(_polyB), point(_point), normal(_normal), distance(_distance) {}

	CPolygonPtr	polyA, polyB;

	Vec2	point;
	Vec2	normal;
	float	distance;
};

class CPhysicEngine
{
public:
	void	Reset();
	void	Activate(bool active);

	void	DetectCollisions();

	void	Step(float deltaTime);

	template<typename TFunctor>
	void	ForEachCollision(TFunctor functor)
	{
		for (const SCollision& collision : m_collidingPairs)
		{
			functor(collision);
		}
	}
	void						CollisionBroadPhase();

private:
	friend class CPenetrationVelocitySolver;

	void						CollisionNarrowPhase();

	bool						m_active = true;

	// Collision detection
	IBroadPhase* m_broadPhase;
	std::vector<SPolygonPair>	m_pairsToCheck;
	std::vector<SCollision>		m_collidingPairs;
public:
	const std::vector<SPolygonPair> GetBroadPhaseResultPaired() const { return m_pairsToCheck; };
	const std::vector<CPolygon> GetBroadPhaseResult() const
	{
		std::vector<CPolygon> result;
		for (SPolygonPair pair : m_pairsToCheck)
		{
			result.push_back(*pair.polyA.get());
			result.push_back(*pair.polyB.get());
		}

		return result;
	};

	const bool IsInBroadPhaseResult(const CPolygon* poly) const
	{
		for (SPolygonPair pair : m_pairsToCheck)
		{
			if (pair.polyA.get() == poly || pair.polyB.get() == poly)
				return true;
		}

		return false;
	};
};

#endif