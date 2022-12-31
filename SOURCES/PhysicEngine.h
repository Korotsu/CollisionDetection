#ifndef _PHYSIC_ENGINE_H_
#define _PHYSIC_ENGINE_H_

#include <vector>
#include <unordered_map>
#include "Maths.h"
#include "Polygon.h"
#include "Collision.h"

class IBroadPhase;

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
		for (SCollision& collision : m_collidingPairs)
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