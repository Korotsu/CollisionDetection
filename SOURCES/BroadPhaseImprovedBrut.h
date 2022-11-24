#ifndef _BROAD_PHASE_IMPROVED_BRUT_H_
#define _BROAD_PHASE_IMPROVED_BRUT_H_

#include "BroadPhase.h"

#include "Polygon.h"
#include "GlobalVariables.h"
#include "World.h"

class CBroadPhaseImprovedBrut : public IBroadPhase
{
public:
	virtual void GetCollidingPairsToCheck(std::vector<SPolygonPair>& pairsToCheck) override
	{
		size_t poly_count = gVars->pWorld->GetPolygonCount();
		for (size_t i = 0; i < poly_count; ++i) // old brute test version
		{
			CPolygonPtr APoly = gVars->pWorld->GetPolygon(i);
			CAABB* A = APoly.get()->GetAABB();
			for (size_t j = i + 1; j < poly_count; ++j)
			{
				CPolygonPtr BPoly = gVars->pWorld->GetPolygon(j);
				CAABB* B = BPoly.get()->GetAABB();
				if (A->DoesOverlap(*B))
					pairsToCheck.push_back(SPolygonPair(APoly, BPoly));
			}
		}
	}
};

#endif