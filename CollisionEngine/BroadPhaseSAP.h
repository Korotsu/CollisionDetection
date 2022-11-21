#ifndef _BROAD_PHASE_SAP_H_
#define _BROAD_PHASE_SAP_H_

#include "BroadPhase.h"

#include "Polygon.h"
#include "GlobalVariables.h"
#include "World.h"

class CBroadPhaseSAP : public IBroadPhase
{
public:
	int compare(const void* a, const void* b)
	{
		if (static_cast<const CPolygonPtr*>(a)->get()->GetAABB()->GetMinX() < static_cast<const CPolygonPtr*>(b)->get()->GetAABB()->GetMinX())
			return -1;
		return 1;
	}

	virtual void GetCollidingPairsToCheck(std::vector<SPolygonPair>& pairsToCheck) override
	{
		if(sortedList.empty())
			sortedList = gVars->pWorld->GetPolygons();
		size_t poly_count = gVars->pWorld->GetPolygonCount();
		std::vector<CPolygonPtr> polyList = gVars->pWorld->GetPolygons();

		qsort(sortedList.data(), sortedList.size(), sizeof(CPolygonPtr), compare);
		std::vector<CPolygonPtr> sortedList;
		for (CPolygonPtr poly : polyList)
		{
			CAABB* aabb = poly.get()->GetAABB();
			for (size_t i = 0; i < sortedList.size(); i++)
			{
				if (aabb->GetMinX() <= sortedList[i].get()->GetAABB()->GetMinX())
				{
					sortedList.
					sortedList.insert(i, poly);
					break;
				}
			}
		}
		for (size_t i = 0; i < poly_count; ++i)
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
