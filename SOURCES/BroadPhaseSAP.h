#ifndef _BROAD_PHASE_SAP_H_
#define _BROAD_PHASE_SAP_H_

#include "BroadPhase.h"

#include "Polygon.h"
#include "GlobalVariables.h"
#include "World.h"
#include <algorithm>

class CBroadPhaseSAP : public IBroadPhase
{
public:
	inline static int compare(const void* a, const void* b)
	{
		const float A = static_cast<const CPolygonPtr*>(a)->get()->aabb->GetMinX();
		const float B = static_cast<const CPolygonPtr*>(b)->get()->aabb->GetMinX();
		if (A < B)
			return -1;
		return 1;
	}

	inline static int compare2(const CPolygonPtr a, const CPolygonPtr b)
	{
		const float A = a.get()->aabb->GetMinX();
		const float B = b.get()->aabb->GetMinX();
		if (A < B)
			return -1;
		return 1;
	}

	virtual void GetCollidingPairsToCheck(std::vector<SPolygonPair>& pairsToCheck) override
	{
		if (sortedList.empty())
			sortedList = gVars->pWorld->GetPolygons();
		std::qsort(sortedList.data(), sortedList.size(), sizeof(CPolygonPtr), compare);
		//std::sort(sortedList.begin(), sortedList.end(), compare2); other sort for benchmark

		size_t sortedListSize = sortedList.size();
		for (size_t i = 0; i < sortedListSize; i++)
		{
			CAABB* a = sortedList[i].get()->aabb;
			for (size_t j = i + 1; j < sortedListSize; j++)
			{
				CAABB* b = sortedList[j].get()->aabb;
				const float AMaxX = a->GetMaxX();
				const float BMinX = b->GetMinX();
				if (AMaxX < BMinX)
					break;
				if (a->DoesOtherAxisOverlap(*b))
				{
					a->isOverlaping = true;
					b->isOverlaping = true;
					pairsToCheck.push_back(SPolygonPair(sortedList[i], sortedList[j]));
				}
			}
		}
	}
};

#endif
