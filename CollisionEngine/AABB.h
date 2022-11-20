#ifndef _AABB_H_
#define _AABB_H_

#include "GLObject.h"

class CAABB : public CGLObject
{
private:
	Vec2 min;
	Vec2 max;

public:
	CAABB(const std::vector<Vec2>& points, const Vec2& inPosition, const Mat2& inRotation);
	~CAABB() = default;

	bool DoesOverlap(CAABB& testedAABB);
	void ApplyRotation(const Mat2& rotation);
	void ComputePoints(const std::vector<Vec2>& inPoints, const Mat2& inRotation);
	void Draw();
	Vec2 position;
	Mat2 rotation;
	bool isOverlaping = false;
};
#endif // _AABB_H_