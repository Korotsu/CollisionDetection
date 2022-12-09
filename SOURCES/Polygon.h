#ifndef _POLYGON_H_
#define _POLYGON_H_

#include <vector>
#include <memory>

#include "GLObject.h"
#include "Maths.h"
#include "AABB.h"

class CPolygon : public CGLObject
{
private:
	friend class CWorld;

	CPolygon(size_t index);
public:
	~CPolygon();

	Vec2				position;
	Mat2				rotation;

	inline void SetPosition(const Vec2& inPosition)
	{
		position = inPosition;
		aabb->position = inPosition;
	}

	inline void AddPosition(const Vec2& inPosition)
	{
		position += inPosition;
		aabb->position += inPosition;
	}

	inline void SetRotation(const Mat2& inRotation)
	{
		rotation = inRotation;
		aabb->ApplyRotation(points, inRotation);
	}

	void				Build();
	void				Draw();
	size_t				GetIndex() const;
	Vec2				TransformPoint(const Vec2& point) const;
	Vec2				InverseTransformPoint(const Vec2& point) const;

	// if point is outside then returned distance is negative (and doesn't make sense)
	bool				IsPointInside(const Vec2& point) const;

	bool				CheckCollision(const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist, std::vector<Vec2>& simplexPoints = std::vector<Vec2>(), std::vector<Vec2>& outA = std::vector<Vec2>(), std::vector<Vec2>& outB = std::vector<Vec2>()) const;

	// Physics
	float				density;
	Vec2				speed;
	CAABB* aabb;
	bool isOverlaping = false;

	inline const Vec2 Support(const Vec2& dir) const
	{
		Vec2 support;
		float maxProjection = -FLT_MAX;
		for (Vec2 vertex : points)
		{
			vertex = rotation * vertex + position;
			float projection = vertex | dir;
			if (projection > maxProjection)
			{
				maxProjection = projection;
				support = vertex;
			}
		}
		return support;
	}

	inline const std::vector<Vec2> MinkovskiDiff(const CPolygon& poly, std::vector<Vec2>& outABase, std::vector<Vec2>& outBBase) const
	{
		Vec2 dir = Vec2(0, 0);
		std::vector<Vec2> result;
		for (float angle = 0; angle < 360; angle += 0.1)
		{
			dir.x = cos(angle);
			dir.y = sin(angle);
			Vec2 A = Support(dir * -1);
			Vec2 B = poly.Support(dir);
			outABase.push_back(A);
			outBBase.push_back(B);
			result.push_back(B - A);
		}
		return result;
	}

private:
	void				BuildLines();
	size_t				m_index;
	std::vector<Line>	m_lines;
};

typedef std::shared_ptr<CPolygon>	CPolygonPtr;

#endif