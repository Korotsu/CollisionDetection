#ifndef _POLYGON_H_
#define _POLYGON_H_

#include <vector>
#include <memory>

#include "GLObject.h"
#include "Maths.h"
#include "AABB.h"

struct SCollision;

class CPolygon : public CGLObject
{
private:
	friend class CWorld;

	CPolygon(size_t index);
public:
	~CPolygon();

	Vec2				position;
	Mat2				rotation;
	float				bounciness = 0.9f;
	float				friction = 0.5f;

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

	float				GetArea() const;

	Vec2				TransformPoint(const Vec2& point) const;
	Vec2				InverseTransformPoint(const Vec2& point) const;

	// if point is outside then returned distance is negative (and doesn't make sense)
	bool				IsPointInside(const Vec2& point) const;

	bool				CheckCollision(CPolygon& poly, SCollision& collisionInfo);
	bool				CheckCollisionDebug(CPolygon& poly, SCollision& collisionInfo, Vec2& otherResult = Vec2(), std::vector<Vec2>& outSimplex = std::vector<Vec2>());
	bool				GJK(const CPolygon& poly, std::vector<Vec2>& outSimplex) const;
	void				EPA(std::vector<Vec2>& polytope, CPolygon& poly, SCollision& collisionInfo);
	void				EPADebug(std::vector<Vec2>& polytope, CPolygon& poly, SCollision& collisionInfo, Vec2& otherResult);
	// If line intersect polygon, colDist is the penetration distance, and colPoint most penetrating point of poly inside the line
	bool				IsLineIntersectingPolygon(const Line& line, Vec2& colPoint, float& colDist) const;
	float				GetMass() const;
	float				GetInertiaTensor() const;
	float				GetInversedInertiaTensor() const;

	Vec2				GetPointVelocity(const Vec2& point) const;
	// Physics
	float				density;
	Vec2				speed;
	CAABB* aabb;
	bool isOverlaping = false;
	float				angularVelocity = 0.0f;
	Vec2				forces;
	float				torques = 0.0f;

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
		Vec2 dir = Vec2(0.0f, 0.0f);
		std::vector<Vec2> result;
		for (float angle = 0.0f; angle < 360.0f; angle += 0.1f)
		{
			dir.x = cosf(angle);
			dir.y = sinf(angle);
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

	void				ComputeArea();
	void				RecenterOnCenterOfMass(); // Area must be computed
	void				ComputeLocalInertiaTensor(); // Must be centered on center of mass
	size_t				m_index;

	std::vector<Line>	m_lines;

	float				m_signedArea;

	// Physics
	float				m_localInertiaTensor; // don't consider mass
};

typedef std::shared_ptr<CPolygon>	CPolygonPtr;

#endif