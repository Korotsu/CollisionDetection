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
		GetAABB()->position = inPosition;
	}

	inline void AddPosition(const Vec2& inPosition)
	{
		position += inPosition;
		GetAABB()->position += inPosition;
	}

	inline void SetRotation(const Mat2& inRotation)
	{
		rotation = inRotation;
		GetAABB()->ApplyRotation(inRotation);
	}

	void				Build();
	void				Draw();
	size_t				GetIndex() const;

	CAABB* GetAABB() {
		if (m_aabb == nullptr)
			m_aabb = new CAABB(points, position, rotation);
		//m_aabb->position = position;
		/*if (m_aabb->rotation != rotation)
			m_aabb->ApplyRotation(rotation);*/
		return m_aabb;
	};

	Vec2				TransformPoint(const Vec2& point) const;
	Vec2				InverseTransformPoint(const Vec2& point) const;

	// if point is outside then returned distance is negative (and doesn't make sense)
	bool				IsPointInside(const Vec2& point) const;

	bool				CheckCollision(const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist) const;

	// Physics
	float				density;
	Vec2				speed;

private:
	void				BuildLines();
	size_t				m_index;

	CAABB* m_aabb;

	std::vector<Line>	m_lines;
};

typedef std::shared_ptr<CPolygon>	CPolygonPtr;

#endif