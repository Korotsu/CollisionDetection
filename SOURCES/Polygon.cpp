#include "Polygon.h"
#include <GL/glu.h>


#include "PhysicEngine.h"
#include "GlobalVariables.h"

#define	MAXITERATION 100

CPolygon::CPolygon(size_t index)
	: CGLObject(), m_index(index), density(0.1f), aabb(new CAABB(points, position, rotation))
{}

CPolygon::~CPolygon()
{
	delete aabb;
}

void CPolygon::Build()
{
	m_lines.clear();

	CreateBuffers();
	BuildLines();
	aabb->ApplyRotation(points, rotation);
}

void CPolygon::Draw()
{
	// Set transforms (assuming model view mode is set)
	float transfMat[16] = { rotation.X.x, rotation.X.y, 0.0f, 0.0f,
							rotation.Y.x, rotation.Y.y, 0.0f, 0.0f,
							0.0f, 0.0f, 0.0f, 1.0f,
							position.x, position.y, -1.0f, 1.0f };
	glPushMatrix();
	glMultMatrixf(transfMat);

	// Draw vertices
	BindBuffers();
	if (isOverlaping)
	{
		glColor3f(0, 1, 0);
		//isOverlaping = false;
	}
	else
		glColor3f(0, 0, 1);
	glDrawArrays(GL_LINE_LOOP, 0, points.size());
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();

	if (gVars->bDebugElem)
		aabb->Draw();
}

size_t	CPolygon::GetIndex() const
{
	return m_index;
}

Vec2	CPolygon::TransformPoint(const Vec2& point) const
{
	return position + rotation * point;
}

Vec2	CPolygon::InverseTransformPoint(const Vec2& point) const
{
	return rotation.GetInverse() * (point - position);
}

bool	CPolygon::IsPointInside(const Vec2& point) const
{
	float maxDist = -FLT_MAX;

	for (const Line& line : m_lines)
	{
		Line globalLine = line.Transform(rotation, position);
		float pointDist = globalLine.GetPointDist(point);
		maxDist = Max(maxDist, pointDist);
	}

	return maxDist <= 0.0f;
}

bool	CPolygon::CheckCollision(const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist) const
{
	std::vector<Vec2> simplexPoints;
	Vec2 dir = Vec2(1, 0);
	simplexPoints.push_back(poly.Support(dir) - Support(dir * -1));
	dir = (simplexPoints.back() * -1).Normalized();
	simplexPoints.push_back(poly.Support(dir) - Support(dir * -1));
	size_t size = 0;
	Vec2 A = Vec2(0, 0);
	Vec2 B = Vec2(0, 0);
	Vec2 AB = Vec2(0, 0);
	float angle = 0.f;
	while (size < MAXITERATION)
	{
		size = simplexPoints.size();
		// Get the orthogonal direction toward origin.
		A = simplexPoints[size - 2];
		B = simplexPoints[size - 1];
		AB = B - A;
		if (AB.GetLength() * A.GetLength() > 0)
			angle = 90 - (1 - (AB | (A * -1)) / (AB.GetLength() * A.GetLength())) * 90;
		else
			angle = 0;
		dir.x = -1 * (A.x * cos(angle) - A.y * sin(angle));
		dir.y = -1 * (A.x * sin(angle) + A.y * cos(angle));
		dir.Normalize();
		simplexPoints.push_back(poly.Support(dir) - Support(dir * -1));
		if (Triangle::IsPointInside(Vec2(0, 0), A, B, simplexPoints.back()))
			return true;

		else if (size >= 4 && Triangle::IsPointInside(simplexPoints.back(), A, B, simplexPoints[size - 4]))
			return false;

	}
	return false;
}

void CPolygon::BuildLines()
{
	for (size_t index = 0; index < points.size(); ++index)
	{
		const Vec2 pointA = points[index];
		const Vec2 pointB = points[(index + 1) % points.size()];

		Vec2 lineDir = (pointA - pointB).Normalized();

		m_lines.push_back(Line(pointB, lineDir));
	}
}