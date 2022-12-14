#include "Polygon.h"
#include <GL/glu.h>


#include "PhysicEngine.h"
#include "GlobalVariables.h"

#define	MAXITERATION 10000

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
	Vec2 dir = Vec2(1, 0);
	Vec2 A = poly.Support(dir) - Support(dir * -1);
	dir = (A * -1).Normalized();
	Vec2 B = poly.Support(dir) - Support(dir * -1);
	Vec2 AB = Vec2(0, 0);
	Vec2 C = Vec2(0, 0);
	Vec2 oldC = Vec2(0, 0);
	float angle = 0.f;
	for (size_t i = 0; i < MAXITERATION; i++)
	{
		if (C.GetSqrLength() != 0)
		{
			if (C.GetSqrLength() > A.GetSqrLength() && C.GetSqrLength() > B.GetSqrLength())
				return false;
			if (A.GetSqrLength() > B.GetSqrLength())
				A = C;
			else
				B = C;
		}
		if (A == B)
			return false;
		AB = B - A;
		dir = Vec2(-1 * AB.y, AB.x).Normalized();
		angle = Clamp(B.Normalized() | dir.Normalized(), -1.0f, 1.0f);
		dir = angle <= 0 ? dir : Vec2(AB.y, -1 * AB.x).Normalized();
		oldC = C;
		C = poly.Support(dir) - Support(dir * -1);
		if (Triangle::IsPointInside(Vec2(0, 0), A, B, C))
			return true;

		else if (C.GetSqrLength() != 0 && Triangle::IsPointInside(C, A, B, oldC))
			return false;

	}
	return false;
}

bool CPolygon::CheckCollisionWithDebug(const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist, std::vector<Vec2>& simplexPoints, std::vector<Vec2>& outA, std::vector<Vec2>& outB) const
{
	Vec2 dir = Vec2(1, 0);
	Vec2 outAV = Support(dir * -1) * -1;
	Vec2 outBV = poly.Support(dir);
	Vec2 A = outBV + outAV;
	simplexPoints.push_back(A);
	outA.push_back(outAV);
	outB.push_back(outBV);
	dir = (A * -1).Normalized();
	outAV = Support(dir * -1) * -1;
	outBV = poly.Support(dir);
	Vec2 B = outBV + outAV;
	simplexPoints.push_back(B);
	outA.push_back(outAV);
	outB.push_back(outBV);
	Vec2 AB = Vec2(0, 0);
	Vec2 C = Vec2(0, 0);
	Vec2 oldC = Vec2(0, 0);
	float angle = 0.f;
	for (size_t i = 0; i < MAXITERATION; i++)
	{
		if (C.GetSqrLength() != 0)
		{
			if (C.GetSqrLength() > A.GetSqrLength() && C.GetSqrLength() > B.GetSqrLength())
				return false;
			if (A.GetSqrLength() > B.GetSqrLength())
				A = C;
			else
				B = C;
		}
		if (A == B)
			return false;
		AB = B - A;
		dir = Vec2(-1 * AB.y, AB.x).Normalized();
		angle = Clamp(B.Normalized() | dir.Normalized(), -1.0f, 1.0f);
		dir = angle <= 0 ? dir : Vec2(AB.y, -1 * AB.x).Normalized();
		oldC = C;
		outAV = Support(dir * -1) * -1;
		outBV = poly.Support(dir);
		C = outBV + outAV;
		simplexPoints.push_back(C);
		outA.push_back(outAV);
		outB.push_back(outBV);
		if (Triangle::IsPointInside(Vec2(0, 0), A, B, C))
			return true;

		else if (C.GetSqrLength() != 0 && Triangle::IsPointInside(C, A, B, oldC))
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