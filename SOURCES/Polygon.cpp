#include "Polygon.h"
#include <GL/glu.h>


#include "PhysicEngine.h"
#include "GlobalVariables.h"

#define	MAXITERATION 1000

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
		glColor3f(0, 1, 0);
	else
		glColor3f(0, 0, 1);
	glDrawArrays(GL_LINE_LOOP, 0, points.size());
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();

	if (gVars->bDebugElem && gVars->bToggleAABB)
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

bool	CPolygon::CheckCollision(const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist, std::vector<Vec2>& outSimplex) const
{
	if (GJK(poly, outSimplex))
	{
		EPA(std::vector<Vec2>(outSimplex), poly, colPoint, colNormal, colDist);
		return true;
	}
	return false;
}

bool CPolygon::GJK(const CPolygon& poly, std::vector<Vec2>& outSimplex) const
{
	Vec2 dir = Vec2(1.0f, 0.0f);

	Vec2 A = poly.Support(dir) - Support(dir * -1.0f);
	Vec2 ANormalized = A.Normalized();

	dir = ANormalized * -1.0f;

	Vec2 B = poly.Support(dir) - Support(dir * -1.0f);
	Vec2 BNormalized = B.Normalized();

	Vec2 ABNormalized = Vec2(0.0f, 0.0f);

	Vec2 C = Vec2(0.0f, 0.0f);
	Vec2 CNormalized = C;
	Vec2 oldC = C;

	float angle = 0.0f;
	float ACangle = 0.0f;
	float BCangle = 0.0f;

	for (size_t i = 0; i < MAXITERATION; i++)
	{
		if (C.GetSqrLength() != 0)
		{
			if (C.GetSqrLength() > A.GetSqrLength() && C.GetSqrLength() > B.GetSqrLength())
				return false;

			ACangle = Clamp(ANormalized | CNormalized, -1.0f, 1.0f);
			BCangle = Clamp(BNormalized | CNormalized, -1.0f, 1.0f);
			if (ACangle > BCangle)
			{
				oldC = A;
				A = C;
				ANormalized = CNormalized;
			}
			else
			{
				oldC = B;
				B = C;
				BNormalized = CNormalized;
			}
		}

		if (A == B)
			return false;

		ABNormalized = BNormalized - ANormalized;
		dir = Vec2(-1.0f * ABNormalized.y, ABNormalized.x);

		angle = Clamp(BNormalized | dir, -1.0f, 1.0f);
		dir = angle <= 0 ? dir : (dir * -1.0f);

		C = poly.Support(dir) - Support(dir * -1.0f);
		CNormalized = C.Normalized();

		if (Triangle::IsPointInside(Vec2(0, 0), A, B, C))
		{
			outSimplex.push_back(A);
			outSimplex.push_back(B);
			outSimplex.push_back(C);
			return true;
		}

		else if (C.GetSqrLength() != 0 && Triangle::IsPointInside(C, A, B, oldC))
			return false;

	}
	return false;
}

void CPolygon::EPA(std::vector<Vec2>& polytope, const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist) const
{
	Vec2 A = Vec2(0, 0);
	Vec2 B = Vec2(0, 0);
	Vec2 C = Vec2(0, 0);
	Vec2 AB = Vec2(0, 0);
	Vec2 normal = Vec2(0, 0);
	Vec2 minNormal = Vec2(0, 0);
	float distance = 0.0f;
	float minDistance = FLT_MAX;
	float newDistance = 0.0f;
	size_t minIndex = 0;
	for (size_t limit = 0; limit < MAXITERATION; limit++)
	{
		minDistance = FLT_MAX;
		for (size_t i = 0; i < polytope.size(); i++)
		{
			A = polytope[i];
			B = ((i + 1) < polytope.size()) ? polytope[i + 1] : polytope[0];
			AB = B - A;
			normal = Vec2(AB.y, -1 * AB.x).Normalized();
			distance = normal | A;

			if (distance < 0)
			{
				distance *= -1;
				normal *= -1;
			}
			if (distance < minDistance)
			{
				minDistance = distance;
				minIndex = i + 1;
				minNormal = normal;
			}
		}
		C = poly.Support(minNormal) - Support(minNormal * -1);
		newDistance = minNormal | C;
		if (fabs(newDistance - minDistance) < EPSILON)
		{
			colDist = minDistance + EPSILON;
			Vec2 t1 = poly.Support(minNormal);
			Vec2 t2 = Support(minNormal * -1);
			float AT1L = (t1 - position).GetLength();
			float T1BL = (poly.position - t1).GetLength();
			float AT2L = (t2 - position).GetLength();
			float T2BL = (poly.position - t2).GetLength();

			bool testResult = (AT1L + T1BL < AT2L + T2BL);
			if (testResult)
			{
				colPoint = t1;
				colNormal = minNormal * -1;
			}
			else
			{
				colPoint = t2;
				colNormal = minNormal;
			}
			return;
		}
		polytope.insert(polytope.begin() + minIndex, C);
	}
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