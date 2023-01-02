#include "Polygon.h"
#include <GL/glu.h>

#include "InertiaTensor.h"

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

	ComputeArea();
	RecenterOnCenterOfMass();
	ComputeLocalInertiaTensor();

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

float	CPolygon::GetArea() const
{
	return fabsf(m_signedArea);
}

Vec2	CPolygon::TransformPoint(const Vec2& point) const
{
	return position + rotation * point;
}

Vec2	CPolygon::InverseTransformPoint(const Vec2& point) const
{
	return rotation.GetInverseOrtho() * (point - position);
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

bool	CPolygon::IsLineIntersectingPolygon(const Line& line, Vec2& colPoint, float& colDist) const
{
	//float dist = 0.0f;
	float minDist = FLT_MAX;
	Vec2 minPoint;
	float lastDist = 0.0f;
	bool intersecting = false;

	for (const Vec2& point : points)
	{
		Vec2 globalPoint = TransformPoint(point);
		float dist = line.GetPointDist(globalPoint);
		if (dist < minDist)
		{
			minPoint = globalPoint;
			minDist = dist;
		}

		intersecting = intersecting || (dist != 0.0f && lastDist * dist < 0.0f);
		lastDist = dist;
	}

	if (minDist <= 0.0f)
	{
		colDist = -minDist;
		colPoint = minPoint;
	}
	return (minDist <= 0.0f);
}

bool	CPolygon::CheckCollision(CPolygon& poly, SCollision& collisionInfo)
{
	std::vector<Vec2> outSimplex;
	if (GJK(poly, outSimplex))
	{
		EPA(std::vector<Vec2>(outSimplex), poly, collisionInfo);
		return true;
	}
	return false;
}

bool	CPolygon::CheckCollisionDebug(CPolygon& poly, SCollision& collisionInfo, Vec2& otherResult, std::vector<Vec2>& outSimplex)
{
	if (GJK(poly, outSimplex))
	{
		if (gVars->bToggleEPADebug)
			EPADebug(std::vector<Vec2>(outSimplex), poly, collisionInfo, otherResult);
		else
			EPA(std::vector<Vec2>(outSimplex), poly, collisionInfo);
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

	Vec2 AB = Vec2(0.0f, 0.0f);

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

		AB = B - A;
		dir = Vec2(-1.0f * AB.y, AB.x);

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

void CPolygon::EPA(std::vector<Vec2>& polytope, CPolygon& poly, SCollision& collisionInfo)
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
			collisionInfo.distance = minDistance + EPSILON;

			//PolyA:
			Vec2 pt1 = Support(minNormal * -1);
			Vec2 t1 = pt1 + minNormal * (minDistance - EPSILON);
			float AT1L = (t1 - position).GetLength();
			float T1BL = (poly.position - t1).GetLength();
			
			//PolyB:
			Vec2 pt2 = poly.Support(minNormal);
			Vec2 t2 = pt2 - minNormal * (minDistance - EPSILON);
			float AT2L = (t2 - position).GetLength();
			float T2BL = (poly.position - t2).GetLength();

			bool testResult = ((AT1L + T1BL) - (AT2L + T2BL) < EPSILON);
			bool t1In = poly.IsPointInside(t1);
			bool t2In = IsPointInside(t2);

			if (t1In && t2In)
			{
				if (testResult)
				{
					collisionInfo.point = pt1;
					collisionInfo.normal = minNormal * -1;
					collisionInfo.polyA.swap(collisionInfo.polyB);
				}
				else
				{
					collisionInfo.point = pt2;
					collisionInfo.normal = minNormal;

				}
			}
			else if (t1In)
			{
				collisionInfo.point = pt1;
				collisionInfo.normal = minNormal * -1;
				collisionInfo.polyA.swap(collisionInfo.polyB);
			}
			else if (t2In)
			{
				collisionInfo.point = pt2;
				collisionInfo.normal = minNormal;
			}
			else
			{
				if (testResult)
				{
					collisionInfo.point = pt1;
					collisionInfo.normal = minNormal * -1;
					collisionInfo.polyA.swap(collisionInfo.polyB);
				}
				else
				{
					collisionInfo.point = pt2;
					collisionInfo.normal = minNormal;

				}
			}

			Vec2 rAi = collisionInfo.point - collisionInfo.polyA->position;
			Vec2 rBi = collisionInfo.point - collisionInfo.polyB->position;
			
			collisionInfo.baseSeparation = collisionInfo.distance + rAi.GetLength() + rBi.GetLength();
			
			Vec2 velA = collisionInfo.polyA->speed + (rAi ^ collisionInfo.polyA->angularVelocity);
			Vec2 velB = collisionInfo.polyB->speed + (rBi ^ collisionInfo.polyB->angularVelocity);
			
			collisionInfo.relativeVelocity = (velA - velB);
			collisionInfo.tangent = Vec3::GetTangent(collisionInfo.relativeVelocity, collisionInfo.normal);

			return;
		}
		polytope.insert(polytope.begin() + minIndex, C);
	}
}

void CPolygon::EPADebug(std::vector<Vec2>& polytope, CPolygon& poly, SCollision& collisionInfo, Vec2& otherResult)
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
			collisionInfo.distance = minDistance + EPSILON;

			//PolyA:
			Vec2 pt1 = Support(minNormal * -1);
			Vec2 t1 = pt1 + minNormal * (minDistance - EPSILON);
			float AT1L = (t1 - position).GetLength();
			float T1BL = (poly.position - t1).GetLength();

			//PolyB:
			Vec2 pt2 = poly.Support(minNormal);
			Vec2 t2 = pt2 + (minNormal * -1) * (minDistance - EPSILON);
			float AT2L = (t2 - position).GetLength();
			float T2BL = (poly.position - t2).GetLength();

			bool testResult = ((AT1L + T1BL) - (AT2L + T2BL) < EPSILON);
			bool t1In = poly.IsPointInside(t1);
			bool t2In = IsPointInside(t2);

			if (t1In && t2In)
			{
				if (testResult)
				{
					collisionInfo.point = pt1;
					collisionInfo.polyA.swap(collisionInfo.polyB);
					collisionInfo.normal = minNormal * -1;
					otherResult = pt2;
				}
				else
				{
					collisionInfo.point = pt2;
					collisionInfo.normal = minNormal;
					otherResult = pt1;
				}
			}
			else if (t1In)
			{
				collisionInfo.point = pt1;
				collisionInfo.polyA.swap(collisionInfo.polyB);
				collisionInfo.normal = minNormal * -1;
				otherResult = pt2;
			}
			else if (t2In)
			{
				collisionInfo.point = pt2;
				collisionInfo.normal = minNormal;
				otherResult = pt1;
			}
			else
			{
				if (testResult)
				{
					collisionInfo.point = pt1;
					collisionInfo.polyA.swap(collisionInfo.polyB);
					collisionInfo.normal = minNormal * -1;
					otherResult = pt2;
				}
				else
				{
					collisionInfo.point = pt2;
					collisionInfo.normal = minNormal;
					otherResult = pt1;
				}
			}
			Vec2 rAi = collisionInfo.point - collisionInfo.polyA->position;
			Vec2 rBi = collisionInfo.point - collisionInfo.polyB->position;

			collisionInfo.baseSeparation = collisionInfo.distance + rAi.GetLength() + rBi.GetLength();

			Vec2 velA = collisionInfo.polyA->speed + (rAi ^ collisionInfo.polyA->angularVelocity);
			Vec2 velB = collisionInfo.polyB->speed + (rBi ^ collisionInfo.polyB->angularVelocity);

			collisionInfo.relativeVelocity = (velA - velB);
			collisionInfo.tangent = Vec3::GetTangent(collisionInfo.relativeVelocity, collisionInfo.normal);
			return;
		}
		polytope.insert(polytope.begin() + minIndex, C);
	}
}

float CPolygon::GetMass() const
{
	return density * GetArea();
}

float CPolygon::GetInertiaTensor() const
{
	return /*m_localInertiaTensor */ GetMass();
}

float CPolygon::GetInversedInertiaTensor() const
{
	float result = GetInertiaTensor();
	return (result) ? (1/result) : FLT_MAX;
}

Vec2 CPolygon::GetPointVelocity(const Vec2& point) const
{
	return speed + (point - position).GetNormal() * angularVelocity;
}

void CPolygon::BuildLines()
{
	for (size_t index = 0; index < points.size(); ++index)
	{
		const Vec2 pointA = points[index];
		const Vec2 pointB = points[(index + 1) % points.size()];

		Vec2 lineDir = (pointA - pointB).Normalized();

		m_lines.push_back(Line(pointB, lineDir, (pointA - pointB).GetLength()));
	}
}

void CPolygon::ComputeArea()
{
	m_signedArea = 0.0f;
	for (size_t index = 0; index < points.size(); ++index)
	{
		const Vec2& pointA = points[index];
		const Vec2& pointB = points[(index + 1) % points.size()];
		m_signedArea += pointA.x * pointB.y - pointB.x * pointA.y;
	}
	m_signedArea *= 0.5f;
}

void CPolygon::RecenterOnCenterOfMass()
{
	Vec2 centroid;
	for (size_t index = 0; index < points.size(); ++index)
	{
		const Vec2& pointA = points[index];
		const Vec2& pointB = points[(index + 1) % points.size()];
		float factor = pointA.x * pointB.y - pointB.x * pointA.y;
		centroid.x += (pointA.x + pointB.x) * factor;
		centroid.y += (pointA.y + pointB.y) * factor;
	}
	centroid /= 6.0f * m_signedArea;

	for (Vec2& point : points)
	{
		point -= centroid;
	}
	position += centroid;
}

void CPolygon::ComputeLocalInertiaTensor()
{
	m_localInertiaTensor = 0.0f;
	for (size_t i = 0; i + 1 < points.size(); ++i)
	{
		const Vec2& pointA = points[i];
		const Vec2& pointB = points[i + 1];

		m_localInertiaTensor += ComputeInertiaTensor_Triangle(Vec2(), pointA, pointB);
	}
}

/*void CPolygon::UpdateAABB()
{
	aabb.Center(position);
	for (const Vec2& point : points)
	{
		aabb.Extend(TransformPoint(point));
	}
}*/
