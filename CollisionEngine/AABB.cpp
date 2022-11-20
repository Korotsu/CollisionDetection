#include "AABB.h"

CAABB::CAABB(const std::vector<Vec2>& inPoints, const Vec2& inPosition, const Mat2& inRotation) : CGLObject(), min(Vec2(FLT_MAX, FLT_MAX)), max(Vec2(-FLT_MAX, -FLT_MAX)), position(inPosition), rotation(inRotation)
{
	ComputePoints(inPoints, inRotation);
	CreateBuffers();
}

bool CAABB::DoesOverlap(CAABB& testedAABB)
{
	Vec2 Amin = min + position;
	Vec2 Amax = max + position;
	Vec2 Bmin = testedAABB.min + testedAABB.position;
	Vec2 Bmax = testedAABB.max + testedAABB.position;
	if (Amin.x <= Bmax.x && Bmin.x <= Amax.x && Amin.y <= Bmax.y && Bmin.y <= Amax.y)
	{
		isOverlaping = true;
		testedAABB.isOverlaping = true;
		return true;
	}
	return false;
}

void CAABB::Draw()
{
	// Set transforms (assuming model view mode is set)
	float transfMat[16] = { 1.0f, 0.0f, 0.0f, 0.0f,
							0.0f, 1.0f, 0.0f, 0.0f,
							0.0f, 0.0f, 1.0f, 1.0f,
							position.x, position.y, -1.0f, 1.0f };
	glPushMatrix();
	glMultMatrixf(transfMat);

	// Draw vertices
	BindBuffers();
	if (isOverlaping)
	{
		glColor3f(0, 1, 0);
		isOverlaping = false;
	}
	else
		glColor3f(0, 0, 1);
	glDrawArrays(GL_LINE_LOOP, 0, points.size());
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();
	glColor3f(0, 0, 0);
}

void CAABB::ComputePoints(const std::vector<Vec2>& inPoints, const Mat2& inRotation)
{
	for (Vec2 point : inPoints)
	{
		point = inRotation * point;
		if (point.x < min.x)
			min.x = point.x;

		if (point.y < min.y)
			min.y = point.y;

		if (point.x > max.x)
			max.x = point.x;

		if (point.y > max.y)
			max.y = point.y;
	}
	points = { Vec2(min.x, min.y), Vec2(min.x, max.y), Vec2(max.x, max.y), Vec2(max.x, min.y) };
}

void CAABB::ApplyRotation(const Mat2& inRotation)
{
	rotation = inRotation;
	ComputePoints(points, rotation);
}