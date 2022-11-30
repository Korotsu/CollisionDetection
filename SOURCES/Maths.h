#ifndef _MATHS_H_
#define _MATHS_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <float.h>
#include <vector>


#define RAD2DEG(x) ((x)*(180.0f/(float)M_PI))
#define DEG2RAD(x) ((x)*((float)M_PI/180.0f))

template<typename T>
inline T Select(bool condition, T a, T b)
{
	return ((T)condition) * a + (1 - ((T)condition)) * b;
}

template<typename T>
inline T Min(T a, T b)
{
	return Select(a < b, a, b);
}

template<typename T>
inline T Max(T a, T b)
{
	return Select(a > b, a, b);
}

template<typename T>
inline T Clamp(T val, T min, T max)
{
	return Min(Max(val, min), max);
}

float Sign(float a);

float Random(float from, float to);

struct Vec2
{
	float x, y;

	Vec2() : x(0.0f), y(0.0f) {}

	Vec2(float _x, float _y) : x(_x), y(_y) {}

	inline Vec2 operator+(const Vec2& rhs) const
	{
		return Vec2(x + rhs.x, y + rhs.y);
	}

	inline Vec2& operator+=(const Vec2& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		return *this;
	}

	inline Vec2 operator-(const Vec2& rhs) const
	{
		return Vec2(x - rhs.x, y - rhs.y);
	}

	inline Vec2& operator-=(const Vec2& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		return *this;
	}

	inline Vec2 operator*(float factor) const
	{
		return Vec2(x * factor, y * factor);
	}

	inline Vec2& operator*=(float factor)
	{
		*this = Vec2(x * factor, y * factor);
		return *this;
	}

	inline Vec2 operator/(float factor) const
	{
		return Vec2(x / factor, y / factor);
	}

	inline Vec2& operator/=(float factor)
	{
		*this = Vec2(x / factor, y / factor);
		return *this;
	}

	inline float operator|(const Vec2& rhs) const
	{
		return x * rhs.x + y * rhs.y;
	}

	inline float operator^(const Vec2& rhs) const
	{
		return x * rhs.y - y * rhs.x;
	}

	inline float GetLength() const
	{
		return sqrtf(x * x + y * y);
	}

	inline float GetSqrLength() const
	{
		return x * x + y * y;
	}

	inline void	Normalize()
	{
		float length = GetLength();
		x /= length;
		y /= length;
	}

	inline Vec2	Normalized() const
	{
		Vec2 res = *this;
		res.Normalize();
		return res;
	}

	inline void Reflect(Vec2 normal, float elasticity = 1.0f)
	{
		*this = *this - normal * (1.0f + elasticity) * (*this | normal);
	}

	inline Vec2 GetNormal() const
	{
		return Vec2(-y, x);
	}

	inline float Angle(const Vec2& to)
	{
		float cosAngle = Clamp(Normalized() | to.Normalized(), -1.0f, 1.0f);
		float angle = RAD2DEG(acosf(cosAngle)) * Sign(*this ^ to);
		return angle;
	}
};


struct Mat2
{
	Vec2 X, Y;

	Mat2() : X(1.0f, 0.0f), Y(0.0f, 1.0f) {}

	Mat2(float a, float b, float c, float d) : X(a, c), Y(b, d) {}

	inline Mat2	GetInverse() const
	{
		return Mat2(X.x, X.y, Y.x, Y.y);
	}

	inline float	GetAngle() const
	{
		return Vec2(1.0f, 0.0f).Angle(X);
	}

	inline void	SetAngle(float angle)
	{
		float c = cosf(angle * ((float)M_PI / 180.0f));
		float s = sinf(angle * ((float)M_PI / 180.0f));

		X.x = c; X.y = s;
		Y.x = -s; Y.y = c;
	}

	inline void Rotate(float angle)
	{
		Mat2 matRot;
		matRot.SetAngle(angle);
		*this = *this * matRot;
	}

	inline Mat2 operator*(const Mat2& rhs) const
	{
		return Mat2(X.x * rhs.X.x + Y.x * rhs.X.y, X.x * rhs.Y.x + Y.x * rhs.Y.y, X.y * rhs.X.x + Y.y * rhs.X.y, X.y * rhs.Y.x + Y.y * rhs.Y.y);
	}

	inline Vec2 operator*(const Vec2& vec) const
	{
		return Vec2(X.x * vec.x + Y.x * vec.y, X.y * vec.x + Y.y * vec.y);
	}

	inline bool operator==(const Mat2& other) const
	{
		return (X.x == other.X.x && X.y == other.X.y && Y.x == other.Y.x && Y.y == other.Y.y);
	}

	inline bool operator!=(const Mat2& other) const
	{
		return (X.x != other.X.x || X.y != other.X.y || Y.x != other.Y.x || Y.y != other.Y.y);
	}
};

struct Line
{
	Vec2 point, dir;

	Line() = default;
	Line(Vec2 _point, Vec2 _dir) : point(_point), dir(_dir) {}

	inline Vec2	GetNormal() const
	{
		return dir.GetNormal();
	}

	// positive value means point above line, negative means point is under line
	inline float	GetPointDist(const Vec2& pt) const
	{
		return (pt - point) | GetNormal();
	}

	inline Line	Transform(const Mat2& rotation, const Vec2& position) const
	{
		return Line(position + rotation * point, rotation * dir);
	}

	inline Vec2	Project(const Vec2& pt) const
	{
		return point + dir * ((pt - point) | dir);
	}
};


template<typename T>
bool find(const T& element, const std::vector<T>& inList)
{
	return (std::find(inList.begin(), inList.end(), element) != inList.end())
}

struct Triangle
{
	Vec2 a, b, c;
	Triangle() = default;
	Triangle(const Vec2& A, const Vec2& B, const Vec2& C) : a(A), b(B), c(C) {}

	// Get area using Heron's formula
	inline const float Area() const
	{
		const float AB = (b - a).GetLength();
		const float BC = (c - b).GetLength();
		const float CA = (a - c).GetLength();
		const float s = (AB + BC + CA) / 2;
		return sqrtf(s * (s - AB) * (s - BC) * (s - CA));
	}

	inline static const float Area(const Vec2& a, const Vec2& b, const Vec2& c)
	{
		const float AB = (b - a).GetLength();
		const float BC = (c - b).GetLength();
		const float CA = (a - c).GetLength();
		const float s = (AB + BC + CA) / 2;
		return sqrtf(s * (s - AB) * (s - BC) * (s - CA));
	}

	inline bool IsPointInside(const Vec2& point) const
	{
		const float baseArea = Area();
		const float t1Area = Triangle::Area(point, a, b);
		const float t2Area = Triangle::Area(point, b, c);
		const float t3Area = Triangle::Area(point, c, a);

		return (baseArea == t1Area + t2Area + t3Area) ? true : false;
	}

	inline static bool IsPointInside(const Vec2& point, const Vec2& a, const Vec2& b, const Vec2& c)
	{
		const float baseArea = Triangle::Area(a, b, c);
		const float t1Area = Triangle::Area(point, a, b);
		const float t2Area = Triangle::Area(point, b, c);
		const float t3Area = Triangle::Area(point, c, a);

		return (baseArea + 1 >= t1Area + t2Area + t3Area) ? true : false;
	}
};

#endif