#ifndef _MATHS_H_
#define _MATHS_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <float.h>
#include <vector>

#define RAD2DEG(x) ((x)*(180.0f/(float)M_PI))
#define DEG2RAD(x) ((x)*((float)M_PI/180.0f))
constexpr float EPSILON = 0.01f;

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

float ClampAngleRadians(float angle);

struct Vec3;

struct Vec2
{
	float x, y;

	Vec2() : x(0.0f), y(0.0f) {}

	Vec2(float _x, float _y) : x(_x), y(_y) {}

	Vec2(const Vec3& _vec3);

	inline void operator=(const Vec3& rhs);

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

	inline Vec2& operator+=(const Vec3& rhs);

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

	inline bool  IsZero() const
	{
		return x == 0.0f && y == 0.0f;
	}

	inline bool operator==(const Vec2& other) const
	{
		return (x == other.x && y == other.y);
	}

	inline bool operator!=(const Vec2& other) const
	{
		return (x != other.x || y != other.y);
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

	inline void Reflect(Vec2 normal, float elasticity = 0.0f)
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

	inline void Rotate(const float angle) //Radians
	{
		Vec2 copy = Vec2(*this);
		x = copy.x * cos(angle) - copy.y * sin(angle);
		y = copy.x * sin(angle) + copy.y * cos(angle);
	}

	inline Vec2 Rotated(const float angle) //Radians
	{
		Vec2 copy = Vec2(*this);
		copy.x = x * cos(angle) - y * sin(angle);
		copy.y = x * sin(angle) + y * cos(angle);
		return copy;
	}
};

Vec2 minv(const Vec2& a, const Vec2& b);
Vec2 maxv(const Vec2& a, const Vec2& b);


struct Vec3
{
	float x, y, z;

	Vec3() : x(0.0f), y(0.0f), z(0.0f) {}

	Vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

	Vec3(Vec2 vec2) : x(vec2.x), y(vec2.y), z(0.0f) {}

	inline Vec3 operator+(const Vec3& rhs) const
	{
		return Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	inline Vec3& operator+=(const Vec3& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		return *this;
	}

	inline Vec3 operator-(const Vec3& rhs) const
	{
		return Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
	}

	inline Vec3& operator-=(const Vec3& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		return *this;
	}

	inline Vec3 operator*(float factor) const
	{
		return Vec3(x * factor, y * factor, z * factor);
	}

	inline Vec3& operator*=(float factor)
	{
		*this = Vec3(x * factor, y * factor, z * factor);
		return *this;
	}

	inline Vec3 operator/(float factor) const
	{
		return Vec3(x / factor, y / factor, z / factor);
	}

	inline Vec3& operator/=(float factor)
	{
		*this = Vec3(x / factor, y / factor, z / factor);
		return *this;
	}

	inline float operator|(const Vec3& rhs) const
	{
		return x * rhs.x + y * rhs.y + z * rhs.z;
	}

	inline Vec3 operator^(const Vec3& rhs) const
	{
		return Vec3(y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z, x * rhs.y - y * rhs.x);
	}

	inline bool IsZero() const
	{
		return x == 0.0f && y == 0.0f && z == 0.0f;
	}

	inline bool operator==(const Vec3& other) const
	{
		return (x == other.x && y == other.y && z == other.z);
	}

	inline bool operator!=(const Vec3& other) const
	{
		return (x != other.x || y != other.y || z != other.z);
	}

	inline float GetLength() const
	{
		return sqrtf(x * x + y * y + z * z);
	}

	inline float GetSqrLength() const
	{
		return x * x + y * y + z * z;
	}

	inline void	Normalize()
	{
		float length = GetLength();
		x /= length;
		y /= length;
		z /= length;
	}

	inline Vec3	Normalized() const
	{
		Vec3 res = *this;
		res.Normalize();
		return res;
	}

	inline static Vec3 GetTangent(const Vec3& direction, const Vec3& normal)
	{
		return (direction ^ normal) ^ normal;
	}

	inline static Vec2 GetTangent(const Vec2& _direction, const Vec2& _normal)
	{
		Vec3 direction = Vec3(_direction);
		Vec3 normal = Vec3(_normal);
		return Vec2((direction ^ normal) ^ normal);
	}
};

struct Mat2
{
	Vec2 X, Y;

	Mat2() : X(1.0f, 0.0f), Y(0.0f, 1.0f) {}


	Mat2(float a, float b, float c, float d) : X(a, c), Y(b, d) {}

	float	GetDeterminant() const
	{
		return X ^ Y;
	}

	Mat2	GetInverse() const
	{
		return Mat2(Y.y, -X.y, -Y.x, X.x) * (1.0f / GetDeterminant());
	}

	Mat2	GetInverseOrtho() const
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

	Mat2 operator*(float factor) const
	{
		return Mat2(X.x * factor, Y.x * factor, X.y * factor, Y.y * factor);
	}

	inline Vec2 operator*(const Vec2& vec) const
	{
		return Vec2(X.x * vec.x + Y.x * vec.y, X.y * vec.x + Y.y * vec.y);
	}

	inline Vec3 operator*(const Vec3& vec) const
	{
		return Vec3(X.x * vec.x + Y.x * vec.y, X.y * vec.x + Y.y * vec.y, vec.z);
	}

	// make sure that pt1 and pt2 are not clipping through
	bool Clip(const Vec2& center, const Vec2& normal, Vec2& pt1, Vec2& pt2);

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
	float length;

	Line() = default;
	Line(Vec2 _point, Vec2 _dir, float _length) : point(_point), dir(_dir), length(_length) {}

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
		return Line(position + rotation * point, rotation * dir, length);
	}

	inline Vec2	Project(const Vec2& pt) const
	{
		return point + dir * ((pt - point) | dir);
	}

	float	RayCast(const Vec2& rayStart, const Vec2& rayDir) const
	{
		Vec2 n = GetNormal();
		float dirDotN = n | rayDir;
		if (dirDotN == 0.0f)
		{
			return -1.0f;
		}

		float colDist = ((point - rayStart) | n) / dirDotN;
		Vec2 colPoint = rayStart + rayDir * colDist;
		float distOnLine = (colPoint - point) | dir;
		return Select(distOnLine >= 0.0f && distOnLine <= length, colDist, -1.0f);
	}

	float	UnProject(const Vec2& pt, const Vec2& unprojectDir)
	{
		return Select((unprojectDir | GetNormal()) < 0.0f, 0.0f, Max(RayCast(pt, unprojectDir), 0.0f));
	}

	void	GetPoints(Vec2& start, Vec2& end) const
	{
		start = point;
		end = point + dir * length;
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

		return (baseArea + EPSILON >= t1Area + t2Area + t3Area) ? true : false;
	}
};

struct AABB
{
	Vec2 min, max;

	void Center(const Vec2& point)
	{
		min = max = point;
	}

	void Extend(const Vec2& point)
	{
		min = minv(min, point);
		max = maxv(max, point);
	}

	bool Intersect(const AABB& aabb)
	{
		bool separateAxis = (min.x > aabb.max.x) || (min.y > aabb.max.y) || (aabb.min.x > max.x) || (aabb.min.y > max.y);
		return !separateAxis;
	}
};

// 2D Analytic LCP solver (find exact solution)
bool Solve2DLCP(const Mat2& A, const Mat2& invA, const Vec2& b, Vec2& x);


/*float KernelDefault(float r, float h);
float KernelSpikyGradientFactor(float r, float h);
float KernelViscosityLaplacian(float r, float h);*/


#endif