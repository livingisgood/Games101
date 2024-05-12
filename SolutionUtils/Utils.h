#pragma once
#include <eigen3/Eigen/Eigen>

namespace Utils
{
	constexpr float PI = 3.1415927f;

	inline float deg2rad(float degrees)
	{
		return degrees * PI / 180.0f;
	}
	
	template<typename T>
	T min(T a, T b)
	{
		return a <= b ? a : b;
	}

	template<typename T>
	T max(T a, T b)
	{
		return a >= b ? a : b;
	}

	template<typename T>
	T min3(T a, T b, T c)
	{
		return min(min(a, b), c);
	}

	template<typename T>
	T max3(T a, T b, T c)
	{
		return max(max(a, b), c);
	}

	inline Eigen::Matrix4f mat_proj(float eye_fov, float aspect, float zNear, float zFar)
	{
		Eigen::Matrix4f projection;
		
		float py = 1 / tan(eye_fov * PI / 180.f);
		float px = py / aspect;
		float k = zNear + zFar / (zFar - zNear);
		float b = 2 * zNear * zFar / (zFar - zNear);

		// camera looking towards -z, plane z = -near mapping to 1, z = -far mapping to -1.

		projection <<
			px, 0, 0, 0,
			0, py, 0, 0,
			0, 0, k, b,
			0, 0, -1, 0;

		return projection;
	}

	inline float cross_v2(Eigen::Vector2f a, Eigen::Vector2f b)
	{
		return a.y() * b.x() - a.x() * b.y();
	}

	inline Eigen::Vector2f cut_z(Eigen::Vector3f v) { return {v.x(), v.y()}; }

	inline Eigen::Vector3f tri_coord(Eigen::Vector2f p, Eigen::Vector2f a, Eigen::Vector2f b, Eigen::Vector2f c)
	{
		Eigen::Vector2f V0 = b - a;
		Eigen::Vector2f V1 = c - a;

		float s = cross_v2(V0, V1);
		float bc = cross_v2(b - p, c - p);
		float ca = cross_v2(c - p, a - p);
		float ab = cross_v2(a - p, b - p);

		return {bc / s, ca / s, ab / s};
	}
}