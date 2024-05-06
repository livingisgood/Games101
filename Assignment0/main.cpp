#include<cmath>
#include<eigen3/Eigen/Core>
#include<iostream>

Eigen::Matrix3f Rotate(float Degrees)
{
	constexpr float DegToRad = 3.1415927f / 180.f;
	float Rads = Degrees * DegToRad;

	float X = cos(Rads);
	float Y = sin(Rads);

	Eigen::Matrix3f Ret;
	Ret << X, -Y, 0.0f, Y, X, 0.0f, 0.0f, 0.0f, 1.0f;
	return Ret;
}

Eigen::Matrix3f Translate(float X, float Y)
{
	Eigen::Matrix3f Ret;
	Ret << 1.0f, 0.0f, X, 0.0f, 1.0f, Y, 0.0f, 0.0f, 1.0f;
	return Ret;
}

Eigen::Vector2f RotateTranslate(Eigen::Vector2f Input, float Degrees, float X, float Y)
{
	Eigen::Vector3f Ret = Translate(X, Y) * (Rotate(Degrees) * Eigen::Vector3f(Input[0], Input[1], 1.0f));
	return { Ret[0] / Ret[2], Ret[1] / Ret[2] };
}

int main()
{
	Eigen::Vector2f Test = RotateTranslate({2.0f, 1.0f}, 45.0f, 1.0f, 2.0f);
	std::cout << "Result [" << Test[0] << "," << Test[1] << "]" << '\n';

	float X = sqrt(2.0f) * 0.5f;
	float Y = X;

	float X0 = X * 2 - Y * 1 + 1;
	float Y0 = Y * 2 + X * 1 + 2;

	std::cout << "Compare to X = " << X0 << ", Y = " << Y0 << '\n'; 
	
	return 0;
}