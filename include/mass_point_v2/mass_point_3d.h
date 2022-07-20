#ifndef MASS_POINT_3D_H
#define MASS_POINT_3D_H

#include <iostream>
#include <cmath>

struct Point3D
{
	// member variables
	double x;
	double y;
	double z;
	double v;
	double phi;
	double theta;

	// constructor functions
	Point3D(double _x, double _y, double _z, double _v, double _phi, double _theta) :
		x(_x), y(_y), z(_z), v(_v), phi(_phi), theta(_theta) {}
	Point3D(double _x, double _y, double _z) :
		x(_x), y(_y), z(_z), v(0.0), phi(0.0), theta(0.0) {}
	Point3D() : x(0.0), y(0.0), z(0.0), v(0.0), phi(0.0), theta(0.0) {}
	// overload basic functions
	bool operator == (Point3D& p)
	{
		return ((x == p.x) && (y == p.y) && (z == p.z)
			&& (v == p.v) && (phi == p.phi) && (theta == p.theta));
	}
	Point3D operator + (Point3D& p) // Directional
	{
		return Point3D(x + p.x, y + p.y, z + p.z, v, phi, theta);
	}
	Point3D operator - (Point3D& p) // Directional
	{
		return Point3D(x - p.x, y - p.y, z - p.z, v, phi, theta);
	}
	Point3D scalar(double m)
	{
		return Point3D(m * x, m * y, m * z, v, phi, theta);
	}
	// calculate vx and vy functions
	double vx() { return v * std::sin(theta) * std::cos(phi); }
	double vy() { return v * std::sin(theta) * std::sin(phi); }
	double vz() { return v * std::cos(theta); }
	// calculate module function
	double mod() { return std::pow(x * x + y * y + z * z, 0.5); }
	// rotate function
	Point3D rot(double angle)
	{
		return Point3D(
			x * std::cos(angle) - y * std::sin(angle),
			x * std::sin(angle) + y * std::cos(angle),
			z);
	}
	// move functions
	Point3D move(double tick_time, double velocity, double angle_phi, double angle_theta)
	{
		return Point3D(
			x + velocity * std::sin(angle_theta) * std::cos(angle_phi) * tick_time,
			y + velocity * std::sin(angle_theta) * std::sin(angle_phi) * tick_time,
			z + velocity * std::cos(angle_theta) * tick_time,
			velocity, angle_phi, angle_theta);
	}
	Point3D move(double tick_time)  // overload Point3D::move() without arguments velocity and angle
	{
		Point3D p{ x, y, z, v, phi, theta };
		return p.move(tick_time, v, phi, theta);
	}
};


#endif // MASS_POINT_3D_H