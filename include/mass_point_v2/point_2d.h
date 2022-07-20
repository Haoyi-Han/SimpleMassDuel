#ifndef POINT_2D_H
#define POINT_2D_H

#include <iostream>
#include <ostream>
#include <vector>

struct Point2D
{
	// member variables
	double x;
	double y;
	double v;
	double theta;

	// constructor functions
	explicit Point2D(double _x = 0.0, double _y = 0.0, double _v = 0.0, double _theta = 0.0) :
		x(_x), y(_y), v(_v), theta(_theta) {}
	// overload basic functions
	bool operator == (const Point2D& p);
	bool operator != (const Point2D& p);
	friend Point2D operator + (const Point2D& pA, const Point2D& pB);
	friend Point2D operator - (const Point2D& pA, const Point2D& pB);
	Point2D scalar(const double& m);
	// calculate vx and vy functions
	double vx() const;
	double vy() const;
	// calculate module function
	double mod() const;
	// rotate function
	Point2D rot(const double& angle) const;
	// move functions
	Point2D move(const double& tick_time, const double& velocity, const double& angle, bool speed_cartesian = false) const;
	Point2D move(const double& tick_time, const Point2D& target, const double& velocity) const;
	Point2D move(const double& tick_time, const Point2D& target) const;
	Point2D move(const double& tick_time) const;  // overload Point2D::move() without arguments velocity and angle
};

// functions for one Point2D object
std::ostream& operator<<(std::ostream& os, const Point2D& p);
std::string pToString(const Point2D& p, bool only_position = false, bool speed_cartesian = false, std::string sep = ";");
std::string pToString2(const Point2D& p);

// functions for multiple Point2D objects
bool isZero(const Point2D& p, double esp = 1e-6, bool only_position = true);
bool isNear(const Point2D& pA, const Point2D& pB, double esp = 1e-6, bool only_position = true);
double innerProd(const Point2D& pA, const Point2D& pB);

double dist(const Point2D& pA, const Point2D& pB);
double angle(const Point2D& pA, const Point2D& pB);

Point2D midpoint(const Point2D& pA, const Point2D& pB);
Point2D barycenter(const std::vector<Point2D>& pts);

#endif // MASS_POINT_2D_H