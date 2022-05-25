#ifndef MASS_POINT_H
#define MASS_POINT_H

#include <iostream>
#include <cmath>

struct Point2D
{
	// member variables
	double x;
	double y;
	double v;
	double theta;

	// constructor functions
	Point2D(double _x, double _y, double _v, double _theta) :
		x(_x), y(_y), v(_v), theta(_theta) {}
	Point2D(double _x, double _y) :
		x(_x), y(_y), v(0.0), theta(0.0) {}
	Point2D() : x(0.0), y(0.0), v(0.0), theta(0.0) {}
	// overload basic functions
	bool operator == (Point2D& p)
	{
		return ((x == p.x) && (y == p.y) && (v == p.v) && (theta == p.theta));
	}
	Point2D operator + (Point2D& p) // Directional
	{
		return Point2D(x + p.x, y + p.y, v, theta);
	}
	Point2D operator - (Point2D& p) // Directional
	{
		return Point2D(x - p.x, y - p.y, v, theta);
	}
	Point2D scalar(double m)
	{
		return Point2D(m * x, m * y, v, theta);
	}
	// calculate vx and vy functions
	double vx() { return v * std::cos(theta); }
	double vy() { return v * std::sin(theta); }
	// calculate module function
	double mod() { return std::pow(x * x + y * y, 0.5); }
	// rotate function
	Point2D rot(double angle)
	{
		return Point2D(
			x * std::cos(angle) - y * std::sin(angle),
			x * std::sin(angle) + y * std::cos(angle));
	}
	// move functions
	Point2D move(double tick_time, double velocity, double angle)
	{
		return Point2D(
			x + velocity * std::cos(angle) * tick_time, 
			y + velocity * std::sin(angle) * tick_time, 
			velocity, angle);
	}
	Point2D move(double tick_time)  // overload Point2D::move() without arguments velocity and angle
	{
		Point2D p{ x, y, v, theta };
		return p.move(tick_time, v, theta);
	}
};

// functions for one Point2D object
std::ostream& operator<<(std::ostream& os, const Point2D& p);
std::string pToString(Point2D p);

// functions for multiple Point2D objects
double calcDistPoints(Point2D pA, Point2D pB);
double calcAngleVectors(Point2D vA, Point2D vB);
std::array <double, 3> calcLineEqCoeff (Point2D pA, Point2D pB);
double calcDistPointLine(Point2D p, std::array <double, 3> l);
double calcDistTriPoints(Point2D pA, Point2D pB, Point2D pC);
bool isTwoPointBilateral(Point2D pA, Point2D pB, std::array <double, 3> l);
std::array <Point2D, 2> calcTanPointsOnCircle(Point2D pM, Point2D pC, double r);

//functions for other math calculations
double calcWeightedAvg(double A, double B, double fA, double fB);

#endif // MASS_POINT_H