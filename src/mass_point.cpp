#include <iostream>
#include <sstream>
#include <cmath>
#include <array>
#include "mass_duel_v1/mass_point.h"

double getVX(Point2D p)
{
	return p.v * std::cos(p.theta);
}

double getVY(Point2D p)
{
	return p.v * std::sin(p.theta);
}

void changePos(Point2D p, double nx, double ny)
{
	p.x = nx;
	p.y = ny;
}

std::ostream& operator<<(std::ostream& os, const Point2D& p)
{
	return os << p.x << ";" << p.y << ";" << p.v << ";" << p.theta;
}

std::string pToString(Point2D p)
{
	std::ostringstream p_str; 
	p_str << p; 
	return p_str.str();
}

double calcDistPoints(Point2D pA, Point2D pB) 
{
	auto dx = pB.x - pA.x;
	auto dy = pB.y - pA.y;
	return std::pow(dx * dx + dy * dy, 0.5);
}

std::array <double, 3> calcLineEqCoeff (Point2D pA, Point2D pB)
{
	auto eA = pA.y - pB.y;
	auto eB = pB.x - pA.x;
	auto eC = pA.x * pB.y - pA.y * pB.x;
	std::array <double, 3> coeff {eA, eB, eC};
	return coeff;
}

double CalcDistPointLine (Point2D p, std::array <double, 3> l) {
	return std::abs((l[0] * p.x + l[1] * p.y + l[2]) / std::pow((l[0] * l[0] + l[1] * l[1]), 0.5));
}
