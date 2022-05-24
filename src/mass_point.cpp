#include <iostream>
#include <sstream>
#include <cmath>
#include <array>
#include "mass_duel_v1/mass_point.h"

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
	return (pB - pA).mod();
}

double calcAngleVectors(Point2D vA, Point2D vB)
{
	auto cos_angle = (vA.x * vB.x + vA.y * vB.y) / (vA.mod() * vB.mod());
	return std::acos(cos_angle);
}

std::array <double, 3> calcLineEqCoeff (Point2D pA, Point2D pB)
{
	auto eA = pA.y - pB.y;
	auto eB = pB.x - pA.x;
	auto eC = pA.x * pB.y - pA.y * pB.x;
	std::array <double, 3> coeff {eA, eB, eC};
	return coeff;
}

double calcDistPointLine (Point2D p, std::array <double, 3> l) 
{
	return std::abs((l[0] * p.x + l[1] * p.y + l[2]) / std::pow((l[0] * l[0] + l[1] * l[1]), 0.5));
}

double calcDistTriPoints(Point2D pA, Point2D pB, Point2D pC)
{
	auto l = calcLineEqCoeff(pA, pB);
	return calcDistPointLine(pC, l);
}

std::array <Point2D, 2> calcTanPointsOnCircle(Point2D pM, Point2D pC, double r)
{
	Point2D v_MC = pC - pM;
	auto d_MC = v_MC.mod();
	auto d_angle = std::asin(r / d_MC);
	auto d_M_TP = d_MC * std::cos(d_angle);
	v_MC = v_MC.scalar(1.0 / d_MC);
	std::array<Point2D, 2> pTanPts;
	pTanPts[0] = Point2D(
		v_MC.x * std::cos(d_angle) - v_MC.y * std::sin(d_angle),
		v_MC.x * std::sin(d_angle) + v_MC.y * std::cos(d_angle)
	);
	pTanPts[1] = Point2D(
		v_MC.x * std::cos(-d_angle) - v_MC.y * std::sin(-d_angle),
		v_MC.x * std::sin(-d_angle) + v_MC.y * std::cos(-d_angle)
	);
	pTanPts[0] = pTanPts[0].scalar(d_M_TP) + pM;
	pTanPts[1] = pTanPts[1].scalar(d_M_TP) + pM;
	return pTanPts;
}
