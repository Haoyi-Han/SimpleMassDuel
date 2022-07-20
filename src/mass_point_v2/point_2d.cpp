#include "mass_point_v2/point_2d.h"
#include "mass_point_v2/math_2d.h"

#include <sstream>
#include <iomanip>
#include <algorithm>

#include <ranges>
#include <iterator>

Point2D Point2D::scalar(const double& m)
{
	return Point2D(m * x, m * y, v, theta);
}

double Point2D::vx() const { return v * std::cos(theta); }
double Point2D::vy() const { return v * std::sin(theta); }
double Point2D::mod() const { return std::hypot(x, y); }

Point2D Point2D::rot(const double& angle) const
{
	return Point2D(
		x * std::cos(angle) - y * std::sin(angle),
		x * std::sin(angle) + y * std::cos(angle));
}

Point2D Point2D::move(const double& tick_time, const double& velocity, const double& angle, bool speed_cartesian) const
{
	if (!speed_cartesian)
		return Point2D(
			x + velocity * std::cos(angle) * tick_time,
			y + velocity * std::sin(angle) * tick_time,
			velocity, angle);
	else
	{
		double vx(velocity), vy(angle);
		double speed(std::hypot(vx, vy)), arg(atan2ZeroPrevent(vy, vx));
		return Point2D(
			x + vx * tick_time,
			y + vy * tick_time,
			speed, arg
		);
	}
}

Point2D Point2D::move(const double& tick_time, const Point2D& target, const double& velocity) const
{
	Point2D p(x, y);
	double arg(atan2ZeroPrevent(target.y - p.y, target.x - p.x));
	return p.move(tick_time, velocity, arg);
}

Point2D Point2D::move(const double& tick_time, const Point2D& target) const
{
	Point2D p(x, y, v, theta);
	double arg(atan2ZeroPrevent(target.y - p.y, target.x - p.x));
	return p.move(tick_time, p.v, arg);
}

Point2D Point2D::move(const double& tick_time) const  // overload Point2D::move() without arguments velocity and angle
{
	Point2D p(x, y, v, theta);
	return p.move(tick_time, v, theta);
}

bool Point2D::operator == (const Point2D& p)
{
	return ((x == p.x) && (y == p.y) && (v == p.v) && (theta == p.theta));
}
bool Point2D::operator != (const Point2D& p)
{
	return ((x != p.x) || (y != p.y) || (v != p.v) || (theta != p.theta));
}

Point2D operator + (const Point2D& pA, const Point2D& pB)
{
	if (isZero(pA.vx() + pB.vx()))
		return Point2D(pA.x + pB.x, pA.y + pB.y);
	double pAvx = pA.vx();
	double pBvx = pB.vx();
	double pAvy = pA.vy();
	double pBvy = pB.vy();
	return Point2D(pA.x + pB.x, pA.y + pB.y, std::hypot(pAvx + pBvx, pAvy + pBvy), atan2ZeroPrevent(pAvy + pBvy, pAvx + pBvx));
}

Point2D operator - (const Point2D& pA, const Point2D& pB)
{
	if (isNear(pA.vx(), pB.vx()))
		return Point2D(pA.x - pB.x, pA.y - pB.y);
	double pAvx = pA.vx();
	double pBvx = pB.vx();
	double pAvy = pA.vy();
	double pBvy = pB.vy();
	return Point2D(pA.x - pB.x, pA.y - pB.y, std::hypot(pAvx - pBvx, pAvy - pBvy), atan2ZeroPrevent(pAvy - pBvy, pAvx - pBvx));
}

std::ostream& operator<<(std::ostream& os, const Point2D& p)
{
	return os << p.x << ";" << p.y << ";" << p.v << ";" << p.theta;
}

std::string pToString(const Point2D& p, bool only_position, bool speed_cartesian, std::string sep)
{
	std::ostringstream p_str; 
	if (only_position)
		p_str << p.x << sep << p.y;
	else
	{
		if (speed_cartesian)
			p_str << p.x << sep << p.y << sep 
				<< (p.v * std::cos(p.theta)) << sep
				<< (p.v * std::sin(p.theta));
		else
			p_str << p.x << sep << p.y << sep << p.v << sep << p.theta;
	}
	return p_str.str();
}

std::string pToString2(const Point2D& p)
{
	std::ostringstream p_str;
	p_str << std::setprecision(2) << std::fixed;
	p_str << "[" << p.x << ", " << p.y << "]";
	return p_str.str();
}

bool isZero(const Point2D& p, double esp, bool only_position)
{
	if (only_position) return (isZero(p.x, esp) && isZero(p.y, esp));
	return (isZero(p.x, esp) && isZero(p.y, esp) && isZero(p.vx(), esp) && isZero(p.vy(), esp));
}

bool isNear(const Point2D& pA, const Point2D& pB, double esp, bool only_position)
{
	if (only_position) return (isNear(pA.x, pB.x, esp) && isNear(pA.y, pB.y, esp));
	return (isNear(pA.x, pB.x, esp) && isNear(pA.y, pB.y, esp) && isNear(pA.vx(), pB.vx(), esp) && isNear(pA.vy(), pB.vy(), esp));
}

double innerProd(const Point2D& pA, const Point2D& pB)
{
	return pA.x * pB.x + pA.y * pB.y;
}

double dist(const Point2D& pA, const Point2D& pB)
{
	return std::hypot(pB.x - pA.x, pB.y - pA.y);
}

double angle(const Point2D& pA, const Point2D& pB)
{
	if (isZero(pA) || isZero(pB)) return 0.0;
	double arg(std::acos(innerProd(pA, pB) / (pA.mod() * pB.mod())));
	return (isZero(arg)) ? 0.0 : arg;
}

Point2D midpoint(const Point2D& pA, const Point2D& pB)
{
	return Point2D((pA.x + pB.x) / 2.0, (pA.y + pB.y) / 2.0);
}

Point2D barycenter(const std::vector<Point2D>& pts)
{
	std::vector<double> xs(pts.size());
	std::vector<double> ys(pts.size());
	int i(0);
	std::generate(xs.begin(), xs.end(), [&i, &pts]() { return pts[i].x; });
	i = 0;
	std::generate(ys.begin(), ys.end(), [&i, &pts]() { return pts[i].y; });
	return Point2D(avg(xs), avg(ys));
}