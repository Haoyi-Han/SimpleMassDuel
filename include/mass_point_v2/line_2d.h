#ifndef LINE_2D_H
#define LINE_2D_H

#include "mass_point_v2/point_2d.h"

struct Line2D
{
	// member variables
	double A;
	double B;
	double C;

	// constructor functions
	explicit Line2D(double _A, double _B, double _C) : A(_A), B(_B), C(_C) {}
	explicit Line2D(Point2D pA, Point2D pB) : A(pA.y - pB.y), B(pB.x - pA.x), C(pA.x* pB.y - pA.y * pB.x) {}
	explicit Line2D(Point2D p, double k) : A(k), B(-1.0), C(p.y - k * p.x) {}

	// member functions
	double k(bool y_slope = false) const; // slope
	double val(const Point2D& p) const;
	double x(const double& y) const;
	double y(const double& x) const;
};

struct CouplePoint2D
{
	Point2D pA;
	Point2D pB;
	explicit CouplePoint2D(Point2D _pA, Point2D _pB) : pA(_pA), pB(_pB) {}
};

struct Segment2D : CouplePoint2D
{
	using CouplePoint2D::CouplePoint2D;
	explicit Segment2D(Point2D _pA, Point2D _pB) : CouplePoint2D(_pA, _pB) {}
	explicit Segment2D(const Line2D& _l, double x1, double x2) : CouplePoint2D(Point2D(x1, _l.y(x1)), Point2D(x2, _l.y(x2))) {}
	Line2D l() const;
};

double dist(const Point2D& p, const Line2D& l);
double dist(const Point2D& pA, const Point2D& pB, const Point2D& pC);
bool isTwoPointBilateral(const Point2D& pA, const Point2D& pB, const Line2D& l);

bool isOnLine(const Point2D& p, const Line2D& l);
bool isOnSegment(const Point2D& p, const Segment2D& s);
bool isOnSegment(const Point2D& p, const Point2D& pA, const Point2D& pB);
double substitutePointInLine(const Point2D& p, const Segment2D& s); 
bool isLinesParallel(const Line2D& l1, const Line2D& l2);
Point2D calcInterPointTwoLines(const Line2D& l1, const Line2D& l2);
Line2D calcLinePerpendicular(const Point2D& p, const Line2D& l);
Point2D calcFootOfPerpendicular(const Point2D& p, const Line2D& l);
bool isSegmentsIntersect(const Segment2D& s1, const Segment2D& s2);
bool isSegmentsIntersect(const Point2D& pA, const Point2D& pB, const Point2D& pC, const Point2D& pD);

#endif // LINE_2D_H