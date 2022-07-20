#include "mass_point_v2/line_2d.h"
#include "mass_point_v2/math_2d.h"

#include <stdexcept>

double Line2D::k(bool y_slope) const 
{
	if (y_slope) return (isZero(A)) ? INF_F : (-B / A);
	return (isZero(B)) ? INF_F : (-A / B); 
}
double Line2D::val(const Point2D& p) const { return A * p.x + B * p.y + C; }
double Line2D::x(const double& y) const { return isZero(A) ? INF_F : (-(B * y + C) / A); }
double Line2D::y(const double& x) const { return isZero(B) ? INF_F : (-(A * x + C) / B); }

Line2D Segment2D::l() const { return Line2D(pA, pB); }

double dist(const Point2D& p, const Line2D& l) { return std::abs(l.val(p)) / std::hypot(l.A, l.B); };
double dist(const Point2D& pA, const Point2D& pB, const Point2D& pC) { return dist(pA, Line2D(pB, pC)); };
bool isTwoPointBilateral(const Point2D& pA, const Point2D& pB, const Line2D& l) { return (l.val(pA) * l.val(pB) < 0); };

bool isOnLine(const Point2D& p, const Line2D& l) { return isNear(l.val(p), 0.0); }

bool isOnSegment(const Point2D& p, const Segment2D& s)
{
	if (!isOnLine(p, s.l())) return false;
	if (isNear(s.pA, s.pB)) return (isNear(p, s.pA) && isNear(p, s.pB));
	return isNear(s.pA.x, s.pB.x) ? isInRange(p.y, s.pA.y, s.pB.y) : isInRange(p.x, s.pA.x, s.pB.x);
}

bool isOnSegment(const Point2D& p, const Point2D& pA, const Point2D& pB)
{
	return isOnSegment(p, Segment2D(pA, pB));
}

double substitutePointInLine(const Point2D& p, const Segment2D& s)
{
	// return relative position of point to line
	// here use Segment2D to describe the direction from pA to pB
	// return > 0 if p lies on the left of the line, == 0 if on the line, < 0 if on the right of the line
	return (p.y - s.pA.y) * (s.pB.x - s.pA.x) - (p.x - s.pA.x) * (s.pB.y - s.pA.y);
}

bool isLinesParallel(const Line2D& l1, const Line2D& l2)
{
	return isNear(l1.A * l2.B - l1.B * l2.A, 0.0);
}

Point2D calcInterPointTwoLines(const Line2D& l1, const Line2D& l2)
{
	if (isLinesParallel(l1, l2)) throw std::runtime_error("Lines are parallel, no intersection point.");
	double delta(l1.A * l2.B - l1.B * l2.A);
	return Point2D(
		((- l1.C) * l2.B - l1.B * (-l2.C)) / delta,
		(l1.A * (-l2.C) - (-l1.C) * l2.A) / delta
	);
}

Line2D calcLinePerpendicular(const Point2D& p, const Line2D& l)
{
	return Line2D(l.B, -l.A, l.A * p.y - l.B * p.x);
}

Point2D calcFootOfPerpendicular(const Point2D& p, const Line2D& l)
{
	return calcInterPointTwoLines(l, calcLinePerpendicular(p, l));
}

bool isSegmentsIntersect(const Segment2D& s1, const Segment2D& s2)
{
	if (isNear(s1.pA, s1.pB) && isNear(s1.pB, s2.pA) && isNear(s2.pA, s2.pB), isNear(s2.pB, s1.pA)) return true;
	if (isNear(s1.pA, s1.pB)) {
		Line2D l(s2.pA, s2.pB);
		return isOnLine(s1.pA, l) && isOnLine(s1.pB, l);
	}
	if (isNear(s2.pA, s2.pB)) {
		Line2D l(s1.pA, s1.pB);
		return isOnLine(s2.pA, l) && isOnLine(s2.pB, l);
	}
	Line2D l1(s1.pA, s1.pB);
	Line2D l2(s2.pA, s2.pB);
	if (isLinesParallel(l1, l2)) return false;
	Point2D inter_p(calcInterPointTwoLines(l1, l2));
	return isOnSegment(inter_p, s1.pA, s1.pB) && isOnSegment(inter_p, s2.pA, s2.pB);
}

bool isSegmentsIntersect(const Point2D& pA, const Point2D& pB, const Point2D& pC, const Point2D& pD)
{
	return isSegmentsIntersect(Segment2D(pA, pB), Segment2D(pC, pD));
}
