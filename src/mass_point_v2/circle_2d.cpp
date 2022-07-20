#include "mass_point_v2/circle_2d.h"
#include "mass_point_v2/math_2d.h"

Circle2D::Circle2D(Point2D pA, Point2D pB, Point2D pC)
{
	double x1(pA.x), x2(pB.x), x3(pC.x);
	double y1(pA.y), y2(pB.y), y3(pC.y);
	double a(x1 - x2);
	double b(y1 - y2);
	double c(x1 - x3);
	double d(y1 - y3);
	double e(((x1 * x1 - x2 * x2) + (y1 * y1 - y2 * y2)) / 2.0);
	double f(((x1 * x1 - x3 * x3) + (y1 * y1 - y3 * y3)) / 2.0);
	double det(b * c - a * d);
	if (isZero(det))
	{
		center = Point2D();
		r = -1.0;
	}
	double x0((d * e - b * f) / det);
	double y0((a * f - c * e) / det);
	center = Point2D(x0, y0);
	r = dist(center, pA);
}

CouplePoint2D calcTanPtsOnCircle(const Point2D& pM, const Point2D& pC, const double& r)
{
	double d_cos_angle(r / dist(pC, pM));
	// if point is on the circle
	if (isNear(d_cos_angle, 1.0))
		return CouplePoint2D(pM, pM);
	// if point is inside the circle
	Point2D v_CM(pM - pC);
	v_CM = v_CM.scalar(d_cos_angle);
	double d_angle(std::acos(d_cos_angle));
	CouplePoint2D pTanPts{
		v_CM.rot(d_angle) + pC,
		v_CM.rot(-d_angle) + pC
	};
	return pTanPts;
}

CouplePoint2D calcTanPtsOnCircle(const Point2D& pM, const Circle2D& cc)
{
	return calcTanPtsOnCircle(pM, cc.center, cc.r);
}

size_t judgePointCircleRelationship(const Point2D& p, const Circle2D& cc)
{
	// this function return 0 if p is out of circle, 1 if on and 2 if inside the circle
	double d(dist(p, cc.center));
	return isNear(d, cc.r) ? 1 : ((d > cc.r) ? 0 : 2);
}

size_t judgeLineCircleRelationship(const Line2D& l, const Circle2D& cc)
{
	// this function return number of intersection points (0, 1, 2)
	// you can judge relationship by switch case
	double d(dist(cc.center, l));
	return isNear(d, cc.r) ? 1 : ((d > cc.r) ? 0 : 2);
}

size_t judgeSegmentCircleRelationship(const Segment2D& sg, const Circle2D& cc)
{
	Line2D l(sg.l());
	size_t cc_l_relationship(judgeLineCircleRelationship(l, cc));
	switch (cc_l_relationship) {
	case 0: { return 0; break; }
	case 1: {
		// calculate the tangent point by calculating the intersection point between segment line and its orthogonal line passing the circle center
		Point2D tanPoint(calcFootOfPerpendicular(cc.center, l));
		return isOnSegment(tanPoint, sg) ? 1 : 0;
		break;
	}
	case 2: {
		size_t judge_num_A(judgePointCircleRelationship(sg.pA, cc));
		size_t judge_num_B(judgePointCircleRelationship(sg.pB, cc));
		// for all cases except for both two extreme points on or outside the circle, return 1
		if ((judge_num_A != judge_num_B) && ((judge_num_A == 2) || (judge_num_B == 2))) { return 1; break; }
		// if both two extreme points are outside the circle
		if ((judge_num_A == 2) && (judge_num_B == 2)) { return 0; break; }
		// if both two extreme points are on the circle
		// consider the case where the segment is degenerated to a point
		if ((judge_num_A == 1) && (judge_num_B == 1)) { return isNear(sg.pA, sg.pB) ? 1 : 2; break; }   
		// for the other cases: check the relationship of the circle and the intersection point 
		// the intersection point is between segment line and its orthogonal line passing the circle center
		Point2D midInterPoint(calcFootOfPerpendicular(cc.center, l));
		// if one point on and the other point outside the circle
		if ((judge_num_A | judge_num_B) == 1) { return isOnSegment(midInterPoint, sg) ? 2 : 1; break; }
		// if both two extreme points outside the circle
		return isOnSegment(midInterPoint, sg) ? 2 : 0; break;
	}
	default: return 0;
	}
}