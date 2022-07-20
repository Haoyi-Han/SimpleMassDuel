#ifndef CIRCLE_2D_H
#define CIRCLE_2D_H

#include "mass_point_v2/point_2d.h"
#include "mass_point_v2/line_2d.h"

struct Circle2D
{
	// member variables
	Point2D center;
	double r;

	// constructor functions
	explicit Circle2D(Point2D _center = Point2D(), double _r = 0.0) : center(_center), r(_r) {}
	explicit Circle2D(Point2D pA, Point2D pB, Point2D pC);
};

CouplePoint2D calcTanPtsOnCircle(const Point2D& pM, const Point2D& pC, const double& r);
CouplePoint2D calcTanPtsOnCircle(const Point2D& pM, const Circle2D& cc);
size_t judgePointCircleRelationship(const Point2D& p, const Circle2D& cc);
size_t judgeLineCircleRelationship(const Line2D& l, const Circle2D& cc);
size_t judgeSegmentCircleRelationship(const Segment2D& sg, const Circle2D& cc);

#endif // CIRCLE_2D_H