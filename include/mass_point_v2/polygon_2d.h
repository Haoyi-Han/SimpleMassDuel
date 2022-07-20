#ifndef POLYGON_2D_H
#define POLYGON_2D_H

#include "mass_point_v2/point_2d.h"
#include "mass_point_v2/circle_2d.h"
#include <vector>

struct Polygon2D // assumed convex polygon
{
	std::vector<Point2D> vertex;
	int n;
	Polygon2D(std::vector<Point2D> _vertex) : vertex(_vertex), n((int)_vertex.size()) {} // assumed sorted
	Polygon2D(Point2D* _vertex, int _n) : vertex(_vertex, _vertex + _n), n(_n) {}
	Polygon2D(double* _vertex, int _n);
	std::vector<Segment2D> side() const;
};

struct RegularPolygon2D : Polygon2D
{
	using Polygon2D::Polygon2D;
	RegularPolygon2D(std::vector<Point2D> _vertex) : Polygon2D(_vertex) {}
	RegularPolygon2D(Point2D* _vertex, int _n) : Polygon2D(_vertex, n) {}
	RegularPolygon2D(Point2D _center, Point2D _begin, int _n);
	Circle2D circle() const;
	Point2D center() const;
	double r() const;
};

bool isNode(const Point2D& p, const Polygon2D& pg);
int calcWindingNumber(const Point2D& p, const Polygon2D& pg);
bool isInPolygon(const Point2D& p, const Polygon2D& pg);
bool isIntersect(const Segment2D& s, const Polygon2D& pg);
Polygon2D calcConvexHull(std::vector<Point2D>& vertex);
std::vector<Polygon2D> calcConvexHullGroups(const std::vector<Polygon2D>& pg_group);

#endif // POLYGON_2D_H