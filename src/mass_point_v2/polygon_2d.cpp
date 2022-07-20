#include "mass_point_v2/polygon_2d.h"
#include "mass_point_v2/math_2d.h"

#include <algorithm>
#include <stack>

Polygon2D::Polygon2D(double* _vertex, int _n) : vertex(std::vector<Point2D> {}), n(_n)
{
	for (int i = 0; i != n; i++) {
		vertex.push_back(Point2D(_vertex[2 * i], _vertex[2 * i + 1]));
	}
}

std::vector<Segment2D> Polygon2D::side() const
{
	std::vector<Segment2D> edge;
	for (auto& v : vertex)
	{
		int index = (int)(& v - &vertex[0]);
		if (index < n - 1)
			edge.push_back(Segment2D(v, *(&v + 1)));
		else
			edge.push_back(Segment2D(v, vertex[0]));
	}
	return edge;
}

RegularPolygon2D::RegularPolygon2D(Point2D _center, Point2D _begin, int _n) : Polygon2D({})
{
	double r(dist(_center, _begin));
	Point2D p0 = _begin - _center;
	vertex.push_back(_begin);
	n = _n;
	for (int i = 1; i != n; i++)
	{
		p0 = p0.rot(2 * M_PI / n);
		vertex.push_back(p0 + _center);
	}
}

Circle2D RegularPolygon2D::circle() const { return Circle2D(vertex[0], vertex[1], vertex[2]); }
Point2D RegularPolygon2D::center() const { return circle().center; }
double RegularPolygon2D::r() const { return circle().r; }

bool isNode(const Point2D& p, const Polygon2D& pg)
{
	return std::any_of(pg.vertex.begin(), pg.vertex.end(), 
		[&p](const Point2D& p1) { return isNear(p, p1); });
}

int calcWindingNumber(const Point2D& p, const Polygon2D& pg)
{
	int wn(0);
	std::vector<Segment2D> sides = pg.side();
	std::vector<Segment2D> sides_cut_line_parallel_xaxis{};
	for (auto& e : sides)
	{
		if (isInRange(p.y, e.pA.y, e.pB.y))
		{
			if (substitutePointInLine(p, e) > 0) wn++;
			else if (substitutePointInLine(p, e) < 0) wn--;
			else continue;
		}
	}
	return wn;
}

bool isInPolygon(const Point2D& p, const Polygon2D& pg)
{
	// point is inside the polygon if winding number is non zero
	return calcWindingNumber(p, pg);
}

bool isIntersect(const Segment2D& s, const Polygon2D& pg)
{
	// assume the segment is not intersect with polygon if it is part of an edge (except for one point is on extended line of an edge)
	return (((isInPolygon(s.pA, pg)) && (!isInPolygon(s.pB, pg))) || ((!isInPolygon(s.pA, pg)) && (isInPolygon(s.pB, pg))));
}

Polygon2D calcConvexHull(std::vector<Point2D>& vertex)
{
	// using Graham's Algorithm
	Point2D p0(vertex[0]);
	size_t N; // N will be initialized after remove duplicates
	int i(0);
	// select the lowest point as p0, if there exists multiple, select the rightmost one
	for (Point2D& v : vertex) {
		if ((isNear(v.y, p0.y) && v.x > p0.x)
			|| (v.y < p0.y))
		{
			p0 = v;
			i = (int)(&v - &vertex[0]);
		}
	}
	// sort the rest of vertices by calc angles
	sort(vertex.begin(), vertex.end(), [&p0](const Point2D& p1, const Point2D& p2) {
		auto calcAngleXAxis = [&p0](const Point2D& v) { return angle(Point2D(1.0, 0.0), v - p0); };
		return (calcAngleXAxis(p1) < calcAngleXAxis(p2)) && (!isNear(p1, p2));
	});
	// remove duplicate vertices
	std::vector<Point2D>::iterator ip = std::unique(vertex.begin(), vertex.end(),
		[](const Point2D& p1, const Point2D& p2) { return isNear(p1, p2); });
	vertex.resize(std::distance(vertex.begin(), ip));
	N = vertex.size();
	// use stack to treat vertices
	std::stack<Point2D> new_vertex;
	new_vertex.push(vertex.at(N - 1));
	new_vertex.push(vertex.at(0));
	Point2D p_top1, p_top2;
	for (int i = 1; i != N - 1; i++)
	{
		p_top1 = new_vertex.top();
		new_vertex.pop();
		p_top2 = new_vertex.top();
		if (substitutePointInLine(vertex[i], Segment2D(p_top2, p_top1)) > 0)
			new_vertex.push(p_top1);
		new_vertex.push(vertex[i]);
	}
	// copy stack to vector
	std::vector<Point2D> convex_hull;
	while (!new_vertex.empty())
	{
		convex_hull.push_back(new_vertex.top());
		new_vertex.pop();
	}
	// change order as an anticlockwise ring of directed segments
	std::reverse(convex_hull.begin(), convex_hull.end() - 1);
	return Polygon2D(convex_hull);
}

std::vector<Polygon2D> calcConvexHullGroups(const std::vector<Polygon2D>& pg_group)
{
	int m((int)pg_group.size());
	std::vector<std::vector<bool>> overlap_board(m,
		std::vector<bool> (m, false)
	);
	for (int i = 0; i != (m - 2); i++) {
		for (int j = i + 1; j != (m - 1); j++) {
			for (auto& v : pg_group.at(i).vertex) {
				if (isInPolygon(v, pg_group.at(j))) {
					overlap_board[i][j] = true;
					overlap_board[j][i] = true;
				}
			}
		}
	}
	std::vector<int> pg_index(m);
	std::generate(pg_index.begin(), pg_index.end(), [num = 0]()mutable{ return num++; });
	std::vector<std::vector<Point2D>> hull_of_pg;
	for (int i = 0; i != m; i++) {
		if (pg_index[i] == i) {
			hull_of_pg.push_back(pg_group[i].vertex);
		}
		int index(pg_index[i]);
		for (int j = i + 1; j != m; j++) {
			if (overlap_board[i][j]) {
				hull_of_pg[index].insert(hull_of_pg[index].end(), pg_group[j].vertex.begin(), pg_group[j].vertex.end());
				pg_index[j] = index;
			}
		}
	}
	std::vector<Polygon2D> hull_group;
	for (auto& hull_index : hull_of_pg) {
		hull_group.push_back(calcConvexHull(hull_index));
	}
	return hull_group;
}