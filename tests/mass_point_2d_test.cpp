#include "test_tools/test_tools_2d.hpp"


// point_2d.h
TEST(distTest, Simple)
{
	EXPECT_DOUBLE_EQ(1.0, dist(Point2D(0.0, 0.0), Point2D(1.0, 0.0)));
	EXPECT_DOUBLE_EQ(std::sqrt(2), dist(Point2D(1.0, 0.0), Point2D(0.0, 1.0)));
	EXPECT_DOUBLE_EQ(std::sqrt(2), dist(Point2D(0.0, 0.0), Line2D(1.0, 1.0, -2.0)));
	EXPECT_DOUBLE_EQ(12.0 / 5.0, dist(Point2D(1.0, 1.0), Point2D(4.0, 1.0), Point2D(1.0, 5.0)));
}

TEST(innerProdTest, Simple)
{
	EXPECT_DOUBLE_EQ(0.0, innerProd(Point2D(0.0, 1.0), Point2D(1.0, 0.0)));
	EXPECT_DOUBLE_EQ(1.0, innerProd(Point2D(1.0, 0.0), Point2D(1.0, 0.0)));
}

TEST(isInRangeTest, Simple)
{
	EXPECT_TRUE(isInRange(1.0, 0.5, 2.0));
	EXPECT_FALSE(isInRange(3.0, 1.0, 2.9999));
}

// line_2d.h
TEST(isOnSegmentTest, Simple)
{
	EXPECT_TRUE(isOnSegment(Point2D(8.0 / 3.0, 1.0), Point2D(4.0, 0.0), Point2D(0.0, 3.0)));
	EXPECT_FALSE(isOnSegment(Point2D(20.0 / 3.0, -2.0), Point2D(4.0, 0.0), Point2D(0.0, 3.0)));
}

TEST(isLinesParallelTest, Simple)
{
	EXPECT_TRUE(isLinesParallel(Line2D(1.0, 2.0, 3.0), Line2D(-2.0, -4.0, 8.0)));
	EXPECT_FALSE(isLinesParallel(Line2D(1.0, 2.0, 3.0), Line2D(1.0, -2.0, 3.0)));
}

TEST(isSegmentsIntersectTest, Simple)
{
	EXPECT_TRUE(isSegmentsIntersect(Point2D(0.0, 0.0), Point2D(8.0 / 3.0, 1.0), Point2D(4.0, 0.0), Point2D(0.0, 3.0)));
	EXPECT_FALSE(isSegmentsIntersect(Point2D(0.0, 0.0), Point2D(20.0 / 3.0, -2.0), Point2D(4.0, 0.0), Point2D(0.0, 3.0)));
}

// circle_2d.h
TEST(calcTanPtsOnCircleTest, Simple)
{
	EXPECT_TRUE(EXPECT_COUPLEPOINT2D_EQ(CouplePoint2D(Point2D(3.0, 0.0), Point2D(0.0, 3.0)), calcTanPtsOnCircle(Point2D(3.0, 3.0), Point2D(0.0, 0.0), 3.0)));
}

TEST(judgePointCircleRelationshipTest, Simple)
{
	EXPECT_EQ(0, judgePointCircleRelationship(Point2D(5.0, 0.0), Circle2D(Point2D(0.0, 0.0), 3.0)));
	EXPECT_EQ(1, judgePointCircleRelationship(Point2D(3.0 / std::sqrt(2), 3.0 / std::sqrt(2)), Circle2D(Point2D(0.0, 0.0), 3.0)));
	EXPECT_EQ(2, judgePointCircleRelationship(Point2D(1.0, 2.0), Circle2D(Point2D(0.0, 0.0), 3.0)));
}

TEST(judgeLineCircleRelationshipTest, Simple)
{
	EXPECT_EQ(0, judgeLineCircleRelationship(Line2D(Point2D(7.0, 8.0), 0.2), Circle2D(Point2D(0.0, 0.0), 3.0)));
	EXPECT_EQ(1, judgeLineCircleRelationship(Line2D(1.0, 1.0, -3.0 * std::sqrt(2.0)), Circle2D(Point2D(0.0, 0.0), 3.0)));
	EXPECT_EQ(2, judgeLineCircleRelationship(Line2D(Point2D(7.0, 8.0), Point2D(1.0, 1.0)), Circle2D(Point2D(0.0, 0.0), 3.0)));
}

TEST(judgeSegmentCircleRelationshipTest, Simple)
{
	Point2D pA_0(1.0, 2.0);
	Point2D pA_1(3.0, 0.0);
	Point2D pA_2(3.0, 2.0);
	Point2D pB_0(2.0, 1.0);
	Point2D pB_1(0.0, 3.0);
	Point2D pB_21(6.0, 4.0);
	Point2D pB_22(-6.0, -4.0);
	Circle2D cc(Point2D(0.0, 0.0), 3.0);
	EXPECT_EQ(0, judgeSegmentCircleRelationship(Segment2D(pA_0, pB_0), cc));
	EXPECT_EQ(1, judgeSegmentCircleRelationship(Segment2D(pA_1, pB_0), cc));
	EXPECT_EQ(1, judgeSegmentCircleRelationship(Segment2D(pA_2, pB_0), cc));
	EXPECT_EQ(1, judgeSegmentCircleRelationship(Segment2D(pA_0, pB_1), cc));
	EXPECT_EQ(2, judgeSegmentCircleRelationship(Segment2D(pA_1, pB_1), cc));
	EXPECT_EQ(2, judgeSegmentCircleRelationship(Segment2D(pA_2, pB_1), cc));
	EXPECT_EQ(1, judgeSegmentCircleRelationship(Segment2D(pA_0, pB_21), cc));
	EXPECT_EQ(1, judgeSegmentCircleRelationship(Segment2D(pA_1, pB_21), cc));
	EXPECT_EQ(0, judgeSegmentCircleRelationship(Segment2D(pA_2, pB_21), cc));
	EXPECT_EQ(1, judgeSegmentCircleRelationship(Segment2D(pA_0, pB_22), cc));
	EXPECT_EQ(2, judgeSegmentCircleRelationship(Segment2D(pA_1, pB_22), cc));
	EXPECT_EQ(2, judgeSegmentCircleRelationship(Segment2D(pA_2, pB_22), cc));
}

// polygon_2d.h
TEST(Polygon2DSideTest, Simple)
{
	RegularPolygon2D rpg1(Point2D(0.0, 0.0), Point2D(1.0, -1.0), 4);
	std::vector<Point2D> vert1{
		Point2D(1.0, -1.0), Point2D(1.0, 1.0),
		Point2D(-1.0, 1.0), Point2D(-1.0, -1.0),
	};
	std::vector<Segment2D> edge1{
		Segment2D(Point2D(1.0, -1.0), Point2D(1.0, 1.0)),
		Segment2D(Point2D(1.0, 1.0), Point2D(-1.0, 1.0)),
		Segment2D(Point2D(-1.0, 1.0), Point2D(-1.0, -1.0)),
		Segment2D(Point2D(-1.0, -1.0), Point2D(1.0, -1.0))
	};
	std::vector<Segment2D> side1(rpg1.side());
	std::cout << std::setprecision(4) << std::fixed;
	for (auto& e : side1) {
		std::cout << e.pA.x << "\t" << e.pA.y << "\t" << e.pB.x << "\t" << e.pB.y << std::endl;
	}
	EXPECT_TRUE(EXPECT_VECTOR_POINT2D_EQ(vert1, rpg1.vertex));
	EXPECT_TRUE(EXPECT_COUPLEPOINT2D_EQ(edge1[3], rpg1.side()[3]));
}

TEST(isNodeTest, Simple)
{
	RegularPolygon2D rpg1(Point2D(0.0, 0.0), Point2D(1.0, -1.0), 4);
	EXPECT_TRUE(isNode(Point2D(-1.0, -1.0), rpg1));
	EXPECT_FALSE(isNode(Point2D(0.0, 0.0), rpg1));
}

TEST(calcConvexHullTest, Simple)
{
	// general case
	std::vector<Point2D> vert1{
		Point2D(0.0, 3.0), Point2D(1.0, 1.0),
		Point2D(2.0, 5.0), Point2D(3.0, 3.0),
		Point2D(3.0, 0.0), Point2D(4.0, 5.0),
		Point2D(5.0, 3.0), Point2D(5.0, 1.0),
		Point2D(7.0, 3.0)
	};
	std::vector<Point2D> hull1{
		Point2D(3.0, 0.0), Point2D(5.0, 1.0),
		Point2D(7.0, 3.0), Point2D(4.0, 5.0),
		Point2D(2.0, 5.0), Point2D(0.0, 3.0),
		Point2D(1.0, 1.0)
	};
	// colinear case
	std::vector<Point2D> vert2{
		Point2D(0.0, 4.0), Point2D(2.0, 1.0),
		Point2D(3.0, 3.0), Point2D(3.0, 0.0),
		Point2D(6.0, 2.0)
	};
	std::vector<Point2D> hull2{
		Point2D(3.0, 0.0), Point2D(6.0, 2.0),
		Point2D(0.0, 4.0), Point2D(2.0, 1.0)
	};
	// colinear and point-duplicate case
	std::vector<Point2D> vert3{
		Point2D(0.0, 4.0), Point2D(2.0, 1.0),
		Point2D(3.0, 3.0), Point2D(3.0, 0.0),
		Point2D(6.0, 2.0), Point2D(3.0, 3.0),
		Point2D(3.0, 3.0), Point2D(3.0, 0.0),
		Point2D(3.0, 0.0)
	};
	std::vector<Point2D> hull3{
		Point2D(3.0, 0.0), Point2D(6.0, 2.0),
		Point2D(0.0, 4.0), Point2D(2.0, 1.0)
	};
	EXPECT_TRUE(EXPECT_VECTOR_POINT2D_EQ(hull1, calcConvexHull(vert1).vertex));
	EXPECT_TRUE(EXPECT_VECTOR_POINT2D_EQ(hull2, calcConvexHull(vert2).vertex));
	EXPECT_TRUE(EXPECT_VECTOR_POINT2D_EQ(hull3, calcConvexHull(vert3).vertex));
}

TEST(calcConvexHullGroupsTest, Simple)
{
	double pgv1[10]{ 3.0, 0.0, 5.0, 3.0, 2.0, 5.0, 0.0, 3.0, 1.0, 1.0 };
	double pgv2[8]{ 5.0, 1.0, 7.0, 3.0, 4.0, 5.0, 3.0, 3.0 };
	double pgv3[8]{ 0.0, -1.0, 0.0, 0.0, -1.0, 0.0, -1.0, -1.0 };
	std::vector<Polygon2D> pg_g1{
		Polygon2D(pgv1, 5),
		Polygon2D(pgv2, 4),
		Polygon2D(pgv3, 4)
	};
	double hullv1[14]{ 3.0, 0.0, 5.0, 1.0, 7.0, 3.0, 4.0, 5.0, 2.0, 5.0, 0.0, 3.0, 1.0, 1.0 };
	double hullv2[8]{ 0.0, -1.0, 0.0, 0.0, -1.0, 0.0, -1.0, -1.0 };
	std::vector<Polygon2D> hull_g1{
		Polygon2D(hullv1, 7),
		Polygon2D(hullv2, 4)
	};
	std::vector<Polygon2D> hull_g1_calc(calcConvexHullGroups(pg_g1));
	EXPECT_TRUE(EXPECT_VECTOR_POINT2D_EQ(hull_g1[0].vertex, hull_g1_calc[0].vertex));
	EXPECT_TRUE(EXPECT_VECTOR_POINT2D_EQ(hull_g1[1].vertex, hull_g1_calc[1].vertex));
}