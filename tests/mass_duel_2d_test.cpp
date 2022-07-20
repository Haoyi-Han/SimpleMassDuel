#include "test_tools/test_tools_2d.hpp"
#include "mass_duel_2d_v2/search.h"
#include "mass_duel_2d_v2/hinder.h"

namespace SH = SimpleHinder;
namespace ST = SimpleTarget;
namespace SMC = SimpleMoveControl;
namespace VG = VGraph;
namespace FW = FloydWarshall;

TEST(isNecessaryChangeInterTarget1to2Test, Simple)
{
	EXPECT_TRUE(ST::isNeccessaryChangeInterTarget1to2(Point2D(40.7617, 36.3206), Point2D(44.1673, 39.9814), Point2D(39.0, 41.0), 0.5,
		std::vector<double>{3.0, 1.0}, 5.0, 1.5));
	std::cout << ST::calc2ndInterTarget(Point2D(40.7617, 36.3206), Point2D(44.1673, 39.9814), Point2D(39.0, 41.0), 5.0) << std::endl;
}

TEST(graphInitEdgeTest, Simple)
{
	std::vector<Point2D> v{
		Point2D(0.0, 0.0), Point2D(1.0, 0.0), Point2D(3.0, 0.0),
		Point2D(0.0, 1.0), Point2D(2.0, 1.0), Point2D(0.0, 2.0),
		Point2D(1.0, 3.0)};
	VG::Graph g(v);
	std::vector<std::vector<bool>> adja{
		std::vector<bool> {true, true, false, true, false, false, false},
		std::vector<bool> {true, true, true, false, true, false, false},
		std::vector<bool> {false, true, true, true, true, false, false},
		std::vector<bool> {true, false, true, true, false, true, true},
		std::vector<bool> {false, true, true, false, true, true, true},
		std::vector<bool> {false, false, false, true, true, true, true},
		std::vector<bool> {false, false, false, true, true, true, true}
	};
	g.init_edge(adja);
	std::cout << std::setprecision(4) << std::fixed;
	std::cout << "Vertices:" << std::endl;
	g.printVertex();
	std::cout << "\nEdges:" << std::endl;
	g.printEdge();
	EXPECT_DOUBLE_EQ(INF_F, g.edge[0][4]);
	EXPECT_DOUBLE_EQ(0.0, g.edge[3][3]);
	EXPECT_DOUBLE_EQ(std::sqrt(5), g.edge[4][6]);
	EXPECT_EQ(7, g.edge.size());
}

TEST(calcCircumRegularPolygonVerticeTest, Simple)
{
	// case 1: center at origin
	std::vector<Point2D> v1{
		Point2D(1.0, -1.0), Point2D(1.0, 1.0),
		Point2D(-1.0, 1.0), Point2D(-1.0, -1.0)
	};
	std::vector<Point2D> v2 = VG::calcCircumRegularPolygonVertice(Point2D(), 1.0, 4).vertex;
	EXPECT_TRUE(EXPECT_VECTOR_POINT2D_EQ(v1, v2));
	// case 2: center not at origin
	double l = 4 * std::tan(M_PI / 8);
	double R2 = std::sqrt(2);
	std::vector<Point2D> v3{
		Point2D(1.0 + (1.0 / R2), 0.0).scalar(l), Point2D((1.0 + R2), 1.0 / R2).scalar(l),
		Point2D((1.0 + R2), 1.0 + (1.0 / R2)).scalar(l), Point2D(1.0 + (1.0 / R2), (1.0 + R2)).scalar(l),
		Point2D(1.0 / R2, (1.0 + R2)).scalar(l), Point2D(0.0, 1.0 + (1.0 / R2)).scalar(l),
		Point2D(0.0, 1.0 / R2).scalar(l), Point2D(1.0 / R2, 0.0).scalar(l)
	};
	std::vector<Point2D> v4 = VG::calcCircumRegularPolygonVertice(Point2D(2.0, 2.0), 2.0, 8).vertex;
	EXPECT_TRUE(EXPECT_VECTOR_POINT2D_EQ(v3, v4));
}

TEST(FloydWarshallTest, Simple)
{
	std::vector<std::vector<double>> graph{
		std::vector<double> {0.0, 8.0, INF_F, 1.0},
		std::vector<double> {INF_F, 0.0, 1.0, INF_F},
		std::vector<double> {4.0, INF_F, 0.0, INF_F},
		std::vector<double> {INF_F, 2.0, 9.0, 0.0}
	};
	FW::Solution solu(4, graph);
	FW::FloydWarshall(4, solu);
	
	for (int i = 0; i != 4;i++) {
		for (int j = 0; j != 4; j++) {
			std::cout << solu.dist[i][j] << "\t";
		}
		std::cout << std::endl;
	}
	EXPECT_DOUBLE_EQ(7.0, solu.dist[3][0]);
	std::vector<int> path = FW::constructPath(solu, 3, 0);
	EXPECT_THAT(path, ::testing::ElementsAre(3, 1, 2, 0));
}