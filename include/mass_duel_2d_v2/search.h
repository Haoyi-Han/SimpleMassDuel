#ifndef SEARCH_2D_H
#define SEARCH_2D_H

#include <iostream>
#include <vector>
#ifndef INF_F
#define INF_F std::numeric_limits<double>::infinity()
#endif

#include "mass_point_v2/mass_point_2d.hpp"

namespace VGraph
{
	struct Graph
	{
		std::vector<Point2D> vertex;
		std::vector<std::vector<double>> edge;
		explicit Graph(std::vector<Point2D> _vertex = {}, std::vector<std::vector<double>> _edge = {}) :
			vertex(_vertex), edge({}) {}
		// edge constructors
		void init_edge(const std::vector<std::vector<bool>>& adja);
		void init_edge();
		// print functions
		void printVertex();
		void printEdge();
	};
	
	RegularPolygon2D calcCircumRegularPolygonVertice(const Point2D& center, const double& r, const int& n);
	Graph createHinderGraph(const Point2D& origin, const Point2D& target, const std::vector<Point2D>& hinder_list, const double& safe_dist, const int& n);
	std::vector<Point2D> planPathFloydWarshall(Graph& g);
} // end namespace VisGraphPath

namespace FloydWarshall
{
	struct Solution
	{
		std::vector<std::vector<double>> dist;
		std::vector<std::vector<int>> next;

		explicit Solution(size_t num_V, std::vector<std::vector<double>>& graph)
		{
			init(num_V, graph);
		}
		explicit Solution(VGraph::Graph& g)
		{
			init(g.vertex.size(), g.edge);
		}

		void init(size_t num_V, const std::vector<std::vector<double>>& graph);
	};

	std::vector<int> constructPath(const Solution& s, int u, int v);
	void FloydWarshall(int num_V, Solution& s);
} // end namespace FloydWarshall

#endif