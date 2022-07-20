#include "mass_duel_2d_v2/search.h"

#include <cmath>
#include <limits>
#include <algorithm>

namespace VGraph
{
	namespace FW = FloydWarshall;

	void Graph::init_edge(const std::vector<std::vector<bool>>& adja)
	{
		edge.resize(vertex.size());
		for (auto& es : edge) {
			auto i = &es - &edge[0];
			edge[i].resize(vertex.size());
			for (auto& e : es) {
				auto j = &e - &es[0];
				if (adja[i][j])
					edge[i][j] = dist(vertex[i], vertex[j]);
				else
					edge[i][j] = INF_F;
			}
		}
	}
	void Graph::init_edge()
	{
		edge.resize(vertex.size());
		for (auto &es : edge) {
			auto i = &es - &edge[0];
			edge[i].resize(vertex.size());
			for (auto &e : es) {
				auto j = &e - &es[0];
				edge[i][j] = dist(vertex[i], vertex[j]);
			}
		}
	}
	void Graph::printVertex()
	{
		for (const Point2D& v : vertex) {
			std::cout << pToString(v, true, false, "\t") << std::endl;
		}
	}
	void Graph::printEdge()
	{
		for (const std::vector<double>& es : edge) {
			for (const double& e : es) {
				std::cout << e << "\t";
			}
			std::cout << std::endl;
		}
	}
	
	RegularPolygon2D calcCircumRegularPolygonVertice(const Point2D& center, const double& r, const int& n)
	{
		// n > 2 is the number of polygon edges
		// assume one edge of the circumscribed regular polygon is right below the circle and parallel to horizontal axis
		Point2D P0 = Point2D(r * std::tan(M_PI / n), -r) + center;
		return RegularPolygon2D(center, P0, n);
	}

	Graph createHinderGraph(const Point2D& origin, const Point2D& target, const std::vector<Point2D>& hinder_list, const double& safe_dist, const int& n)
	{
		Graph g;
		// create all vertices by calculating circumscribed regular polygon vertices of each hinder circle with radius = safe_dist
		g.vertex.push_back(origin);
		const size_t num_hinder = hinder_list.size();
		std::vector <int> hinder_index_list(num_hinder);
		for (auto& h : hinder_list)
		{
			auto i = &h - &hinder_list[0];
			RegularPolygon2D pg = calcCircumRegularPolygonVertice(h, safe_dist, n);
			g.vertex.insert(g.vertex.end(), pg.vertex.begin(), pg.vertex.end());
			hinder_index_list[i] = pg.n;
		}
		g.vertex.push_back(target);
		// initialize edge table and remove the edge of vertices from the same polygon
		g.init_edge();
		for (size_t k = 0; k != num_hinder - 1; k++) {
			for (int i = hinder_index_list[k] + 1; i != hinder_index_list[k + 1] + 1; i++) {
				for (int j = hinder_index_list[k] + 1; j != hinder_index_list[k + 1] + 1; j++) {
					g.edge[i][j] = INF_F;
				}
			}
		}
		return g;
	}

	std::vector<Point2D> planPathFloydWarshall(Graph& g)
	{
		const int num_V = (int)g.vertex.size();
		FW::Solution s(g);
		FW::FloydWarshall(num_V, s);
		std::vector<int> path_index = FW::constructPath(s, 0, num_V - 1);
		std::vector<Point2D> path;
		for (auto p : path_index) {
			path.push_back(g.vertex[p]);
		}		
		return path;
	}
}

namespace FloydWarshall
{
	void Solution::init(size_t num_V, const std::vector<std::vector<double>>& graph)
	{
		dist.resize(num_V);
		next.resize(num_V);
		for (auto& es : graph) {
			auto i = &es - &graph[0];
			dist[i].resize(num_V);
			next[i].resize(num_V);
			for (auto& e : es) {
				auto j = &e - &es[0];
				dist[i][j] = graph[i][j];
				if (graph[i][j] == INF_F)
					next[i][j] = -1;
				else
					next[i][j] = (int)j;
			}
		}
	}

	std::vector<int> constructPath(const Solution& s, int u, int v)
	{
		if (s.next[u][v] == -1)
			return {};
		std::vector<int> path{ u };
		while (u != v)
		{
			u = s.next[u][v];
			path.push_back(u);
		}
		return path;
	}

	void FloydWarshall(int num_V, Solution& s)
	{
		for (int k = 0; k != num_V; k++) {
			for (int i = 0; i != num_V; i++) {
				for (int j = 0; j != num_V; j++) {
					if ((s.dist[i][k] == INF_F) || (s.dist[k][j] == INF_F))
						continue;
					if (s.dist[i][j] > s.dist[i][k] + s.dist[k][j])
					{
						s.dist[i][j] = s.dist[i][k] + s.dist[k][j];
						s.next[i][j] = s.next[i][k];
					}
				}
			}
		}
	}
}