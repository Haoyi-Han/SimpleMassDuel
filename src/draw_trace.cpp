// Basic Lib
#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <cmath>
#include <cstring>
#include <boost/tuple/tuple.hpp>
#include "gnuplot-iostream.h"

// MassDuel Lib
#include "mass_duel_v1/mass_point.h"
#include "mass_duel_v1/mass_bt_nodes.h"

const int MAX_LINE_NUM(50);

// define enumerator of Point2D attributes
enum {x=0, y=1, v=2, theta=3};

int main()
{
	std::ifstream traceMassFile;
	std::string tracemass_filename = "./trace_mass.dat";
	traceMassFile.open(tracemass_filename);
	if (!traceMassFile.is_open())
	{
		std::cout << "Could not open the file " << tracemass_filename << "." << std::endl;
		exit(EXIT_FAILURE);
	}
	std::vector<std::array<double, 4>> trace_list;
	int count(0);
	std::string read_trace_line;
	while (traceMassFile.good())
	{
		trace_list.push_back(std::array<double, 4>{ 0.0, 0.0, 0.0, 0.0 });
		traceMassFile >> trace_list.back()[0] >> trace_list.back()[1] >> trace_list.back()[2] >> trace_list.back()[3];
		//trace_list.push_back(std::array<double, 4>{ 50.0, 50.0, 50.0, 50.0 });
		count++;
	}

	Gnuplot gp;
	std::vector<boost::tuple<double, double>> pts_mass;
	std::vector<boost::tuple<double, double>> pts_target;
	std::vector<boost::tuple<double, double>> pts_hinder;
	for (auto i = 0; i != count; i++)
	{
		pts_mass.push_back(boost::make_tuple(trace_list[i][0], trace_list[i][1]));
		pts_target.push_back(boost::make_tuple(MassBTNodes::target.x, MassBTNodes::target.y));
		pts_hinder.push_back(boost::make_tuple(MassBTNodes::hinder.x, MassBTNodes::hinder.y));
	}
	gp << "set xrange [-1:101]\nset yrange [-1:101]\n";
	gp << "plot '-' with lines title 'Mass', '-' with points title 'Target', '-' with points title 'Hinder'\n";
	gp << "set title 'Trajectory of Mass'\n";
	gp.send1d(pts_mass);
	gp.send1d(pts_target);
	gp.send1d(pts_hinder);
	
	return 0;
}