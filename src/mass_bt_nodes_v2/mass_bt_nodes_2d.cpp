#include "mass_bt_nodes_v2/node_var_func_2d.hpp"
#include "mass_bt_nodes_v2/node_reg_init_2d.hpp"
#include "mass_point_v2/mass_point_2d.hpp"

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
	MassBTNodes2D::RegisterNodes(factory);
}

namespace MassBTNodes2D
{
	inline namespace Var
	{
		//define extern variables
		extern double tick_time(0.5); // time period for one tick
		extern std::string tracemass_filename("");
	}

	inline namespace Func
	{
		double gearToSpeed(std::string gear)
		{
			return gear_speed.at(gear).value;
		}
	}
}