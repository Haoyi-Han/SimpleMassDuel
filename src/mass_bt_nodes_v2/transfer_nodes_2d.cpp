#include "mass_bt_nodes_v2/transfer_nodes_2d.h"
#include "mass_point_v2/mass_point_2d.hpp"

#include <fstream>
#include <iomanip>
#include <cstring>

namespace MassBTNodes2D
{
	BT::NodeStatus CreateMass::tick()
	{
		Point2D _now_pos;
		if (!getInput<Point2D>("pos", _now_pos))
		{
			_now_pos = _origin;
			std::cout << "Create a mass for moving." << std::endl;
		}
		if (_SAVE)
		{
			std::ofstream _out;
			_out.open(_tracemass_filename, std::ios::app);
			_out << std::setprecision(4) << std::fixed;
			_out << pToString(_now_pos, false, true, " ") << std::endl;
			std::cout << "Write current position to file:" << _tracemass_filename << std::endl;
			_out.close();
		}
		setOutput("setpos", pToString(_now_pos));
		return BT::NodeStatus::SUCCESS;
	}

	BT::NodeStatus CreateInterTarget::tick()
	{
		Point2D _inter_target;
		if (!getInput<Point2D>("target", _inter_target))
		{
			_inter_target = _target;
			std::cout << "Create a target for moving." << std::endl;
		}
		setOutput("settarget", pToString(_inter_target));
		return BT::NodeStatus::SUCCESS;
	}

	BT::NodeStatus IsOnTarget::tick()
	{
		Point2D _now_pos;
		if (!getInput<Point2D>("pos", _now_pos))
		{
			throw BT::RuntimeError("missing required input [now_pos]");
		}
		double d(dist(_now_pos, _target));
		std::cout << "Distance Mass-Target:" << d << std::endl;
		if (d < _reach_dist)
		{
			std::cout << "Target reached successfully." << std::endl;
			return BT::NodeStatus::SUCCESS;
		}
		else
		{
			std::cout << "Still in course." << std::endl;
			return BT::NodeStatus::FAILURE;
		}
	}

	BT::NodeStatus IsCloseToTarget::tick()
	{
		Point2D _now_pos;
		if (!getInput<Point2D>("pos", _now_pos))
		{
			throw BT::RuntimeError("missing required input [now_pos]");
		}
		double d(dist(_now_pos, _target));
		if (d < _safe_dist)
		{
			setOutput("setgear", "low");
			std::cout << "Close to target. Set low gear." << std::endl;
			return BT::NodeStatus::SUCCESS;
		}
		else
		{
			setOutput("setgear", "high");
			std::cout << "Far from target. Set high gear." << std::endl;
			return BT::NodeStatus::FAILURE;
		}
	}

	BT::NodeStatus ResetInterTarget::tick()
	{
		setOutput("settarget", pToString(_target));
		setOutput("setgear", "high");
		std::cout << "Reset intertarget to original target." << std::endl;
		return BT::NodeStatus::SUCCESS;
	}
} // end namespace