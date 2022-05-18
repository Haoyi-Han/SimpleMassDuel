#include "mass_duel_v1/mass_bt_nodes.h"
#include "mass_duel_v1/mass_point.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <cstring>

// using namespace BT;
// using namespace MassBTNodes;

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
	MassBTNodes::RegisterNodes(factory);
}

namespace MassBTNodes
{
	std::string tracemass_filename = "";
	
	double convertSpeedFromGear(std::string gear)
	{
		if (gear == "high") return speed_high;
		else if (gear == "mid") return speed_mid;
		else if (gear == "low") return speed_low;
		// if wrong input, return the default value without throwing error
		else return speed_high;
	}

	BT::NodeStatus CreateMass::tick()
	{
		Point2D _now_pos;
		if (!getInput<Point2D>("pos", _now_pos))
		{
			_now_pos = _origin;
			std::cout << "Create a mass for moving." << std::endl;
		}
		auto _now_pos_str(pToString(_now_pos));
		if (_SAVE)
		{
			std::ofstream _out;
			_out.open(_tracemass_filename, std::ios::app);
			_out << std::setprecision(4) << std::fixed;
			_out << _now_pos << std::endl;
			std::cout << "Write current position to file:" << _tracemass_filename << std::endl;
			_out.close();
		}
		setOutput("setpos", _now_pos_str);
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
		auto _inter_target_str(pToString(_inter_target));
		setOutput("settarget", _inter_target_str);
		return BT::NodeStatus::SUCCESS;
	}

	BT::NodeStatus IsOnTarget::tick()
	{
		Point2D _now_pos;
		if (!getInput<Point2D>("pos", _now_pos))
		{
			throw BT::RuntimeError("missing required input [now_pos]");
		}
		auto dist = calcDistPoints(_now_pos, _target);
		std::cout << "Distance Mass-Target:" << dist << std::endl;
		if (dist < _reach_dist)
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
		auto dist = calcDistPoints(_now_pos, _target);
		if (dist < _safe_dist)
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

	BT::NodeStatus IsThereHinder::tick()
	{
		Point2D _now_pos;
		if (!getInput<Point2D>("pos", _now_pos))
		{
			throw BT::RuntimeError("missing required input [now_pos]");
		}
		std::string _gear;
		if (!getInput<std::string>("gear", _gear))
		{
			throw BT::RuntimeError("missing required input [gear]");
		}
		auto real_line_dist = [](Point2D pA, Point2D pB, Point2D pC)
		{
			auto l = calcLineEqCoeff(pA, pB);
			return CalcDistPointLine(pC, l);
		} (_now_pos, _target, _hinder);
		auto real_dist = calcDistPoints(_now_pos, _hinder);
		std::cout << "Distance from Hinder to Mass-Target Line:" << real_line_dist << std::endl;
		std::cout << "Distance Mass-Hinder:" << real_dist << std::endl;
		if ((real_line_dist < _m * _safe_dist) && (real_dist < _safe_dist))
		{
			if (_gear == "low")
			{
				setOutput("setgear", "low");
				std::cout << "Hinder found. Keep low gear." << std::endl;
			}
			else
			{
				setOutput("setgear", "mid");
				std::cout << "Hinder found. Set mid gear." << std::endl;
			}
			return BT::NodeStatus::SUCCESS;
		}
		else
		{
			if (_gear == "low")
			{
				setOutput("setgear", "low");
				std::cout << "Hinder not found. Keep low gear." << std::endl;
			}
			else
			{
				setOutput("setgear", "high");
				std::cout << "Hinder not found. Set high gear." << std::endl;
			}
			return BT::NodeStatus::FAILURE;
		}
	}

	BT::NodeStatus ChangeInterTarget::tick()
	{
		Point2D _now_pos;
		if (!getInput<Point2D>("pos", _now_pos))
		{
			throw BT::RuntimeError("missing required input [now_pos]");
		}
		auto target_route = calcLineEqCoeff(_now_pos, _target);
		auto dx = std::pow(std::pow(_m * _safe_dist, 2) / (1.0 + std::pow(target_route[1] / target_route[0], 2)), 0.5);
		auto l = calcLineEqCoeff(_now_pos, _target);
		auto isTwoPointBilateral = [](Point2D pA, Point2D pB, std::array <double, 3> l)
		{
			return ((l[0] * pA.x + l[1] * pA.y + l[2]) * (l[0] * pB.x + l[1] * pB.y + l[2]) < 0);
		};
		Point2D p_1{ _hinder.x - dx, _hinder.y - (l[1] / l[0]) * dx, 0.0, 0.0 };
		Point2D p_2{ _hinder.x + dx, _hinder.y + (l[1] / l[0]) * dx, 0.0, 0.0 };
		std::string _target_str;
		if (isTwoPointBilateral(_hinder, p_1, l))
		{
			_target_str = (pToString(p_1));	
		}
		else
		{
			_target_str = (pToString(p_2));
		}
		setOutput("settarget", _target_str);
		std::cout << "Add an intertarget to the current route." << std::endl;
		return BT::NodeStatus::SUCCESS;
	}

	BT::NodeStatus ResetInterTarget::tick()
	{
		auto _target_str(pToString(_target));
		setOutput("settarget", _target_str);
		setOutput("setgear", "high");
		std::cout << "Reset intertarget to original target." << std::endl;
		return BT::NodeStatus::SUCCESS;
	}

	BT::NodeStatus MoveTo::tick()
	{
		Point2D _now_pos;
		if (!getInput<Point2D>("pos", _now_pos))
		{
			throw BT::RuntimeError("missing required input [now_pos]");
		}
		std::string _gear;
		if (!getInput<std::string>("gear", _gear))
		{
			throw BT::RuntimeError("missing required input [gear]");
		}
		Point2D _inter_target;
		if (!getInput<Point2D>("target", _inter_target))
		{
			throw BT::RuntimeError("missing required input [target]");
		}
		double _speed(convertSpeedFromGear(_gear));
		double dx(_inter_target.x - _now_pos.x);
		double dy(_inter_target.y - _now_pos.y);
		double dist_now_target(std::pow(dx * dx + dy * dy, 0.5));
		double nvx(_speed * dx / dist_now_target);
		double nvy(_speed * dy / dist_now_target);
		_now_pos.x += _tick_time * nvx;
		_now_pos.y += _tick_time * nvy;
		_now_pos.v = _speed;
		_now_pos.theta = (dx * dy != 0.0) ? std::atan2(dy, dx) : 0.0;
		printf("Now Position: [%.1f, %.1f]\n", _now_pos.x, _now_pos.y);
		auto _now_pos_str(pToString(_now_pos));
		setOutput("setpos", _now_pos_str);
		return BT::NodeStatus::SUCCESS;
	}
} // end namespace