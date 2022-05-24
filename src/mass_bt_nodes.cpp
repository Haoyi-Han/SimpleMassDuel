#include "mass_duel_v1/mass_bt_nodes.h"
#include "mass_duel_v1/mass_point.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <cstring>
#include <unordered_map>

// using namespace BT;

const double ZERO_CHKPT(1e-9);

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
	MassBTNodes::RegisterNodes(factory);
}

namespace MassBTNodes
{
	double tick_time(1.00);
	std::string tracemass_filename = "";

	double gearToSpeed(std::string gear)
	{
		return gear_speed.at(gear).value;
	}

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
			_out << _now_pos.x << " " << _now_pos.y << " " << _now_pos.v * std::cos(_now_pos.theta) << " " << _now_pos.v * std::sin(_now_pos.theta) << std::endl;
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
		double dist = calcDistPoints(_now_pos, _target);
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
		double dist = calcDistPoints(_now_pos, _target);
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
		// define conditions for checking a hinder
		auto isHinderTargetSameDirection = [](Point2D pos, Point2D target, Point2D hinder)
		{
			return (std::cos(calcAngleVectors(target - pos, hinder - pos)) <= 0);
		};
		auto isPosHitHinder = [](Point2D pos, Point2D hinder, double hinder_safe_dist)
		{
			return (calcDistPoints(pos, hinder) <= hinder_safe_dist);
		};
		auto isPredictRouteHitHinder = [](Point2D pos, Point2D target, Point2D hinder, double hinder_safe_dist)
		{
			return (calcDistTriPoints(pos, target, hinder) <= hinder_safe_dist);
		};
		// select the hinder which is the most threated by score
		auto calcHinderScore = [](Point2D pos, Point2D target, Point2D hinder)
		{
			if (std::cos(calcAngleVectors(target - pos, hinder - pos)) <= 0) return 0.0;
			double normal_factor = calcDistPoints(pos, target);
			double hinder_mass_dist = calcDistPoints(pos, hinder) / normal_factor;
			double hinder_massline_dist = calcDistTriPoints(pos, target, hinder) / normal_factor;
			double factor_hinder_mass(0.7), factor_hinder_massline(0.3);
			return calcWeightedAvg(hinder_mass_dist, hinder_massline_dist, factor_hinder_mass, factor_hinder_massline);
		};
		_now_pos.v = gearToSpeed(_gear);
		Point2D _predict_pos = _now_pos.move(_tick_time);
		Point2D _hinder;
		double hinder_score, hinder_former_score(1.0);
		for (Point2D _hinder_check : _hinder_list)
		{
			hinder_score = calcHinderScore(_predict_pos, _target, _hinder_check);
			if (hinder_score == 0.0) continue;
			if (hinder_score < hinder_former_score)
			{
				_hinder = _hinder_check;
				hinder_former_score = hinder_score;
			}
			std::cout << "Hinder \{ " << _hinder_check << " \} score: " << hinder_score << std::endl;
		}
		// calculate some distances
		double real_dist = calcDistPoints(_predict_pos, _hinder);
		double real_line_dist = calcDistTriPoints(_now_pos, _target, _hinder);
		std::cout << "Distance from Hinder to Mass-Target Line:" << real_line_dist << std::endl;
		std::cout << "Distance Mass-Hinder:" << real_dist << std::endl;
		// encapcule some often-reused codes in the function below
		auto setGearCondition = [](std::string _gear, std::string new_gear)
		{
			std::string _gear_old = _gear;
			_gear = (_gear == "low") ? "low" : new_gear;
			std::string out_str = std::string("Hinder") + ((new_gear == "high") ? " not" : "") 
				+ (" found. ") + ((_gear == _gear_old) ? "Keep " : "Set ") + _gear + " gear.";
			std::cout << out_str << std::endl;
			return _gear;
		};
		// reduce calculation by check if hinder is next to the progressive direction
		if (isHinderTargetSameDirection(_now_pos, _target, _hinder))
		{
			_gear = setGearCondition(_gear, "high");
			setOutput("setgear", _gear);
			setOutput("setkeyhinder", pToString(_hinder));
			std::cout << "Angle > pi/2, hinder without threat." << std::endl;
			return BT::NodeStatus::FAILURE;
		}
		// check if predicted position of mass is *not* in the safe range of hinder 
		if (!(isPredictRouteHitHinder(_now_pos, _target, _hinder, _safe_dist) 
			&& isPosHitHinder(_predict_pos, _hinder, _safe_dist)))
		{
			_gear = setGearCondition(_gear, "high");
			setOutput("setgear", _gear);
			setOutput("setkeyhinder", pToString(_hinder));
			std::cout << "Predicted position not in hinder safe range." << std::endl;
			return BT::NodeStatus::FAILURE;
		}
		// default case: mass in the safe range of hinder
		_gear = setGearCondition(_gear, "mid");
		setOutput("setgear", _gear);
		setOutput("setkeyhinder", pToString(_hinder));
		std::cout << "Current position in hinder safe range." << std::endl;
		return BT::NodeStatus::SUCCESS;
	}

	BT::NodeStatus ChangeInterTarget::tick()
	{
		Point2D _now_pos;
		if (!getInput<Point2D>("pos", _now_pos))
		{
			throw BT::RuntimeError("missing required input [now_pos]");
		}
		Point2D _hinder;
		if (!getInput<Point2D>("keyhinder", _hinder))
		{
			throw BT::RuntimeError("missing required input [key_hinder]");
		}
		// calculate the inter-target defined by hinder and target_route
		auto target_route = calcLineEqCoeff(_now_pos, _target);
		Point2D p1, p2;
		auto l = calcLineEqCoeff(_now_pos, _target);
		auto isTwoPointBilateral = [](Point2D pA, Point2D pB, std::array <double, 3> l)
		{
			return ((l[0] * pA.x + l[1] * pA.y + l[2]) * (l[0] * pB.x + l[1] * pB.y + l[2]) < 0);
		};
		double dx, dy;
		if (std::abs(target_route[0]) < ZERO_CHKPT)
		{
			dx = 0; dy = _m * _safe_dist;
		}
		else
		{
			dx = std::pow(std::pow(_m * _safe_dist, 2) / (1.0 + std::pow(target_route[1] / target_route[0], 2)), 0.5);
			dy = (l[1] / l[0]) * dx;
		}
		p1 = { _hinder.x - dx, _hinder.y - dy, 0.0, 0.0 };
		p2 = { _hinder.x + dx, _hinder.y + dy, 0.0, 0.0 };
		Point2D _inter_target = (isTwoPointBilateral(_hinder, p1, l)) ? p1 : p2;
		// check if the new predicted position is in the hinder range
		Point2D _predict_pos = _now_pos.move(_tick_time, _now_pos.v, std::atan2(_inter_target.y - _now_pos.y, _inter_target.x - _now_pos.x));
		double predict_dist = calcDistPoints(_predict_pos, _hinder);
		std::cout << "safedist - distNowHnder = " << _safe_dist - calcDistPoints(_now_pos, _hinder) << std::endl;
		if (predict_dist < _m * _safe_dist)
		{
			// change inter-target to the intersection point of tangent and safe range
			auto mass_hinder_TPs = calcTanPointsOnCircle(_now_pos, _hinder, _safe_dist);
			_inter_target = (isTwoPointBilateral(_hinder, mass_hinder_TPs[0], l)) ? mass_hinder_TPs[0] : mass_hinder_TPs[1];
			std::cout << "Change to second inter-target." << std::endl;
		}
		setOutput("settarget", pToString(_inter_target));
		printf("Add an intertarget [%.2f, %.2f] to the current route.\n", _inter_target.x, _inter_target.y);
		return BT::NodeStatus::SUCCESS;
	}

	BT::NodeStatus ResetInterTarget::tick()
	{
		setOutput("settarget", pToString(_target));
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
		double _speed(gearToSpeed(_gear));
		double dx(_inter_target.x - _now_pos.x);
		double dy(_inter_target.y - _now_pos.y);
		double ntheta = (dx * dy != 0.0) ? std::atan2(dy, dx) : 0.0;
		if (std::abs(_speed * std::cos(ntheta) * _tick_time) > std::abs(dx))
		{
			_speed = gearToSpeed("low");
			std::cout << "Predicted movement will be beyond the range. Set low gear." << std::endl;
		}
		_now_pos = _now_pos.move(_tick_time, _speed, ntheta);
		printf("Now Position: [%.2f, %.2f]\n", _now_pos.x, _now_pos.y);
		setOutput("setpos", pToString(_now_pos));
		return BT::NodeStatus::SUCCESS;
	}
} // end namespace