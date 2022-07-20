#include "mass_bt_nodes_v2/move_nodes_2d.h"
#include "mass_bt_nodes_v2/node_var_func_2d.hpp"
#include "mass_duel_2d_v2/hinder.h"

#include <iomanip>
#include <cstring>

namespace SH = SimpleHinder;
namespace ST = SimpleTarget;
namespace SMC = SimpleMoveControl;

namespace MassBTNodes2D
{
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
		// build predicted position after a tick time
		Point2D _predict_pos = _now_pos.move(_tick_time, _target, gearToSpeed(_gear));
		// select the hinder which is the most threated by score
		Point2D _hinder(SH::selectMostThreateningHinder(_now_pos, _predict_pos, _target, _hinder_list));
		// calculate some distances
		double real_dist = dist(_predict_pos, _hinder);
		double real_line_dist = dist(_hinder, _now_pos, _target);
		std::cout << "Distance from Hinder to Mass-Target Line:" << real_line_dist << std::endl;
		std::cout << "Distance Mass-Hinder:" << real_dist << std::endl;
		// encapcule some often-reused codes in the function below
		auto setGearCondition = [](std::string _gear, std::string new_gear)->std::string
		{
			std::string _gear_old = _gear;
			_gear = (_gear == "low") ? "low" : new_gear;
			std::string out_str = std::string("Hinder") + ((new_gear == "high") ? " not" : "")
				+ (" found. ") + ((_gear == _gear_old) ? "Keep " : "Set ") + _gear + " gear.";
			std::cout << out_str << std::endl;
			return _gear;
		};
		// reduce calculation by check if hinder is next to the progressive direction
		if (SH::isHinderTargetSameDirection(_now_pos, _target, _hinder))
		{
			_gear = setGearCondition(_gear, "high");
			setOutput("setgear", _gear);
			setOutput("setkeyhinder", pToString(_hinder));
			std::cout << "Angle > Orthogonal angle, hinder without threat." << std::endl;
			return BT::NodeStatus::FAILURE;
		}
		// check if predicted position of mass is *not* in the safe range of hinder
		if (!(SH::isPredictRouteHitHinder(_now_pos, _target, _hinder, _safe_dist)
			&& SH::isPosHitHinder(_predict_pos, _hinder, _safe_dist)))
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
		// calculate the 2nd inter-target
		Point2D _inter_target(ST::calc2ndInterTarget(_now_pos, _target, _hinder, _m2 * _safe_dist)); 
		// check if the new predicted position is in the hinder range
		std::cout << "distMassHinder - safedist  = " << dist(_now_pos, _hinder) - _safe_dist << std::endl;
		std::vector<double> speed_mid_low{ gearToSpeed("mid") , gearToSpeed("low") };
		if (!ST::isNeccessaryChangeInterTarget1to2(_now_pos, _target, _hinder, _tick_time, speed_mid_low, _safe_dist, _m1))
		{
			// change inter-target to the 1st inter-target
			_inter_target = ST::calc1stInterTarget(_now_pos, _target, _hinder, _m1 * _safe_dist);
			std::cout << "Change to first inter-target." << std::endl;
		}
		setOutput("settarget", pToString(_inter_target));
		std::cout << "Add an intertarget " << pToString2(_inter_target) << " to the current route." << std::endl;
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
		if (SMC::isNecessaryDecelerate(_now_pos, _inter_target, _tick_time, _speed, gearToSpeed("low")))
		{
			_speed = gearToSpeed("low");
			std::cout << "Predicted movement will be beyond the range. Set low gear." << std::endl;
		}
		_now_pos = _now_pos.move(_tick_time, _inter_target, _speed);
		printf("Now Position: [%.4f, %.4f]\n", _now_pos.x, _now_pos.y);
		setOutput("setpos", pToString(_now_pos));
		return BT::NodeStatus::SUCCESS;
	}
} // end namespace