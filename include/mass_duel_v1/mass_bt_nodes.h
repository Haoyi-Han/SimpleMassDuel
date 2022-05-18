#ifndef MASS_BT_NODES_H
#define MASS_BT_NODES_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "mass_duel_v1/mass_point.h"
#include <iostream>
#include <sstream>

namespace BT
{
	template <> inline Point2D convertFromString(StringView key)
	{
		auto parts = BT::splitString(key, ';');
		if (parts.size() != 4)
		{
			throw BT::RuntimeError("invalid error!");
		}
		else
		{
			//std::cout << parts[0] <<";"<< parts[1] << ";" << parts[2] << ";" << parts[3] << std::endl;
			Point2D output
			{
				convertFromString <double>(parts[0]),
				convertFromString <double>(parts[1]),
				convertFromString <double>(parts[2]),
				convertFromString <double>(parts[3])
			};
			return output;
		}
	}
} // end namespace BT

namespace MassBTNodes
{
	// define const variables
	const double speed_high(5.0); // per second
	const double speed_mid(3.0); // per second, for escaping from the hinder
	const double speed_low(1.0); // per second, for reaching the target
	const double tick_time(1.00); // time period for one tick
	const double safe_dist(5.0); // safe distance
	const double reach_dist(0.5); // distance for judging if reached 
	const double m(1.5); // amplification factor, m > 1
	const Point2D origin{ 0.0, 0.0, 5.0, 0.0 };
	const Point2D target{ 100.0, 100.0, 0.0, 0.0 };
	const Point2D hinder{ 39.0, 41.0, 0.0, 0.0 };
	extern std::string tracemass_filename;

	double convertSpeedFromGear(std::string gear);

	inline void SleepMS(int ms)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(ms));
	}

	class CreateMass : public BT::SyncActionNode
	{
	public:
		CreateMass(const std::string& name, const BT::NodeConfiguration& config) :
			BT::SyncActionNode(name, config) {}
		void init(Point2D origin, std::string tracemass_filename)
		{
			_origin = (origin);
			_SAVE = !(tracemass_filename.empty());
			if (_SAVE) _tracemass_filename = (tracemass_filename);
		}
		static BT::PortsList providedPorts()
		{
			return {
				BT::InputPort<Point2D>("pos"),
				BT::OutputPort<Point2D>("setpos")
			};
		}
		BT::NodeStatus tick() override;
	private:
		Point2D _origin;
		std::string _tracemass_filename;
		bool _SAVE;
	};

	class CreateInterTarget : public BT::SyncActionNode
	{
	public:
		CreateInterTarget(const std::string& name, const BT::NodeConfiguration& config) :
			BT::SyncActionNode(name, config) {}
		void init(Point2D target)
		{
			_target = (target);
		}
		static BT::PortsList providedPorts()
		{
			return {
				BT::InputPort<Point2D>("target"),
				BT::OutputPort<Point2D>("settarget")
			};
		}
		BT::NodeStatus tick() override;
	private:
		Point2D _target;
	};

	class IsOnTarget : public BT::ConditionNode
	{
	public:
		IsOnTarget(const std::string& name, const BT::NodeConfiguration& config) :
			BT::ConditionNode(name, config) {}
		void init(Point2D target, double reach_dist)
		{
			_target = (target);
			_reach_dist = (reach_dist);
		}
		static BT::PortsList providedPorts()
		{
			return { BT::InputPort<Point2D>("pos") };
		}
		BT::NodeStatus tick() override;

	private:
		Point2D _target;
		double _reach_dist;
	};

	class IsCloseToTarget : public BT::ConditionNode
	{
	public:
		IsCloseToTarget(const std::string& name, const BT::NodeConfiguration& config) :
			BT::ConditionNode(name, config) {}
		void init(Point2D target, double safe_dist)
		{
			_target = (target);
			_safe_dist = (safe_dist);
		}
		static BT::PortsList providedPorts()
		{
			return {
				BT::InputPort<Point2D>("pos"),
				BT::OutputPort<std::string>("setgear")
			};
		}
		BT::NodeStatus tick() override;

	private:
		Point2D _target;
		double _safe_dist;
	};

	class IsThereHinder : public BT::ConditionNode
	{
	public:
		IsThereHinder(const std::string& name, const BT::NodeConfiguration& config) :
			BT::ConditionNode(name, config) {}
		void init(Point2D target, Point2D hinder, double safe_dist, double m)
		{
			_target = (target);
			_hinder = (hinder);
			_safe_dist = (safe_dist);
			_m = (m);
		}
		static BT::PortsList providedPorts()
		{
			return {
				BT::InputPort<Point2D>("pos"),
				BT::InputPort<std::string>("gear"),
				BT::OutputPort<std::string>("setgear")
			};
		}
		BT::NodeStatus tick() override;

	private:
		Point2D _target;
		Point2D _hinder;
		double _safe_dist;
		double _m;
	};

	class ChangeInterTarget : public BT::SyncActionNode
	{
	public:
		ChangeInterTarget(const std::string& name, const BT::NodeConfiguration& config) :
			BT::SyncActionNode(name, config) {}
		void init(Point2D target, Point2D hinder, double safe_dist, double m)
		{
			_target = (target);
			_hinder = (hinder);
			_safe_dist = (safe_dist);
			_m = (m);
		}
		static BT::PortsList providedPorts()
		{
			return {
				BT::InputPort<Point2D>("pos"),
				BT::OutputPort<Point2D>("settarget")
			};
		}
		BT::NodeStatus tick() override;

	private:
		Point2D _target;
		Point2D _hinder;
		double _safe_dist;
		Point2D _inter_target;
		double _m;
	};

	class ResetInterTarget : public BT::SyncActionNode
	{
	public:
		ResetInterTarget(const std::string& name, const BT::NodeConfiguration& config) :
			BT::SyncActionNode(name, config) {}
		void init(Point2D target)
		{
			_target = (target);
		}
		static BT::PortsList providedPorts()
		{
			return {
				BT::OutputPort<Point2D>("settarget"),
				BT::OutputPort<std::string>("setgear")
			};
		}
		BT::NodeStatus tick() override;

	private:
		Point2D _target;
	};

	class MoveTo : public BT::SyncActionNode
	{
	public:
		MoveTo(const std::string& name, const BT::NodeConfiguration& config) :
			BT::SyncActionNode(name, config) {}
		void init(double tick_time)
		{
			_tick_time = (tick_time);
		}
		static BT::PortsList providedPorts()
		{
			return {
				BT::InputPort<Point2D>("pos"),
				BT::InputPort<std::string>("gear"),
				BT::InputPort<Point2D>("target"),
				BT::OutputPort<Point2D>("setpos")
			};
		}
		BT::NodeStatus tick() override;

	private:
		double _tick_time;
	};

	inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
	{
		factory.registerNodeType<CreateMass>("CreateMass");
		std::cout << "Register SyncActionNode CreateMass." << std::endl;
		factory.registerNodeType<CreateInterTarget>("CreateInterTarget");
		std::cout << "Register SyncActionNode CreateInterTarget." << std::endl;
		factory.registerNodeType<IsOnTarget>("IsOnTarget");
		std::cout << "Register ConditionNode IsOnTarget." << std::endl;
		factory.registerNodeType<IsCloseToTarget>("IsCloseToTarget");
		std::cout << "Register ConditionNode IsCloseToTarget." << std::endl;
		factory.registerNodeType<IsThereHinder>("IsThereHinder");
		std::cout << "Register ConditionNode IsThereHinder." << std::endl;
		factory.registerNodeType<ChangeInterTarget>("ChangeInterTarget");
		std::cout << "Register SyncActionNode ChangeInterTarget." << std::endl;
		factory.registerNodeType<ResetInterTarget>("ResetInterTarget");
		std::cout << "Register SyncActionNode ResetInterTarget." << std::endl;
		factory.registerNodeType<MoveTo>("MoveTo");
		std::cout << "Register SyncActionNode MoveTo." << std::endl;

		std::cout << "All Nodes Registered." << std::endl;
	}

	inline void InitNodes(BT::Tree& tree)
	{
		for (auto& node : tree.nodes)
		{
			// Not a typo: it is "=", not "=="
			if (auto node_CreateMass = dynamic_cast<CreateMass*>(node.get()))
			{
				node_CreateMass->init(origin, tracemass_filename);
				std::cout << "Node CreateMass Initialized." << std::endl;
			}
			else if (auto node_CreateInterTarget = dynamic_cast<CreateInterTarget*>(node.get()))
			{
				node_CreateInterTarget->init(target);
				std::cout << "Node CreateInterTarget Initialized." << std::endl;
			}
			else if (auto node_IsOnTarget = dynamic_cast<IsOnTarget*>(node.get()))
			{
				node_IsOnTarget->init(target, reach_dist);
				std::cout << "Node IsOnTarget Initialized." << std::endl;
			}
			else if (auto node_IsCloseToTarget = dynamic_cast<IsCloseToTarget*>(node.get()))
			{
				node_IsCloseToTarget->init(target, safe_dist);
				std::cout << "Node IsCloseToTarget Initialized." << std::endl;
			}
			else if (auto node_IsThereHinder = dynamic_cast<IsThereHinder*>(node.get()))
			{
				node_IsThereHinder->init(target, hinder, safe_dist, m);
				std::cout << "Node IsThereHinder Initialized." << std::endl;
			}
			else if (auto node_ChangeInterTarget = dynamic_cast<ChangeInterTarget*>(node.get()))
			{
				node_ChangeInterTarget->init(target, hinder, safe_dist, m);
				std::cout << "Node ChangeInterTarget Initialized." << std::endl;
			}
			else if (auto node_ResetInterTarget = dynamic_cast<ResetInterTarget*>(node.get()))
			{
				node_ResetInterTarget->init(target);
				std::cout << "Node ResetInterTarget Initialized." << std::endl;
			}
			else if (auto node_MoveTo = dynamic_cast<MoveTo*>(node.get()))
			{
				node_MoveTo->init(tick_time);
				std::cout << "Node MoveTo Initialized." << std::endl;
			}
		}
		std::cout << "All Nodes Initialized." << std::endl;
	}

} // end namespace MassBTNodes

#endif // MASS_BT_NODES_H