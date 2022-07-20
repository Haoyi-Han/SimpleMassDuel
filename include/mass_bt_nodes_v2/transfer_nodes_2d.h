#ifndef TRANSFER_NODES_2D_H
#define TRANSFER_NODES_2D_H

#include "mass_point_v2/mass_point_2d.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <vector>

namespace MassBTNodes2D
{
	class CreateMass : public BT::SyncActionNode
	{
	public:
		CreateMass(const std::string& name, const BT::NodeConfiguration& config,
			Point2D origin, std::string tracemass_filename) :
			BT::SyncActionNode(name, config),
			_origin(origin),
			_SAVE(!tracemass_filename.empty()),
			_tracemass_filename("")
		{
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
		CreateInterTarget(const std::string& name, const BT::NodeConfiguration& config, Point2D target) :
			BT::SyncActionNode(name, config), _target(target) {}
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
		IsOnTarget(const std::string& name, const BT::NodeConfiguration& config, Point2D target, double reach_dist) :
			BT::ConditionNode(name, config), _target(target), _reach_dist(reach_dist) {}
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
		IsCloseToTarget(const std::string& name, const BT::NodeConfiguration& config, Point2D target, double safe_dist) :
			BT::ConditionNode(name, config), _target(target), _safe_dist(safe_dist) {}
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

	class ResetInterTarget : public BT::SyncActionNode
	{
	public:
		ResetInterTarget(const std::string& name, const BT::NodeConfiguration& config, Point2D target) :
			BT::SyncActionNode(name, config), _target(target) {}
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

} // end namespace MassBTNodes2D

#endif // TRANSFER_NODES_2D_H