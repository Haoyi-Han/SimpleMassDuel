#ifndef MOVE_NODES_2D_H
#define MOVE_NODES_2D_H

#include "mass_point_v2/mass_point_2d.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <vector>

namespace MassBTNodes2D
{
	class IsThereHinder : public BT::ConditionNode
	{
	public:
		IsThereHinder(const std::string& name, const BT::NodeConfiguration& config,
			Point2D target, std::vector<Point2D> hinder_list, double tick_time, double safe_dist) :
			BT::ConditionNode(name, config),
			_target(target),
			_hinder_list(hinder_list),
			_tick_time(tick_time),
			_safe_dist(safe_dist) {}
		static BT::PortsList providedPorts()
		{
			return {
				BT::InputPort<Point2D>("pos"),
				BT::InputPort<std::string>("gear"),
				BT::OutputPort<std::string>("setgear"),
				BT::OutputPort<Point2D>("setkeyhinder")
			};
		}
		BT::NodeStatus tick() override;

	private:
		Point2D _target;
		std::vector<Point2D> _hinder_list;
		double _tick_time;
		double _safe_dist;
	};

	class ChangeInterTarget : public BT::SyncActionNode
	{
	public:
		ChangeInterTarget(const std::string& name, const BT::NodeConfiguration& config,
			Point2D target, double safe_dist, double m1, double m2, double tick_time) :
			BT::SyncActionNode(name, config),
			_target(target),
			_safe_dist(safe_dist),
			_m1(m1),
			_m2(m2),
			_tick_time(tick_time) {}
		static BT::PortsList providedPorts()
		{
			return {
				BT::InputPort<Point2D>("pos"),
				BT::InputPort<Point2D>("keyhinder"),
				BT::OutputPort<Point2D>("settarget")
			};
		}
		BT::NodeStatus tick() override;

	private:
		Point2D _target;
		double _safe_dist;
		double _m1;
		double _m2;
		double _tick_time;
	};

	class MoveTo : public BT::SyncActionNode
	{
	public:
		MoveTo(const std::string& name, const BT::NodeConfiguration& config, double tick_time) :
			BT::SyncActionNode(name, config), _tick_time(tick_time) {}
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

} // end namespace MassBTNodes2D

#endif // MOVE_NODES_2D_H