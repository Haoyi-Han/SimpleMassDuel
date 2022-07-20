#ifndef NODE_REG_INIT_2D_HPP
#define NODE_REG_INIT_2D_HPP

#include "mass_bt_nodes_v2/node_var_func_2d.hpp"
#include "mass_bt_nodes_v2/transfer_nodes_2d.h"
#include "mass_bt_nodes_v2/move_nodes_2d.h"

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

namespace MassBTNodes2D
{	
	inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
	{
		BT::NodeBuilder builder_CreateMass = [](const std::string& name, const BT::NodeConfiguration& config)
		{
			return std::make_unique<CreateMass>(name, config, origin, tracemass_filename);
		};
		BT::NodeBuilder builder_CreateInterTarget = [](const std::string& name, const BT::NodeConfiguration& config)
		{
			return std::make_unique<CreateInterTarget>(name, config, target);
		};
		BT::NodeBuilder builder_IsOnTarget = [](const std::string& name, const BT::NodeConfiguration& config)
		{
			return std::make_unique<IsOnTarget>(name, config, target, reach_dist);
		};
		BT::NodeBuilder builder_IsCloseToTarget = [](const std::string& name, const BT::NodeConfiguration& config)
		{
			return std::make_unique<IsCloseToTarget>(name, config, target, safe_dist);
		};
		BT::NodeBuilder builder_IsThereHinder = [](const std::string& name, const BT::NodeConfiguration& config)
		{
			return std::make_unique<IsThereHinder>(name, config, target, hinder_list, tick_time, safe_dist);
		};
		BT::NodeBuilder builder_ChangeInterTarget = [](const std::string& name, const BT::NodeConfiguration& config)
		{
			return std::make_unique<ChangeInterTarget>(name, config, target, safe_dist, m1, m2, tick_time);
		};
		BT::NodeBuilder builder_ResetInterTarget = [](const std::string& name, const BT::NodeConfiguration& config)
		{
			return std::make_unique<ResetInterTarget>(name, config, target);
		};
		BT::NodeBuilder builder_MoveTo = [](const std::string& name, const BT::NodeConfiguration& config)
		{
			return std::make_unique<MoveTo>(name, config, tick_time);
		};
		factory.registerBuilder<CreateMass>("CreateMass", builder_CreateMass);
		std::cout << "Register SyncActionNode CreateMass." << std::endl;
		factory.registerBuilder<CreateInterTarget>("CreateInterTarget", builder_CreateInterTarget);
		std::cout << "Register SyncActionNode CreateInterTarget." << std::endl;
		factory.registerBuilder<IsOnTarget>("IsOnTarget", builder_IsOnTarget);
		std::cout << "Register ConditionNode IsOnTarget." << std::endl;
		factory.registerBuilder<IsCloseToTarget>("IsCloseToTarget", builder_IsCloseToTarget);
		std::cout << "Register ConditionNode IsCloseToTarget." << std::endl;
		factory.registerBuilder<IsThereHinder>("IsThereHinder", builder_IsThereHinder);
		std::cout << "Register ConditionNode IsThereHinder." << std::endl;
		factory.registerBuilder<ChangeInterTarget>("ChangeInterTarget", builder_ChangeInterTarget);
		std::cout << "Register SyncActionNode ChangeInterTarget." << std::endl;
		factory.registerBuilder<ResetInterTarget>("ResetInterTarget", builder_ResetInterTarget);
		std::cout << "Register SyncActionNode ResetInterTarget." << std::endl;
		factory.registerBuilder<MoveTo>("MoveTo", builder_MoveTo);
		std::cout << "Register SyncActionNode MoveTo." << std::endl;

		std::cout << "All Nodes Registered and Initialized." << std::endl;
	}
} // end namespace MassBTNodes2D

#endif // NODE_REG_INIT_2D_HPP