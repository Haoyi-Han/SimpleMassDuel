#ifndef NODE_VAR_FUNC_2D_HPP
#define NODE_VAR_FUNC_2D_HPP

#include "mass_point_v2/mass_point_2d.hpp"
#include <unordered_map>

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace BT
{
	template <> inline Point2D convertFromString(StringView key)
	{
		auto parts = BT::splitString(key, ';');
		if ((parts.size() != 2) && (parts.size() != 4)) {
			throw BT::RuntimeError("invalid error!");
		}
		else {
			if (parts.size() == 2) {
				return Point2D(
					convertFromString <double>(parts[0]),
					convertFromString <double>(parts[1])
				);
			}
			else {
				return Point2D(
					convertFromString <double>(parts[0]),
					convertFromString <double>(parts[1]),
					convertFromString <double>(parts[2]),
					convertFromString <double>(parts[3])
				);
			}
		}
	}
} // end namespace BT

namespace MassBTNodes2D
{
	inline namespace Var
	{
		// define const positions
		const Point2D origin{ 0.0, 0.0, 5.0, 0.0 };
		const Point2D target{ 100.0, 100.0, 0.0, 0.0 };
		const std::vector<Point2D> hinder_list{
			Point2D(39.0, 41.0),
			Point2D(61.0, 61.0),
			Point2D(80.0, 75.0)
		};
		// define const speeds and gear-speed converter
		const double speed_high(5.0); // per second
		const double speed_mid(3.0); // per second, for escaping from the hinder
		const double speed_low(1.0); // per second, for reaching the target
		struct Speed2D
		{
			double value;
			explicit Speed2D(double v = speed_high) : value(v) {}
		};
		const std::unordered_map<std::string, Speed2D> gear_speed
		{
			{"high", Speed2D(speed_high)},
			{"mid", Speed2D(speed_mid)},
			{"low", Speed2D(speed_low)}
		};
		//define const distances and factors
		const double safe_dist(5.0); // safe distance
		const double reach_dist(0.5); // distance for judging if reached 
		const double m1(1.5); // amplification factor for creating 1st inter-target, m > 1
		const double m2(1.01); // amplification factor for creating 2nd inter-target, m > 1
		//define extern variables
		extern double tick_time; // time period for one tick
		extern std::string tracemass_filename;
	}

	inline namespace Func
	{
		double gearToSpeed(std::string gear);

		inline void SleepMS(int ms)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(ms));
		}
	}
		
	inline namespace Tree
	{
		static std::string xml_text = R"(
<root main_tree_to_execute="MainTree">
	<BehaviorTree ID="MainTree">
		<Sequence>
			<Action ID="CreateMass" pos="{now_pos}" setpos="{now_pos}"/>
			<Action ID="CreateInterTarget" target="{inter_target}" settarget="{inter_target}"/>
			<Fallback name="main_control">
				<Condition ID="IsOnTarget" pos="{now_pos}"/>
				<ForceFailure>
					<Sequence name="move_and_set_target">
						<ForceSuccess>
							<Condition ID="IsCloseToTarget" pos="{now_pos}" setgear="{gear}"/>
						</ForceSuccess>
						<SubTree ID="MoveBase" now_pos="now_pos" gear="gear" inter_target="inter_target"/>
						<Action ID="ResetInterTarget" settarget="{inter_target}" setgear="high"/>
					</Sequence>
				</ForceFailure>
			</Fallback>
		</Sequence>
	</BehaviorTree>
	<BehaviorTree ID="MoveBase">
		<Sequence name="move_to_intertarget">
			<Fallback name="avoid_hinder">
				<Inverter>
					<Condition ID="IsThereHinder" pos="{now_pos}" gear="{gear}" setgear="{gear}" setkeyhinder="{key_hinder}"/>
				</Inverter>
				<Action ID="ChangeInterTarget" pos="{now_pos}" keyhinder="{key_hinder}" settarget="{inter_target}"/>
			</Fallback>
			<Action ID="MoveTo" pos="{now_pos}" gear="{gear}" target="{inter_target}" setpos="{now_pos}"/>
		</Sequence>
	</BehaviorTree>
</root>
		)";
	}
}

#endif // NODE_VAR_FUNC_2D_HPP