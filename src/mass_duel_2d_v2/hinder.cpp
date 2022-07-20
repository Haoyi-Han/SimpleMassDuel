#include "mass_duel_2d_v2/hinder.h"

#include "mass_point_v2/mass_point_2d.hpp"
#include <vector>
#include <algorithm>

namespace SimpleHinder
{
	bool isHinderTargetSameDirection(const Point2D& pos, const Point2D& target, const Point2D& hinder)
	{
		return (std::cos(angle(target - pos, hinder - pos)) <= 0);
	}
	bool isPosHitHinder(const Point2D& pos, const Point2D& hinder, const double& hinder_safe_dist)
	{
		return (dist(pos, hinder) <= hinder_safe_dist);
	}
	bool isPredictRouteHitHinder(const Point2D& pos, const Point2D& target, const Point2D& hinder, const double& hinder_safe_dist)
	{
		return judgeSegmentCircleRelationship(Segment2D(pos, target), Circle2D(hinder, hinder_safe_dist)) > 0;
	}
	double calcHinderScore(const Point2D& pos, const Point2D& next_pos, const Point2D& target, const Point2D& hinder)
	{
		if (std::cos(angle(target - pos, hinder - pos)) <= 0) return 0.0;
		double normal_factor = dist(pos, target);
		double hinder_mass_dist = dist(next_pos, hinder) / normal_factor;
		double hinder_massline_dist = dist(hinder, pos, target) / normal_factor;
		return calcWeightedAvg(std::vector<double> { hinder_mass_dist, hinder_massline_dist },
			std::vector<double>{ 0.7, 0.3 });
	}
	Point2D selectMostThreateningHinder(const Point2D& pos, const Point2D& next_pos, const Point2D& target, const std::vector<Point2D>& hinder_list)
	{
		Point2D _hinder;
		double hinder_score, hinder_former_score(1.0);
		for (const Point2D& _hinder_check : hinder_list)
		{
			hinder_score = calcHinderScore(pos, next_pos, target, _hinder_check);
			if (isZero(hinder_score)) continue;
			if (hinder_score < hinder_former_score)
			{
				_hinder = _hinder_check;
				hinder_former_score = hinder_score;
			}
			std::cout << "Hinder {{ " << _hinder_check << " }} score: " << hinder_score << std::endl;
		}
		return _hinder;
	}
}

namespace SimpleTarget
{
	Point2D calc1stInterTarget(const Point2D& pos, const Point2D& target, const Point2D& hinder, const double& hinder_safe_dist)
	{
		Line2D l(pos, target);
		// initialize dx, dy as zero
		double dx(0.0), dy(0.0);
		if (!isZero(l.A))
		{
			dx = std::pow(std::pow(hinder_safe_dist, 2) / (1.0 + std::pow(l.B / l.A, 2)), 0.5);
			dy = (l.B / l.A) * dx;
		}
		else
		{
			dy = std::pow(std::pow(hinder_safe_dist, 2) / (1.0 + std::pow(l.A / l.B, 2)), 0.5);
			dx = (l.A / l.B) * dy;
		}
		Point2D p1(hinder.x - dx, hinder.y - dy);
		Point2D p2(hinder.x + dx, hinder.y + dy);
		return (isTwoPointBilateral(hinder, p1, l)) ? p1 : p2;
	}

	Point2D calc2ndInterTarget(const Point2D& pos, const Point2D& target, const Point2D& hinder, const double& hinder_safe_dist)
	{
		CouplePoint2D mass_hinder_TPs(calcTanPtsOnCircle(pos, hinder, hinder_safe_dist));
		Line2D l(pos, target);
		return (isTwoPointBilateral(hinder, mass_hinder_TPs.pA, l)) ? mass_hinder_TPs.pA : mass_hinder_TPs.pB;
	}

	bool isNeccessaryChangeInterTarget1to2(const Point2D& pos, const Point2D& target, const Point2D& hinder, 
		const double& tick_time, const std::vector<double>& speed_list, const double& hinder_safe_dist, const double& m)
	{
		Point2D inter_target(calc1stInterTarget(pos, target, hinder, m * hinder_safe_dist));
		std::vector<Point2D> predict_pos{};
		std::for_each(speed_list.begin(), speed_list.end(), [&](double v) {
			predict_pos.push_back(pos.move(tick_time, inter_target, v));
		});
		for (auto& ppos : predict_pos) {
			if (dist(ppos, hinder) < (m * hinder_safe_dist)) return true;
		}
		Segment2D sg(pos, inter_target);
		return judgeSegmentCircleRelationship(sg, Circle2D(hinder, hinder_safe_dist)) > 1;
	}
}

namespace SimpleMoveControl
{
	bool isNecessaryDecelerate(const Point2D& pos, const Point2D& target, const double& tick_time, const double& curr_speed, const double& lower_speed)
	{
		Point2D predict_move(pos.move(tick_time, target, curr_speed) - pos);
		Point2D target_move(target - pos);
		return (std::abs(predict_move.x) > std::abs(target_move.x)) || (std::abs(predict_move.y) > std::abs(target_move.y));
	}
}