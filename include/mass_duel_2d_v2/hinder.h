#ifndef HINDER_2D_H
#define HINDER_2D_H

#include "mass_point_v2/mass_point_2d.hpp"

namespace SimpleHinder
{
	// conditions for checking if mass will hit hinder
	bool isHinderTargetSameDirection(const Point2D& pos, const Point2D& target, const Point2D& hinder);
	bool isPosHitHinder(const Point2D& pos, const Point2D& hinder, const double& hinder_safe_dist);
	bool isPredictRouteHitHinder(const Point2D& pos, const Point2D& target, const Point2D& hinder, const double& hinder_safe_dist);
	// function evaluating threat level by score
	double calcHinderScore(const Point2D& pos, const Point2D& next_pos, const Point2D& target, const Point2D& hinder);
	Point2D selectMostThreateningHinder(const Point2D& pos, const Point2D& next_pos, const Point2D& target, const std::vector<Point2D>& hinder_list);
}

namespace SimpleTarget
{
	Point2D calc1stInterTarget(const Point2D& pos, const Point2D& target, const Point2D& hinder, const double& hinder_safe_dist);
	Point2D calc2ndInterTarget(const Point2D& pos, const Point2D& target, const Point2D& hinder, const double& hinder_safe_dist);
	bool isNeccessaryChangeInterTarget1to2(const Point2D& pos, const Point2D& target, const Point2D& hinder,
		const double& tick_time, const std::vector<double>& speed_list, const double& hinder_safe_dist, const double& m);
}

namespace SimpleMoveControl
{
	bool isNecessaryDecelerate(const Point2D& pos, const Point2D& target, const double& tick_time, const double& curr_speed, const double& lower_speed);
}














#endif
