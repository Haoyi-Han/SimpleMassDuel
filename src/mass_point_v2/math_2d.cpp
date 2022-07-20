#include "mass_point_v2/math_2d.h"

#include <cmath>
#include <limits>
#include <numeric>

bool isZero(const double& x, double esp) { return std::abs(x) < esp; }
bool isNear(const double& a, const double& b, double esp) { return isZero(a - b, esp); }

bool isInRange(const double& m, const double& r1, const double& r2)
{
	// check if |m - r1| + |m - r2| == |r1 - r2|
	return isNear(std::abs(m - r1) + std::abs(m - r2), std::abs(r1 - r2));
}

double atan2ZeroPrevent(double y, double x)
{
	if (isZero(x) && isZero(y)) return 0.0;
	else if (isZero(x)) return (y > 0) ? INF_F : -INF_F;
	else return std::atan2(y, x);
}

double avg(const std::vector<double>& vals)
{
	return std::accumulate(vals.begin(), vals.end(), 0.0) / (double)(vals.size());
}

double calcWeightedAvg(const std::vector<double>& vals, const std::vector<double>& weights)
{
	return std::inner_product(vals.begin(), vals.end(), weights.begin(), 0.0) / (double)(vals.size());
}