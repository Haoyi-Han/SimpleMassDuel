#ifndef MATH_2D_H
#define MATH_2D_H

#include <cmath>

#ifndef INF_F
#define INF_F std::numeric_limits<double>::infinity()
#endif

#include <vector>

bool isZero(const double& x, double esp = 1e-6);
bool isNear(const double& a, const double& b, double esp = 1e-6);
bool isInRange(const double& m, const double& r1, const double& r2);
double atan2ZeroPrevent(double y, double x);
double avg(const std::vector<double>& vals);
double calcWeightedAvg(const std::vector<double>& vals, const std::vector<double>& weights);


#endif // MATH_2D_H