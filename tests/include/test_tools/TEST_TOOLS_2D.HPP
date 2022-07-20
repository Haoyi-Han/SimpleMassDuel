#ifndef TEST_TOOLS_HPP
#define TEST_TOOLS_HPP

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <iostream>
#include <vector>
#include <cmath>

#include "mass_point_v2/mass_point_2d.hpp"

inline ::testing::AssertionResult EXPECT_POINT2D_EQ(const Point2D& pA, const Point2D& pB)
{
	if (isNear(pA, pB))
		return ::testing::AssertionSuccess() << pA << " and " << pB << " are equal.";
	else
		return ::testing::AssertionFailure() << pA << " and " << pB << " are not equal.";
}

inline ::testing::AssertionResult EXPECT_COUPLEPOINT2D_EQ(const CouplePoint2D& cA, const CouplePoint2D& cB)
{
	if ((isNear(cA.pA, cB.pA) && isNear(cA.pB, cB.pB)) || (isNear(cA.pB, cB.pA) && isNear(cA.pA, cB.pB)))
		return ::testing::AssertionSuccess() << "Couples are equal.";
	if (isNear(cA.pA, cB.pA))
		return ::testing::AssertionFailure() << cA.pB << " and " << cB.pB << " are not equal.";
	if (isNear(cA.pA, cB.pB))
		return ::testing::AssertionFailure() << cA.pB << " and " << cB.pA << " are not equal.";
	if (isNear(cA.pB, cB.pA))
		return ::testing::AssertionFailure() << cA.pA << " and " << cB.pB << " are not equal.";
	if (isNear(cA.pB, cB.pB))
		return ::testing::AssertionFailure() << cA.pA << " and " << cB.pA << " are not equal.";
	return ::testing::AssertionFailure() << "The whole couples are not equal.";
}

inline ::testing::AssertionResult EXPECT_VECTOR_POINT2D_EQ(const std::vector<Point2D>& v1, const std::vector<Point2D>& v2)
{
	if (v1.size() != v2.size())
		return ::testing::AssertionFailure() << "size unequal.";
	for (int i = 0; i != v1.size(); i++) {
		if (!isNear(v1[i], v2[i]))
			return ::testing::AssertionFailure() << "vector[" << i << "] are not equal.";
	}
	return ::testing::AssertionSuccess() << "vectors are equal.";
}

#endif